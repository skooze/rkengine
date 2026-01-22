#include "rkg/instructions.h"

#include "rkg/log.h"

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

#include <filesystem>
#include <fstream>

namespace rkg {

#if RKG_ENABLE_DATA_JSON
namespace {
using json = nlohmann::json;

void apply_transform(ecs::Transform& transform, const json& data) {
  auto read_vec3 = [](const json& node, float out[3]) {
    if (node.is_array() && node.size() >= 3) {
      out[0] = node[0].get<float>();
      out[1] = node[1].get<float>();
      out[2] = node[2].get<float>();
    }
  };
  if (data.contains("position")) read_vec3(data["position"], transform.position);
  if (data.contains("rotation")) read_vec3(data["rotation"], transform.rotation);
  if (data.contains("scale")) read_vec3(data["scale"], transform.scale);
}

void execute_instruction(InstructionContext& ctx, const json& inst) {
  const auto action = inst.value("action", "");
  if (action == "spawn_entity") {
    const auto name = inst.value("name", "entity");
    ecs::Transform transform;
    if (inst.contains("transform")) {
      apply_transform(transform, inst["transform"]);
    }
    const auto entity = ctx.registry->create_entity();
    ctx.registry->set_transform(entity, transform);
    if (ctx.entities_by_name) {
      (*ctx.entities_by_name)[name] = entity;
    }
    rkg::log::info(std::string("spawn_entity: ") + name);
  } else if (action == "set_transform") {
    ecs::Entity entity = ecs::kInvalidEntity;
    if (inst.contains("entity_id")) {
      entity = inst["entity_id"].get<ecs::Entity>();
    } else if (inst.contains("name") && ctx.entities_by_name) {
      auto it = ctx.entities_by_name->find(inst["name"].get<std::string>());
      if (it != ctx.entities_by_name->end()) {
        entity = it->second;
      }
    }
    if (entity != ecs::kInvalidEntity) {
      if (auto* transform = ctx.registry->get_transform(entity)) {
        if (inst.contains("transform")) {
          apply_transform(*transform, inst["transform"]);
        }
      }
    }
  } else if (action == "set_component") {
    rkg::log::info("set_component: stub");
  } else if (action == "trigger_dialogue") {
    rkg::log::info("trigger_dialogue: stub");
  } else if (action == "load_level") {
    rkg::log::info("load_level: stub");
  } else if (action == "schedule_event") {
    rkg::log::info("schedule_event: stub");
  }
}
} // namespace
#endif

bool apply_runtime_instructions(const std::filesystem::path& path, InstructionContext& ctx) {
#if !RKG_ENABLE_DATA_JSON
  (void)path;
  (void)ctx;
  rkg::log::warn("runtime instructions disabled (JSON disabled)");
  return false;
#else
  if (!ctx.registry) {
    return false;
  }
  if (!std::filesystem::exists(path)) {
    return false;
  }

  std::ifstream in(path);
  if (!in) {
    rkg::log::warn("runtime instructions read failed");
    return false;
  }

  json doc;
  in >> doc;
  const auto& list = doc.contains("instructions") ? doc["instructions"] : doc;
  if (!list.is_array()) return false;

  for (const auto& inst : list) {
    execute_instruction(ctx, inst);
  }
  return true;
#endif
}

} // namespace rkg
