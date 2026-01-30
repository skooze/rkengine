#include "rkg/ecs.h"
#include "rkg/ai_results.h"
#include "rkg/log.h"
#include "rkg/math.h"
#include "rkg/host_context.h"
#include "rkg/input.h"
#include "rkg/plugin_api.h"
#include "rkg/paths.h"
#include "rkg/run_cleanup.h"
#include "rkg/snapshot_restore.h"
#include "rkg/staged_runs.h"
#include "rkg/renderer_select.h"
#include "rkg/renderer_util.h"
#include "rkg/renderer_hooks.h"
#include "rkg/asset_import.h"
#include "rkg/asset_cache.h"
#include "rkgctl/cli_api.h"

#include <nlohmann/json.hpp>

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <optional>
#include <cstring>

namespace fs = std::filesystem;
using json = nlohmann::json;

#if RKG_ENABLE_PHYSICS_BASIC
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_physics_basic(uint32_t host_api_version);
#endif

bool write_text(const fs::path& path, const std::string& contents) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  if (!out) return false;
  out << contents;
  return true;
}

bool write_bytes(const fs::path& path, const std::vector<uint8_t>& data) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path, std::ios::binary);
  if (!out) return false;
  out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
  return true;
}

uint64_t fnv1a_64_bytes(const std::vector<uint8_t>& data) {
  uint64_t hash = 1469598103934665603ULL;
  for (uint8_t b : data) {
    hash ^= b;
    hash *= 1099511628211ULL;
  }
  return hash;
}

uint64_t hash_file(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return 0;
  std::vector<uint8_t> data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return fnv1a_64_bytes(data);
}

std::vector<fs::path> list_files_sorted(const fs::path& root) {
  std::vector<fs::path> out;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (entry.is_regular_file()) {
      out.push_back(fs::relative(entry.path(), root));
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

bool write_minimal_glb(const fs::path& path) {
  // Minimal GLB with one triangle (positions + indices).
  const std::string json =
      R"({"asset":{"version":"2.0"},"buffers":[{"byteLength":42}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36,"target":34962},{"buffer":0,"byteOffset":36,"byteLength":6,"target":34963}],"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0]},{"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"}],"meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}],"nodes":[{"mesh":0}],"scenes":[{"nodes":[0]}],"scene":0})";

  // Positions (3 vertices) + indices (uint16)
  std::vector<uint8_t> bin;
  bin.resize(42);
  auto write_f32 = [&](size_t offset, float value) {
    std::memcpy(bin.data() + offset, &value, sizeof(float));
  };
  write_f32(0, 0.0f);
  write_f32(4, 0.0f);
  write_f32(8, 0.0f);
  write_f32(12, 1.0f);
  write_f32(16, 0.0f);
  write_f32(20, 0.0f);
  write_f32(24, 0.0f);
  write_f32(28, 1.0f);
  write_f32(32, 0.0f);
  uint16_t indices[3] = {0, 1, 2};
  std::memcpy(bin.data() + 36, indices, sizeof(indices));

  auto pad4 = [](size_t size) { return (size + 3) & ~size_t(3); };
  const size_t json_padded = pad4(json.size());
  const size_t bin_padded = pad4(bin.size());
  const uint32_t total_length = static_cast<uint32_t>(
      12 + 8 + json_padded + 8 + bin_padded);

  std::vector<uint8_t> glb;
  glb.resize(total_length);
  auto write_u32 = [&](size_t offset, uint32_t value) {
    std::memcpy(glb.data() + offset, &value, sizeof(uint32_t));
  };

  // Header
  write_u32(0, 0x46546C67);  // 'glTF'
  write_u32(4, 2);
  write_u32(8, total_length);

  size_t offset = 12;
  write_u32(offset + 0, static_cast<uint32_t>(json_padded));
  write_u32(offset + 4, 0x4E4F534A);  // 'JSON'
  std::memcpy(glb.data() + offset + 8, json.data(), json.size());
  for (size_t i = json.size(); i < json_padded; ++i) {
    glb[offset + 8 + i] = ' ';
  }

  offset += 8 + json_padded;
  write_u32(offset + 0, static_cast<uint32_t>(bin_padded));
  write_u32(offset + 4, 0x004E4942);  // 'BIN\0'
  std::memcpy(glb.data() + offset + 8, bin.data(), bin.size());
  for (size_t i = bin.size(); i < bin_padded; ++i) {
    glb[offset + 8 + i] = 0;
  }

  return write_bytes(path, glb);
}

std::string read_text(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

int main(int argc, char** argv) {
  const auto paths = rkg::resolve_paths(argc > 0 ? argv[0] : nullptr, std::nullopt, "demo_game");
  rkg::log::init("rkg_tests", paths.root);

  int failures = 0;

  // Test: plan validation.
  {
    json plan;
    plan["tasks"] = json::array();
    plan["tasks"].push_back({{"id", "t1"}, {"type", "content_change"}});
    std::string error;
    if (!validate_plan(plan, error)) {
      std::cerr << "plan validation failed unexpectedly\n";
      ++failures;
    }
    json bad;
    if (validate_plan(bad, error)) {
      std::cerr << "plan validation should have failed\n";
      ++failures;
    }
  }

#if RKG_ENABLE_DATA_JSON
  // Test: cook_status parsing (schema presence).
  {
    const std::string status_text =
        R"({"cook_ok":true,"last_success_timestamp":"2024-01-01T00:00:00","content_index_path":"out/content.index.json","content_pack_path":"out/content.pack","last_error":""})";
    json status = json::parse(status_text);
    if (!status.value("cook_ok", false)) {
      std::cerr << "cook_status cook_ok missing\n";
      ++failures;
    }
    if (status.value("last_success_timestamp", "").empty()) {
      std::cerr << "cook_status last_success_timestamp missing\n";
      ++failures;
    }
    if (status.value("content_index_path", "").empty() || status.value("content_pack_path", "").empty()) {
      std::cerr << "cook_status paths missing\n";
      ++failures;
    }
  }
#endif

  // Test: renderer fallback order.
  {
    const auto order = rkg::build_renderer_fallback_order("d3d12");
    if (order.empty() || order.front() != "renderer_d3d12") {
      std::cerr << "renderer fallback order invalid\n";
      ++failures;
    }
    bool has_null = false;
    for (const auto& item : order) {
      if (item == "renderer_null") {
        has_null = true;
      }
    }
    if (!has_null) {
      std::cerr << "renderer fallback missing null\n";
      ++failures;
    }
  }

  // Test: renderer display name mapping.
  {
    if (rkg::renderer_display_name("renderer_vulkan") != "Vulkan") {
      std::cerr << "renderer display name (vulkan) invalid\n";
      ++failures;
    }
    if (rkg::renderer_display_name("renderer_d3d12") != "D3D12") {
      std::cerr << "renderer display name (d3d12) invalid\n";
      ++failures;
    }
    if (rkg::renderer_display_name("renderer_null") != "Null") {
      std::cerr << "renderer display name (null) invalid\n";
      ++failures;
    }
  }

  // Test: viewport camera + line list hooks.
  {
    float view_proj[16]{};
    view_proj[0] = 1.0f;
    view_proj[5] = 1.0f;
    view_proj[10] = 1.0f;
    view_proj[15] = 1.0f;
    rkg::set_vulkan_viewport_camera(view_proj);
    const auto* camera = rkg::get_vulkan_viewport_camera();
    if (!camera || camera->view_proj[0] != 1.0f || camera->view_proj[15] != 1.0f) {
      std::cerr << "viewport camera set/get failed\n";
      ++failures;
    }

    float positions[6] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    float colors[4] = {1.0f, 0.0f, 0.0f, 1.0f};
    rkg::set_vulkan_viewport_line_list(positions, colors, 1);
    const auto* lines = rkg::get_vulkan_viewport_line_list();
    if (!lines || lines->line_count != 1 || lines->positions[3] != 1.0f) {
      std::cerr << "viewport line list set/get failed\n";
      ++failures;
    }
  }

  // Test: asset import missing file.
  {
    rkg::asset::ImportOptions options{};
    const auto result =
        rkg::asset::import_glb("does_not_exist.glb", "build/test_import_missing", options);
    if (result.ok || result.error.empty()) {
      std::cerr << "asset import missing-file should fail\n";
      ++failures;
    }
  }

  // Test: asset import determinism on tiny glb fixture.
  {
    const fs::path temp_root = fs::temp_directory_path() / "rkg_import_test";
    const fs::path glb_path = temp_root / "fixture.glb";
    const fs::path out1 = temp_root / "out1";
    const fs::path out2 = temp_root / "out2";
    std::error_code ec;
    fs::remove_all(temp_root, ec);
    fs::create_directories(temp_root);
    if (!write_minimal_glb(glb_path)) {
      std::cerr << "failed to write minimal glb\n";
      ++failures;
    } else {
      rkg::asset::ImportOptions options{};
      options.overwrite = true;
      const auto res1 = rkg::asset::import_glb(glb_path, out1, options);
      const auto res2 = rkg::asset::import_glb(glb_path, out2, options);
      if (!res1.ok || !res2.ok) {
        std::cerr << "asset import failed on minimal glb\n";
        ++failures;
      } else {
        const fs::path asset_json = out1 / "asset.json";
        const fs::path mesh_bin = out1 / "mesh.bin";
        const fs::path materials_json = out1 / "materials.json";
        if (!fs::exists(asset_json) || fs::file_size(asset_json) == 0) {
          std::cerr << "asset.json missing or empty\n";
          ++failures;
        }
        if (!fs::exists(mesh_bin) || fs::file_size(mesh_bin) == 0) {
          std::cerr << "mesh.bin missing or empty\n";
          ++failures;
        }
        if (!fs::exists(materials_json) || fs::file_size(materials_json) == 0) {
          std::cerr << "materials.json missing or empty\n";
          ++failures;
        }
        const auto files1 = list_files_sorted(out1);
        const auto files2 = list_files_sorted(out2);
        if (files1 != files2) {
          std::cerr << "asset import outputs differ (file list)\n";
          ++failures;
        } else {
          for (const auto& rel : files1) {
            const uint64_t h1 = hash_file(out1 / rel);
            const uint64_t h2 = hash_file(out2 / rel);
            if (h1 != h2) {
              std::cerr << "asset import outputs differ (hash) for " << rel.string() << "\n";
              ++failures;
              break;
            }
          }
        }
      }
    }
    fs::remove_all(temp_root, ec);
  }

  // Test: asset cache load from minimal import output.
  {
    const fs::path temp_root = fs::temp_directory_path() / "rkg_asset_cache_test";
    const fs::path content_root = temp_root / "content";
    const fs::path glb_path = temp_root / "fixture.glb";
    const fs::path asset_dir = content_root / "assets" / "fixture";
    std::error_code ec;
    fs::remove_all(temp_root, ec);
    fs::create_directories(content_root / "assets");
    if (!write_minimal_glb(glb_path)) {
      std::cerr << "failed to write minimal glb for asset cache\n";
      ++failures;
    } else {
      rkg::asset::ImportOptions options{};
      options.overwrite = true;
      const auto res = rkg::asset::import_glb(glb_path, asset_dir, options);
      if (!res.ok) {
        std::cerr << "asset import failed for asset cache\n";
        ++failures;
      } else {
        rkg::runtime::AssetCache cache;
        std::string error;
        if (!cache.load_from_content_root(content_root, error)) {
          std::cerr << "asset cache load failed: " << error << "\n";
          ++failures;
        } else if (!cache.find("fixture")) {
          std::cerr << "asset cache missing fixture\n";
          ++failures;
        }
#if RKG_ENABLE_DATA_JSON
        // Inject a tiny skeleton + skin and ensure asset cache loads it.
        const std::string skeleton_json =
            R"({"joint_count":2,"bones":[)"
            R"({"name":"root","parent":-1,"bind_local":{"position":[0,0,0],"rotation":[0,0,0],"scale":[1,1,1]},"local_pose":{"position":[0,0,0],"rotation":[0,0,0],"scale":[1,1,1]}},)"
            R"({"name":"child","parent":0,"bind_local":{"position":[0,1,0],"rotation":[0,0,0],"scale":[1,1,1]},"local_pose":{"position":[0,1,0],"rotation":[0,0,0],"scale":[1,1,1]}}]})";
        if (!write_text(asset_dir / "skeleton.json", skeleton_json)) {
          std::cerr << "failed to write skeleton.json\n";
          ++failures;
        }

        std::vector<uint8_t> skin_bytes;
        auto push_u32 = [&](uint32_t value) {
          const size_t offset = skin_bytes.size();
          skin_bytes.resize(offset + sizeof(uint32_t));
          std::memcpy(skin_bytes.data() + offset, &value, sizeof(uint32_t));
        };
        auto push_f32 = [&](float value) {
          const size_t offset = skin_bytes.size();
          skin_bytes.resize(offset + sizeof(float));
          std::memcpy(skin_bytes.data() + offset, &value, sizeof(float));
        };
        push_u32(0x4E494B53); // 'SKIN'
        push_u32(1);
        push_u32(2);
        push_u32(0);
        const float identity[16] = {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        };
        for (int i = 0; i < 16; ++i) push_f32(identity[i]);
        for (int i = 0; i < 16; ++i) push_f32(identity[i]);
        if (!write_bytes(asset_dir / "skin.bin", skin_bytes)) {
          std::cerr << "failed to write skin.bin\n";
          ++failures;
        }

        rkg::runtime::AssetCache cache_with_skel;
        std::string skel_error;
        if (!cache_with_skel.load_from_content_root(content_root, skel_error)) {
          std::cerr << "asset cache reload failed: " << skel_error << "\n";
          ++failures;
        } else {
          const auto* record = cache_with_skel.find("fixture");
          if (!record) {
            std::cerr << "asset cache missing fixture after skeleton\n";
            ++failures;
          } else if (record->skeleton.bones.size() != 2 ||
                     record->skeleton.inverse_bind_mats.size() != 2) {
            std::cerr << "asset cache skeleton/skin load failed\n";
            ++failures;
          }
        }
#endif
      }
    }
    fs::remove_all(temp_root, ec);
  }

  // Test: ECS components and iteration helpers.
  {
    rkg::ecs::Registry registry;
    const auto entity = registry.create_entity();
    rkg::ecs::Transform transform{};
    transform.position[0] = 1.0f;
    registry.set_transform(entity, transform);
    if (!registry.get_transform(entity)) {
      std::cerr << "transform set/get failed\n";
      ++failures;
    }

    rkg::ecs::Velocity velocity{};
    velocity.linear[2] = 3.0f;
    registry.set_velocity(entity, velocity);
    if (!registry.get_velocity(entity)) {
      std::cerr << "velocity set/get failed\n";
      ++failures;
    }
    registry.remove_velocity(entity);
    if (registry.get_velocity(entity)) {
      std::cerr << "velocity remove failed\n";
      ++failures;
    }

    rkg::ecs::RigidBody body{};
    body.mass = 2.0f;
    registry.set_rigid_body(entity, body);
    if (!registry.get_rigid_body(entity)) {
      std::cerr << "rigid body set/get failed\n";
      ++failures;
    }
    registry.remove_rigid_body(entity);
    if (registry.get_rigid_body(entity)) {
      std::cerr << "rigid body remove failed\n";
      ++failures;
    }

    rkg::ecs::Collider collider{};
    collider.type = rkg::ecs::ColliderType::Capsule;
    registry.set_collider(entity, collider);
    if (!registry.get_collider(entity)) {
      std::cerr << "collider set/get failed\n";
      ++failures;
    }

    rkg::ecs::CharacterController controller{};
    controller.max_speed = 7.0f;
    registry.set_character_controller(entity, controller);
    if (!registry.get_character_controller(entity)) {
      std::cerr << "character controller set/get failed\n";
      ++failures;
    }

    rkg::ecs::Skeleton skeleton{};
    rkg::ecs::Bone bone{};
    bone.name = "root";
    skeleton.bones.push_back(bone);
    registry.set_skeleton(entity, skeleton);
    if (!registry.get_skeleton(entity)) {
      std::cerr << "skeleton set/get failed\n";
      ++failures;
    }

    int collider_count = 0;
    for (const auto& kv : registry.colliders()) {
      if (kv.first == entity) {
        collider_count += 1;
      }
    }
    if (collider_count != 1) {
      std::cerr << "collider iteration failed\n";
      ++failures;
    }

    int controller_count = 0;
    for (const auto& kv : registry.character_controllers()) {
      if (kv.first == entity) {
        controller_count += 1;
      }
    }
    if (controller_count != 1) {
      std::cerr << "character controller iteration failed\n";
      ++failures;
    }

    registry.destroy_entity(entity);
    if (registry.get_collider(entity) || registry.get_character_controller(entity) ||
        registry.get_skeleton(entity)) {
      std::cerr << "destroy_entity did not clear components\n";
      ++failures;
    }
  }

  // Test: skeleton world pose compute.
  {
    rkg::ecs::Transform root{};
    root.position[0] = 1.0f;
    root.position[1] = 0.0f;
    root.position[2] = 0.0f;
    rkg::ecs::Skeleton skeleton{};
    rkg::ecs::Bone bone_root{};
    bone_root.name = "root";
    bone_root.parent_index = -1;
    bone_root.local_pose.position[0] = 0.0f;
    bone_root.local_pose.position[1] = 0.0f;
    bone_root.local_pose.position[2] = 0.0f;
    skeleton.bones.push_back(bone_root);
    rkg::ecs::Bone bone_child{};
    bone_child.name = "child";
    bone_child.parent_index = 0;
    bone_child.local_pose.position[0] = 0.0f;
    bone_child.local_pose.position[1] = 1.0f;
    bone_child.local_pose.position[2] = 0.0f;
    skeleton.bones.push_back(bone_child);

    rkg::ecs::compute_skeleton_world_pose(root, skeleton);
    if (skeleton.world_pose.size() != 2) {
      std::cerr << "skeleton world pose size wrong\n";
      ++failures;
    } else {
      const auto& root_world = skeleton.world_pose[0];
      const auto& child_world = skeleton.world_pose[1];
      if (std::abs(root_world.position[0] - 1.0f) > 0.001f ||
          std::abs(root_world.position[1] - 0.0f) > 0.001f) {
        std::cerr << "skeleton root world pos wrong\n";
        ++failures;
      }
      if (std::abs(child_world.position[0] - 1.0f) > 0.001f ||
          std::abs(child_world.position[1] - 1.0f) > 0.001f) {
        std::cerr << "skeleton child world pos wrong\n";
        ++failures;
      }
    }
  }

#if RKG_ENABLE_PHYSICS_BASIC
  // Test: physics_basic plugin updates character controller.
  {
    rkg::ecs::Registry registry;
    const auto entity = registry.create_entity();
    rkg::ecs::Transform transform{};
    transform.position[1] = 1.5f;
    registry.set_transform(entity, transform);
    rkg::ecs::CharacterController controller{};
    controller.grounded = true;
    registry.set_character_controller(entity, controller);

    struct ActionStub {
      bool forward = false;
      bool jump = false;
      bool sprint = false;
      rkg::input::ActionState get(const char* action) const {
        rkg::input::ActionState state{};
        if (std::strcmp(action, "MoveForward") == 0) {
          state.held = forward;
        } else if (std::strcmp(action, "Jump") == 0) {
          state.pressed = jump;
        } else if (std::strcmp(action, "Sprint") == 0) {
          state.held = sprint;
        }
        return state;
      }
    };

    ActionStub stub{};
    stub.forward = true;
    stub.jump = true;

    rkg::HostContext ctx{};
    ctx.registry = &registry;
    ctx.get_action_state = [](void* user, const char* action) -> rkg::input::ActionState {
      return static_cast<ActionStub*>(user)->get(action);
    };
    ctx.action_state_user = &stub;

    auto* api = rkg_plugin_get_api_physics_basic(rkg::kRkgPluginApiVersion);
    if (!api || !api->init || !api->shutdown || !api->update) {
      std::cerr << "physics_basic api missing\n";
      ++failures;
    } else {
      if (!api->init(&ctx)) {
        std::cerr << "physics_basic init failed\n";
        ++failures;
      } else {
        const auto* before = registry.get_transform(entity);
        const float before_z = before ? before->position[2] : 0.0f;
        api->update(1.0f / 60.0f);
        const auto* after = registry.get_transform(entity);
        if (!after || after->position[2] <= before_z) {
          std::cerr << "physics_basic did not move forward\n";
          ++failures;
        }
        const auto* vel = registry.get_velocity(entity);
        if (!vel || vel->linear[1] <= 0.0f) {
          std::cerr << "physics_basic jump failed\n";
          ++failures;
        }

        rkg::ecs::Transform ground_test{};
        ground_test.position[1] = 0.0f;
        registry.set_transform(entity, ground_test);
        if (auto* ctrl = registry.get_character_controller(entity)) {
          ctrl->grounded = false;
        }
        if (auto* v = registry.get_velocity(entity)) {
          v->linear[1] = -1.0f;
        }
        api->update(1.0f / 60.0f);
        const auto* grounded = registry.get_transform(entity);
        if (!grounded || grounded->position[1] < (controller.half_height + controller.radius) - 0.001f) {
          std::cerr << "physics_basic grounding failed\n";
          ++failures;
        }

        api->shutdown();
      }
    }
  }
#endif

#if RKG_ENABLE_DATA_JSON
  // Test: plan schema file exists.
  {
    const fs::path schema_path = paths.root / "docs" / "schemas" / "rkg_plan.schema.json";
    const auto schema_text = read_text(schema_path);
    if (schema_text.empty()) {
      std::cerr << "plan schema missing\n";
      ++failures;
    } else {
      const auto schema = json::parse(schema_text, nullptr, false);
      if (schema.is_discarded() || !schema.contains("properties")) {
        std::cerr << "plan schema invalid\n";
        ++failures;
      }
    }
  }

  // Test: run_info/results schema docs exist and include expected fields.
  {
    const fs::path run_info_path = paths.root / "docs" / "schemas" / "rkg_run_info.schema.json";
    const fs::path run_results_path = paths.root / "docs" / "schemas" / "rkg_run_results.schema.json";
    const auto run_info_text = read_text(run_info_path);
    const auto run_results_text = read_text(run_results_path);
    if (run_info_text.empty() || run_results_text.empty()) {
      std::cerr << "run_info/results schema missing\n";
      ++failures;
    } else {
      const auto run_info = json::parse(run_info_text, nullptr, false);
      const auto run_results = json::parse(run_results_text, nullptr, false);
      if (run_info.is_discarded() || run_results.is_discarded()) {
        std::cerr << "run_info/results schema invalid\n";
        ++failures;
      } else {
        const auto info_props = run_info.value("properties", json::object());
        const auto res_props = run_results.value("properties", json::object());
        if (!info_props.contains("run_id") || !info_props.contains("mode") ||
            !info_props.contains("provider") || !info_props.contains("model")) {
          std::cerr << "run_info schema missing fields\n";
          ++failures;
        }
        if (!res_props.contains("run_id") || !res_props.contains("status") ||
            !res_props.contains("context_drift")) {
          std::cerr << "run_results schema missing fields\n";
          ++failures;
        }
      }
    }
  }

  // Test: OpenAI request builder + response extraction.
  {
    json schema;
    schema["type"] = "object";
    schema["properties"]["tasks"] = json::object();
    schema["required"] = json::array({"tasks"});
    json req;
    std::string error;
    if (!build_openai_request_json(schema, "gpt-5.2-codex", "sys", "user", req, error)) {
      std::cerr << "openai request build failed\n";
      ++failures;
    } else {
      if (!req.contains("text")) {
        std::cerr << "openai request missing text.format\n";
        ++failures;
      }
    }

    json response;
    response["output_text"] = "{\"tasks\":[]}";
    std::string plan_json;
    if (!extract_plan_json_from_openai_response(response, plan_json, error)) {
      std::cerr << "openai response extract failed\n";
      ++failures;
    }
    json response2;
    response2["output"] = json::array(
        {{{"content", json::array({{{"type", "output_text"}, {"text", "{\"tasks\":[]}"}}})}}});
    if (!extract_plan_json_from_openai_response(response2, plan_json, error)) {
      std::cerr << "openai response extract (output content) failed\n";
      ++failures;
    }
    json response3;
    response3["choices"] = json::array({{{"message", {{"content", "{\"tasks\":[]}"}}}}});
    if (!extract_plan_json_from_openai_response(response3, plan_json, error)) {
      std::cerr << "openai response extract (choices) failed\n";
      ++failures;
    }
    json response4;
    response4["output"] = json::array(
        {{{"tool_calls", json::array({{{"arguments", "{\"tasks\":[]}"}}})}}});
    if (!extract_plan_json_from_openai_response(response4, plan_json, error)) {
      std::cerr << "openai response extract (tool_calls) failed\n";
      ++failures;
    }
  }

  // Test: plan validation error count.
  {
    json bad;
    bad["tasks"] = json::array({json::object()});
    std::vector<PlanValidationError> errors;
    if (validate_plan_detailed(bad, errors, true)) {
      std::cerr << "bad plan unexpectedly valid\n";
      ++failures;
    }
    if (errors.size() < 1) {
      std::cerr << "plan errors missing\n";
      ++failures;
    }
  }

  // Test: strict enum validation catches invalid task type.
  {
    json bad;
    bad["tasks"] = json::array({{{"id", "t1"}, {"type", "bad_type"}}});
    std::vector<PlanValidationError> errors;
    if (validate_plan_detailed(bad, errors, true)) {
      std::cerr << "enum validation should have failed\n";
      ++failures;
    } else {
      bool found = false;
      for (const auto& err : errors) {
        if (err.keypath.find(".type") != std::string::npos &&
            err.message.find("allowed") != std::string::npos) {
          found = true;
          break;
        }
      }
      if (!found) {
        std::cerr << "enum validation error missing allowed values\n";
        ++failures;
      }
    }
  }

  // Test: context dump includes fingerprint and diff runs.
  {
    const fs::path tmp_project = paths.root / "build" / "test_ctx_project";
    fs::remove_all(tmp_project);
    fs::create_directories(tmp_project / "content");
    fs::create_directories(tmp_project / "config");
    write_text(tmp_project / "project.yaml", "project:\n  name: ctx_test\n");
    write_text(tmp_project / "content/a.txt", "a");
    write_text(tmp_project / "content/b.txt", "b");
    write_text(tmp_project / "config/input.yaml", "input:\n  actions:\n    Quit: Escape\n");
    const fs::path baseline = tmp_project / "baseline.json";
    if (context_dump(argv[0], tmp_project, baseline, 1) != 0) {
      std::cerr << "context dump failed\n";
      ++failures;
    } else {
      const auto text = read_text(baseline);
      const auto ctx = json::parse(text, nullptr, false);
      if (ctx.is_discarded() || !ctx.contains("fingerprint")) {
        std::cerr << "context fingerprint missing\n";
        ++failures;
      }
      if (!ctx.contains("files") || !ctx.contains("files_cap")) {
        std::cerr << "context files list missing\n";
        ++failures;
      } else {
        const size_t cap = ctx.value("files_cap", 0);
        if (cap == 0 || !ctx["files"].is_array() || ctx["files"].size() > cap) {
          std::cerr << "context files cap invalid\n";
          ++failures;
        }
        if (!ctx.value("files_truncated", false)) {
          std::cerr << "context files should be truncated\n";
          ++failures;
        }
        const size_t total = ctx.value("files_total_count", 0);
        const size_t included = ctx.value("files_included_count", 0);
        if (total == 0 || included == 0 || included > total) {
          std::cerr << "context files counts invalid\n";
          ++failures;
        }
      }
      if (!ctx.contains("core_files") || !ctx.contains("plugins_files")) {
        std::cerr << "context core/plugins file lists missing\n";
        ++failures;
      } else {
        const size_t core_cap = ctx.value("core_files_cap", 0);
        const size_t plugins_cap = ctx.value("plugins_files_cap", 0);
        if (core_cap == 0 || plugins_cap == 0 ||
            !ctx["core_files"].is_array() || !ctx["plugins_files"].is_array() ||
            ctx["core_files"].size() > core_cap || ctx["plugins_files"].size() > plugins_cap) {
          std::cerr << "context core/plugins caps invalid\n";
          ++failures;
        }
        const size_t core_total = ctx.value("core_files_total_count", 0);
        const size_t core_included = ctx.value("core_files_included_count", 0);
        const size_t plugins_total = ctx.value("plugins_files_total_count", 0);
        const size_t plugins_included = ctx.value("plugins_files_included_count", 0);
        if ((core_total > 0 && core_included == 0) || core_included > core_total ||
            (plugins_total > 0 && plugins_included == 0) || plugins_included > plugins_total) {
          std::cerr << "context core/plugins counts invalid\n";
          ++failures;
        }
      }
      if (context_diff(argv[0], baseline, tmp_project, false, false, std::optional<size_t>{1}) != 0) {
        std::cerr << "context diff failed\n";
        ++failures;
      }
    }
  }

  // Test: agent run summary parsing.
  {
    const fs::path run_dir = paths.root / "build" / "test_ai_run_status" / "run_1";
    fs::remove_all(run_dir);
    fs::create_directories(run_dir);
    json run_info;
    run_info["run_id"] = "run_1";
    run_info["created_at"] = "2024-01-01T00:00:00";
    run_info["goal"] = "test goal";
    run_info["mode"] = "apply";
    run_info["plan_path"] = (run_dir / "plan.json").generic_string();
    run_info["provider"] = "openai";
    run_info["model"] = "gpt-5.2-codex";
    run_info["base_url"] = "https://api.openai.com";
    run_info["templates_dir"] = "docs/agent_templates/openai";
    run_info["timeout_seconds"] = 60;
    run_info["strict_enums"] = true;
    write_text(run_dir / "run_info.json", run_info.dump(2));

    json results;
    results["run_id"] = "run_1";
    results["status"] = "ok";
    results["dry_run"] = false;
    results["conflicts"] = 0;
    results["context_drift"]["drift_detected"] = false;
    write_text(run_dir / "results.json", results.dump(2));

    json summary;
    std::string error;
    if (!load_run_summary(run_dir, summary, error)) {
      std::cerr << "run summary failed: " << error << "\n";
      ++failures;
    } else {
      if (summary.value("run_id", "") != "run_1" || summary.value("goal", "").empty()) {
        std::cerr << "run summary missing fields\n";
        ++failures;
      }
      if (!summary.contains("context_drift")) {
        std::cerr << "run summary missing drift info\n";
        ++failures;
      }
    }
  }

  // Test: drift info is written to results.json.
  {
    const fs::path run_dir = paths.root / "build" / "test_ai_results" / "run_2";
    fs::remove_all(run_dir);
    fs::create_directories(run_dir);
    ApplyOptions opts;
    opts.dry_run = false;
    opts.require_approval = false;
    json drift;
    drift["baseline_fingerprint"] = json::object();
    drift["current_fingerprint"] = json::object();
    drift["drift_detected"] = true;
    drift["changed_counts"]["added"] = 1;
    drift["changed_counts"]["removed"] = 0;
    drift["changed_counts"]["modified"] = 2;
    write_ai_results_with_drift(run_dir, "run_2", opts, 0, drift);
    const auto results_text = read_text(run_dir / "results.json");
    const auto parsed = json::parse(results_text, nullptr, false);
    if (parsed.is_discarded() || !parsed.contains("context_drift")) {
      std::cerr << "results missing drift info\n";
      ++failures;
    }
  }

  // Test: ai_results summary parsing.
  {
    const std::string text =
        R"({"run_id":"r1","status":"ok","dry_run":false,"conflicts":0,"context_drift":{"drift_detected":true,"changed_counts":{"added":1,"removed":2,"modified":3},"message":"tree hash mismatch"}})";
    rkg::AiResultsSummary summary{};
    std::string error;
    if (!rkg::parse_ai_results_summary(text, summary, error)) {
      std::cerr << "ai_results parse failed: " << error << "\n";
      ++failures;
    } else {
      if (!summary.drift_detected || summary.drift_added != 1 || summary.drift_removed != 2 ||
          summary.drift_modified != 3) {
        std::cerr << "ai_results drift parse mismatch\n";
        ++failures;
      }
    }
  }
#endif

#if RKG_ENABLE_DATA_YAML
  // Test: YAML update helper.
  {
    YAML::Node root;
    root["project"]["initial_level"] = "old";
    json value = "new_level.yaml";
    update_yaml_keypath(root, "project.initial_level", value);
    const auto updated = root["project"]["initial_level"];
    if (!updated || !updated.IsScalar()) {
      std::cerr << "yaml update failed (non-scalar)\n";
      ++failures;
    } else if (updated.Scalar() != "new_level.yaml") {
      std::cerr << "yaml update failed\n";
      ++failures;
    }
  }
#endif

#if RKG_ENABLE_DATA_YAML
  // Test: pack determinism (same input -> same pack bytes).
  {
    const fs::path tmp_project = paths.root / "build" / "test_pack_project";
    fs::remove_all(tmp_project);
    fs::create_directories(tmp_project / "content" / "prefabs");
    fs::create_directories(tmp_project / "content" / "levels");
    write_text(tmp_project / "project.yaml", "project:\n  name: pack_test\n");
    write_text(tmp_project / "content/prefabs/prefab.yaml",
               "name: prefab\ncomponents: {}\n");
    write_text(tmp_project / "content/levels/level.yaml",
               "name: level\nentities:\n  - name: e1\n    prefab: prefab\n");

    const fs::path out_a = tmp_project / "out_a";
    const fs::path out_b = tmp_project / "out_b";
    if (content_cook(argv[0], tmp_project, out_a) != 0 ||
        content_cook(argv[0], tmp_project, out_b) != 0) {
      std::cerr << "content cook failed\n";
      ++failures;
    } else {
      const auto pack_a = read_text(out_a / "content.pack");
      const auto pack_b = read_text(out_b / "content.pack");
      if (pack_a.empty() || pack_b.empty() || pack_a != pack_b) {
        std::cerr << "pack determinism failed\n";
        ++failures;
      }
    }
  }
#endif

#if RKG_ENABLE_DATA_YAML
  // Test: content validation (missing prefab ref).
  {
    const fs::path tmp_project = paths.root / "build" / "test_tmp_project";
    fs::remove_all(tmp_project);
    fs::create_directories(tmp_project / "content" / "prefabs");
    fs::create_directories(tmp_project / "content" / "levels");
    write_text(tmp_project / "project.yaml", "project:\n  name: test_project\n");
    write_text(tmp_project / "content/prefabs/prefab.yaml",
               "name: prefab\ncomponents: {}\n");
    write_text(tmp_project / "content/levels/level.yaml",
               "name: level\nentities:\n  - name: e1\n    prefab: missing_prefab\n");
    if (content_validate(argv[0], tmp_project) == 0) {
      std::cerr << "content validation should have failed\n";
      ++failures;
    }
  }
#endif

  // Test: patch apply + conflict detection.
  {
    const fs::path tmp_dir = paths.root / "build" / "test_tmp_patches";
    fs::remove_all(tmp_dir);
    fs::create_directories(tmp_dir);
    const fs::path plan_path = tmp_dir / "plan.json";
    json plan;
    plan["tasks"] = json::array();
    plan["tasks"].push_back({
        {"id", "t1"},
        {"type", "content_change"},
        {"actions",
         json::array({{{"action", "write_text_file"},
                       {"path", "build/test_tmp_patches/file.txt"},
                       {"mode", "overwrite"},
                       {"contents", "hello\n"}}})}});
    write_text(plan_path, plan.dump(2));

    const StageResult stage = stage_plan(plan, plan_path);
    if (!stage.errors.empty() || stage.files.empty()) {
      std::cerr << "stage_plan failed\n";
      ++failures;
    } else {
      // Apply should succeed.
      ApplyOptions opts;
      opts.dry_run = false;
      opts.require_approval = false;
      const int rc_ok = apply_staged(stage, opts);
      if (rc_ok != 0) {
        std::cerr << "apply_staged failed unexpectedly\n";
        ++failures;
      }

      // Conflict check: modify file after staging and re-apply.
      write_text(paths.root / "build/test_tmp_patches/file.txt", "modified\n");
      const int rc_conflict = apply_staged(stage, opts);
      if (rc_conflict == 0) {
        std::cerr << "conflict was not detected\n";
        ++failures;
      }
    }
  }

#if RKG_ENABLE_DATA_YAML
  // Test: commit overrides dry-run staging.
  {
    const fs::path tmp_project = paths.root / "build" / "test_tmp_commit";
    fs::remove_all(tmp_project);
    fs::create_directories(tmp_project / "content" / "prefabs");
    fs::create_directories(tmp_project / "content" / "levels");
    write_text(tmp_project / "project.yaml",
               "project:\n  name: commit_test\n  initial_level: content/levels/level.yaml\n");
    write_text(tmp_project / "content/prefabs/prefab.yaml",
               "name: prefab\ncomponents: {}\n");
    write_text(tmp_project / "content/levels/level.yaml",
               "name: level\nentities:\n  - name: e1\n    id: e1\n    prefab: prefab\n    transform:\n"
               "      position: [0, 0, 0]\n      rotation: [0, 0, 0]\n      scale: [1, 1, 1]\n"
               "    renderable:\n      mesh: cube\n      color: [1, 1, 1, 1]\n");
    write_text(tmp_project / "editor_overrides.yaml",
               "overrides:\n  e1:\n    transform:\n      position: [1, 2, 3]\n"
               "    renderable:\n      mesh: quad\n      color: [0.2, 0.3, 0.4, 1.0]\n");

    const std::string run_id = "test_commit_stage";
    const fs::path run_dir = tmp_project / "run";
    const fs::path staging_dir = run_dir / "staged_patches";
    std::string error_code;
    std::string error_message;
    const StageResult stage =
        stage_commit_overrides(paths.root, tmp_project, tmp_project / "editor_overrides.yaml",
                               std::nullopt, std::nullopt, std::nullopt, staging_dir,
                               run_id, error_code, error_message);
    if (!error_message.empty() || !stage.errors.empty() || stage.files.empty()) {
      std::cerr << "commit overrides staging failed\n";
      ++failures;
    } else {
      const auto diff_text = read_text(stage.files.front().diff_path);
      if (diff_text.find("renderable") == std::string::npos ||
          diff_text.find("quad") == std::string::npos ||
          diff_text.find("position") == std::string::npos) {
        std::cerr << "commit overrides diff missing expected edits\n";
        ++failures;
      }
    }
  }

  // Test: commit overrides stage selected entity only.
  {
    const fs::path tmp_project = paths.root / "build" / "test_tmp_commit_selected";
    fs::remove_all(tmp_project);
    fs::create_directories(tmp_project / "content" / "prefabs");
    fs::create_directories(tmp_project / "content" / "levels");
    write_text(tmp_project / "project.yaml",
               "project:\n  name: commit_selected\n  initial_level: content/levels/level.yaml\n");
    write_text(tmp_project / "content/prefabs/prefab.yaml",
               "name: prefab\ncomponents: {}\n");
    write_text(tmp_project / "content/levels/level.yaml",
               "name: level\nentities:\n"
               "  - name: e1\n    id: e1\n    prefab: prefab\n    transform:\n"
               "      position: [0, 0, 0]\n      rotation: [0, 0, 0]\n      scale: [1, 1, 1]\n"
               "  - name: e2\n    id: e2\n    prefab: prefab\n    transform:\n"
               "      position: [0, 0, 0]\n      rotation: [0, 0, 0]\n      scale: [1, 1, 1]\n");
    write_text(tmp_project / "editor_overrides.yaml",
               "overrides:\n"
               "  e1:\n    transform:\n      position: [1, 2, 3]\n"
               "  e2:\n    transform:\n      position: [9, 9, 9]\n");

    const std::string run_id = "test_commit_selected";
    const fs::path run_dir = tmp_project / "run";
    const fs::path staging_dir = run_dir / "staged_patches";
    std::string error_code;
    std::string error_message;
    const StageResult stage =
        stage_commit_overrides(paths.root, tmp_project, tmp_project / "editor_overrides.yaml",
                               std::nullopt, std::optional<std::string>("e1"), std::nullopt, staging_dir,
                               run_id, error_code, error_message);
    if (!error_message.empty() || !stage.errors.empty() || stage.files.empty()) {
      std::cerr << "commit overrides selected staging failed\n";
      ++failures;
    } else {
      const auto diff_text = read_text(stage.files.front().diff_path);
      if (diff_text.find("position: [1, 2, 3]") == std::string::npos ||
          diff_text.find("position: [9, 9, 9]") != std::string::npos) {
        std::cerr << "commit overrides selected diff unexpected\n";
        ++failures;
      }
      if (stage.summary.value("entity_id", "") != "e1") {
        std::cerr << "commit overrides selected summary missing entity_id\n";
        ++failures;
      }
    }
  }

  // Test: commit overrides stage selected by name.
  {
    const fs::path tmp_project = paths.root / "build" / "test_tmp_commit_selected_name";
    fs::remove_all(tmp_project);
    fs::create_directories(tmp_project / "content" / "prefabs");
    fs::create_directories(tmp_project / "content" / "levels");
    write_text(tmp_project / "project.yaml",
               "project:\n  name: commit_selected_name\n  initial_level: content/levels/level.yaml\n");
    write_text(tmp_project / "content/prefabs/prefab.yaml",
               "name: prefab\ncomponents: {}\n");
    write_text(tmp_project / "content/levels/level.yaml",
               "name: level\nentities:\n"
               "  - name: e1\n    prefab: prefab\n    transform:\n"
               "      position: [0, 0, 0]\n      rotation: [0, 0, 0]\n      scale: [1, 1, 1]\n"
               "  - name: e2\n    prefab: prefab\n    transform:\n"
               "      position: [0, 0, 0]\n      rotation: [0, 0, 0]\n      scale: [1, 1, 1]\n");
    write_text(tmp_project / "editor_overrides.yaml",
               "overrides:\n"
               "  e1:\n    transform:\n      position: [1, 2, 3]\n"
               "  e2:\n    transform:\n      position: [9, 9, 9]\n");

    const std::string run_id = "test_commit_selected_name";
    const fs::path run_dir = tmp_project / "run";
    const fs::path staging_dir = run_dir / "staged_patches";
    std::string error_code;
    std::string error_message;
    const StageResult stage =
        stage_commit_overrides(paths.root, tmp_project, tmp_project / "editor_overrides.yaml",
                               std::nullopt, std::nullopt, std::optional<std::string>("e2"),
                               staging_dir, run_id, error_code, error_message);
    if (!error_message.empty() || !stage.errors.empty() || stage.files.empty()) {
      std::cerr << "commit overrides selected-name staging failed\n";
      ++failures;
    } else {
      const auto diff_text = read_text(stage.files.front().diff_path);
      if (diff_text.find("position: [9, 9, 9]") == std::string::npos ||
          diff_text.find("position: [1, 2, 3]") != std::string::npos) {
        std::cerr << "commit overrides selected-name diff unexpected\n";
        ++failures;
      }
      const auto warning = stage.summary.value("selector_warning", "");
      if (warning.empty()) {
        std::cerr << "commit overrides selected-name warning missing\n";
        ++failures;
      }
      if (stage.summary.value("selector_type", "") != "name") {
        std::cerr << "commit overrides selected-name selector_type missing\n";
        ++failures;
      }
    }
  }

  // Test: commit overrides conflict detection.
  {
    const fs::path tmp_project = paths.root / "build" / "test_tmp_commit_conflict";
    fs::remove_all(tmp_project);
    fs::create_directories(tmp_project / "content" / "prefabs");
    fs::create_directories(tmp_project / "content" / "levels");
    write_text(tmp_project / "project.yaml",
               "project:\n  name: commit_conflict\n  initial_level: content/levels/level.yaml\n");
    write_text(tmp_project / "content/prefabs/prefab.yaml",
               "name: prefab\ncomponents: {}\n");
    write_text(tmp_project / "content/levels/level.yaml",
               "name: level\nentities:\n  - name: e1\n    id: e1\n    prefab: prefab\n    transform:\n"
               "      position: [0, 0, 0]\n      rotation: [0, 0, 0]\n      scale: [1, 1, 1]\n");
    write_text(tmp_project / "editor_overrides.yaml",
               "overrides:\n  e1:\n    transform:\n      position: [4, 5, 6]\n");

    const fs::path run_dir = paths.root / "build" / "ai_runs" / "test_commit_conflict";
    fs::remove_all(run_dir);
    CommitOverridesOptions stage_opts;
    stage_opts.apply.dry_run = true;
    stage_opts.apply.require_approval = false;
    stage_opts.stage_only = true;
    stage_opts.run_dir = run_dir;
    if (content_commit_overrides(argv[0], tmp_project, tmp_project / "editor_overrides.yaml", stage_opts) != 0) {
      std::cerr << "commit overrides staging failed for conflict test\n";
      ++failures;
    } else {
      write_text(tmp_project / "content/levels/level.yaml",
                 "name: level\nentities:\n  - name: e1\n    id: e1\n    prefab: prefab\n    transform:\n"
                 "      position: [9, 9, 9]\n      rotation: [0, 0, 0]\n      scale: [1, 1, 1]\n");
      CommitOverridesOptions apply_opts;
      apply_opts.apply.dry_run = true;
      apply_opts.apply.require_approval = false;
      apply_opts.stage_only = false;
      apply_opts.apply_staged_dir = run_dir;
      const int rc = content_commit_overrides(argv[0], tmp_project, tmp_project / "editor_overrides.yaml", apply_opts);
      if (rc == 0) {
        std::cerr << "commit overrides conflict was not detected\n";
        ++failures;
      } else {
        json results;
        const auto text = read_text(run_dir / "results.json");
        results = json::parse(text, nullptr, false);
        if (results.is_discarded()) {
          std::cerr << "commit overrides results missing\n";
          ++failures;
        } else if (results.value("error_code", "") != "conflict" ||
                   results.value("stage", "") != "apply" ||
                   !results.value("conflict_detected", false) ||
                   results.value("forced_apply", false)) {
          std::cerr << "commit overrides results missing conflict fields\n";
          ++failures;
        } else {
          const auto conflict_files = results.value("conflict_files", json::array());
          const auto expected = fs::relative(tmp_project / "content/levels/level.yaml", paths.root).generic_string();
          bool found = false;
          for (const auto& item : conflict_files) {
            if (item.is_string() && item.get<std::string>() == expected) {
              found = true;
              break;
            }
          }
          if (!found) {
            std::cerr << "commit overrides conflict_files missing expected path\n";
            ++failures;
          }
        }
      }

      CommitOverridesOptions force_opts;
      force_opts.apply.dry_run = true;
      force_opts.apply.require_approval = false;
      force_opts.apply.force = true;
      force_opts.stage_only = false;
      force_opts.apply_staged_dir = run_dir;
      const int force_rc = content_commit_overrides(argv[0], tmp_project, tmp_project / "editor_overrides.yaml", force_opts);
      if (force_rc != 0) {
        std::cerr << "commit overrides force apply failed\n";
        ++failures;
      } else {
        json results;
        const auto text = read_text(run_dir / "results.json");
        results = json::parse(text, nullptr, false);
        if (results.is_discarded()) {
          std::cerr << "commit overrides force results missing\n";
          ++failures;
        } else if (!results.value("forced_apply", false) ||
                   !results.value("conflict_detected", false) ||
                   !results.value("success", false)) {
          std::cerr << "commit overrides force results invalid\n";
          ++failures;
        } else {
          const std::string manifest_path = results.value("snapshot_manifest_path", "");
          if (manifest_path.empty() || !fs::exists(manifest_path)) {
            std::cerr << "commit overrides snapshot manifest missing\n";
            ++failures;
          }
          if (!results.value("snapshots_taken", false)) {
            std::cerr << "commit overrides snapshots_taken not set\n";
            ++failures;
          }
        }
      }
    }
  }

  // Test: commit overrides stage-only results + apply-staged results.
  {
    const fs::path tmp_project = paths.root / "build" / "test_tmp_commit_stage_apply";
    fs::remove_all(tmp_project);
    fs::create_directories(tmp_project / "content" / "prefabs");
    fs::create_directories(tmp_project / "content" / "levels");
    write_text(tmp_project / "project.yaml",
               "project:\n  name: commit_stage_apply\n  initial_level: content/levels/level.yaml\n");
    write_text(tmp_project / "content/prefabs/prefab.yaml",
               "name: prefab\ncomponents: {}\n");
    write_text(tmp_project / "content/levels/level.yaml",
               "name: level\nentities:\n  - name: e1\n    id: e1\n    prefab: prefab\n    transform:\n"
               "      position: [0, 0, 0]\n      rotation: [0, 0, 0]\n      scale: [1, 1, 1]\n");
    write_text(tmp_project / "editor_overrides.yaml",
               "overrides:\n  e1:\n    transform:\n      position: [7, 8, 9]\n");

    const fs::path run_dir = paths.root / "build" / "ai_runs" / "test_stage_apply";
    fs::remove_all(run_dir);
    CommitOverridesOptions opts;
    opts.apply.dry_run = true;
    opts.apply.require_approval = false;
    opts.stage_only = true;
    opts.run_dir = run_dir;
    if (content_commit_overrides(argv[0], tmp_project, tmp_project / "editor_overrides.yaml", opts) != 0) {
      std::cerr << "commit overrides stage-only failed\n";
      ++failures;
    } else {
      const auto results_text = read_text(run_dir / "results.json");
      auto results = json::parse(results_text, nullptr, false);
      if (results.is_discarded() || results.value("stage", "") != "stage" ||
          !results.value("success", false) ||
          results.value("forced_apply", true) ||
          results.value("conflict_detected", true)) {
        std::cerr << "commit overrides stage-only results invalid\n";
        ++failures;
      }
    }

    CommitOverridesOptions apply_opts;
    apply_opts.apply.dry_run = true;
    apply_opts.apply.require_approval = false;
    apply_opts.stage_only = false;
    apply_opts.apply_staged_dir = run_dir;
    if (content_commit_overrides(argv[0], tmp_project, tmp_project / "editor_overrides.yaml", apply_opts) != 0) {
      std::cerr << "commit overrides apply-staged failed\n";
      ++failures;
    } else {
      const auto results_text = read_text(run_dir / "results.json");
      auto results = json::parse(results_text, nullptr, false);
      if (results.is_discarded() || results.value("stage", "") != "apply" ||
          results.value("forced_apply", true) || results.value("conflict_detected", true)) {
        std::cerr << "commit overrides apply-staged results invalid\n";
        ++failures;
      }
    }
  }
#endif

  // Test: latest staged run selection.
  {
    const fs::path tmp_root = paths.root / "build" / "test_tmp_staged_runs";
    fs::remove_all(tmp_root);
    const fs::path run_root = tmp_root / "build" / "ai_runs";
    fs::create_directories(run_root);

    const auto make_run = [&](const std::string& name, const std::string& stage) {
      const fs::path run_dir = run_root / name;
      fs::create_directories(run_dir / "staged_patches");
      write_text(run_dir / "staged_patches" / "summary.json", "{\"run_id\":\"" + name + "\"}\n");
      write_text(run_dir / "results.json", "{\"stage\":\"" + stage + "\"}\n");
      return run_dir;
    };

    const fs::path run_old = make_run("run_old", "stage");
    const fs::path run_new = make_run("run_new", "stage");
    const fs::path run_apply = make_run("run_apply", "apply");

    const auto now = fs::file_time_type::clock::now();
    fs::last_write_time(run_old / "results.json", now - std::chrono::hours(2));
    fs::last_write_time(run_new / "results.json", now - std::chrono::hours(1));
    fs::last_write_time(run_apply / "results.json", now);

    std::string err;
    const auto latest = rkg::find_latest_staged_run_dir(tmp_root, &err);
    if (!latest.has_value() || latest->filename().string() != "run_new") {
      std::cerr << "latest staged run selection failed\n";
      ++failures;
    }
  }

  // Test: cleanup candidates preserve snapshot runs.
  {
    const fs::path tmp_root = paths.root / "build" / "test_tmp_cleanup_runs";
    fs::remove_all(tmp_root);
    const fs::path run_root = tmp_root / "build" / "ai_runs";
    fs::create_directories(run_root);

    const auto make_run = [&](const std::string& name, bool snapshot) {
      const fs::path run_dir = run_root / name;
      fs::create_directories(run_dir);
      write_text(run_dir / "results.json", "{\"stage\":\"apply\"}\n");
      if (snapshot) {
        fs::create_directories(run_dir / "snapshots");
        write_text(run_dir / "snapshots/manifest.json", "{\"files\":[]}\n");
      }
      return run_dir;
    };

    const fs::path run_new = make_run("run_new", false);
    const fs::path run_old = make_run("run_old", false);
    const fs::path run_snap = make_run("run_snap", true);

    const auto now = fs::file_time_type::clock::now();
    fs::last_write_time(run_new / "results.json", now);
    fs::last_write_time(run_old / "results.json", now - std::chrono::hours(48));
    fs::last_write_time(run_snap / "results.json", now - std::chrono::hours(72));

    rkg::RunCleanupOptions opts;
    opts.keep_last = 1;
    opts.older_than_days = 0;
    const auto cleanup = rkg::collect_run_cleanup_candidates(tmp_root, opts, {});
    const auto has_old = std::find(cleanup.candidates.begin(), cleanup.candidates.end(), run_old) !=
                         cleanup.candidates.end();
    const auto has_snap = std::find(cleanup.candidates.begin(), cleanup.candidates.end(), run_snap) !=
                          cleanup.candidates.end();
    if (!has_old || has_snap) {
      std::cerr << "cleanup candidates did not preserve snapshot runs\n";
      ++failures;
    }
  }

#if RKG_ENABLE_DATA_JSON
  // Test: snapshot restore staging produces diff + results.
  {
    const fs::path tmp_root = paths.root / "build" / "test_tmp_snapshot_restore";
    fs::remove_all(tmp_root);
    fs::create_directories(tmp_root / "content/levels");
    const fs::path target = tmp_root / "content/levels/level.yaml";
    const fs::path snapshot = tmp_root / "snapshot.before";
    write_text(target, "name: level\nvalue: 1\n");
    write_text(snapshot, "name: level\nvalue: 2\n");

    const auto stage = rkg::stage_snapshot_restore(tmp_root, target, snapshot, "snap_run");
    if (!stage.success) {
      std::cerr << "snapshot restore stage failed\n";
      ++failures;
    } else {
      const fs::path results_path = stage.run_dir / "results.json";
      const auto results_text = read_text(results_path);
      auto results = json::parse(results_text, nullptr, false);
      if (results.is_discarded() || results.value("stage", "") != "stage" ||
          !results.value("success", false)) {
        std::cerr << "snapshot stage results invalid\n";
        ++failures;
      }
      const fs::path diff_dir = stage.run_dir / "staged_patches" / "patches";
      std::string diff_text;
      for (const auto& entry : fs::directory_iterator(diff_dir)) {
        if (entry.path().extension() == ".diff") {
          diff_text = read_text(entry.path());
          break;
        }
      }
      if (diff_text.empty()) {
        std::cerr << "snapshot stage diff missing\n";
        ++failures;
      }
    }
  }
#endif

  // Test: math module basics.
  {
    const auto identity = rkg::mat4_identity();
    auto expected = rkg::mat4_identity();
    expected.m[12] = 2.0f;
    expected.m[13] = -3.0f;
    expected.m[14] = 4.0f;
    const auto translated = rkg::mat4_mul(rkg::mat4_translation({2.0f, -3.0f, 4.0f}), identity);
    bool identity_ok = true;
    for (int i = 0; i < 16; ++i) {
      if (std::abs(identity.m[i] - rkg::mat4_identity().m[i]) > 0.0001f) {
        identity_ok = false;
        break;
      }
    }
    if (!identity_ok) {
      std::cerr << "math identity failed\n";
      ++failures;
    }
    bool mul_ok = true;
    for (int i = 0; i < 16; ++i) {
      if (std::abs(translated.m[i] - expected.m[i]) > 0.0001f) {
        mul_ok = false;
        break;
      }
    }
    if (!mul_ok) {
      std::cerr << "math mul/translation failed\n";
      ++failures;
    }

    const auto proj = rkg::mat4_perspective(60.0f, 16.0f / 9.0f, 0.1f, 100.0f);
    if (!std::isfinite(proj.m[0]) || !std::isfinite(proj.m[5]) || !std::isfinite(proj.m[10]) ||
        proj.m[11] != -1.0f) {
      std::cerr << "math perspective invalid\n";
      ++failures;
    }
    if (proj.m[5] >= 0.0f) {
      std::cerr << "math perspective missing Vulkan Y flip\n";
      ++failures;
    }

    const auto view = rkg::mat4_look_at({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f});
    if (!std::isfinite(view.m[0]) || !std::isfinite(view.m[5]) || !std::isfinite(view.m[10])) {
      std::cerr << "math look_at invalid\n";
      ++failures;
    }
    if (std::abs(view.m[0] + 1.0f) > 0.0001f || std::abs(view.m[5] - 1.0f) > 0.0001f ||
        std::abs(view.m[10] + 1.0f) > 0.0001f) {
      std::cerr << "math look_at unexpected basis\n";
      ++failures;
    }
  }

#if defined(RKG_ENABLE_VULKAN) && RKG_ENABLE_VULKAN
  // Test: textured mesh shader assets exist (Phase 2B).
  {
    const fs::path shader_dir = paths.root / "plugins" / "renderer" / "vulkan" / "shaders";
    const fs::path vert_src = shader_dir / "mesh_textured.vert";
    const fs::path frag_src = shader_dir / "mesh_textured.frag";
    const fs::path vert_spv = shader_dir / "mesh_textured.vert.spv";
    const fs::path frag_spv = shader_dir / "mesh_textured.frag.spv";

    if (!fs::exists(vert_src) || !fs::exists(frag_src)) {
      std::cerr << "textured shader sources missing\n";
      ++failures;
    }
    if (!fs::exists(vert_spv) || !fs::exists(frag_spv)) {
      std::cerr << "textured shader SPIR-V missing\n";
      ++failures;
    }
  }

  // Test: skinned mesh shader assets exist (Phase 4).
  {
    const fs::path shader_dir = paths.root / "plugins" / "renderer" / "vulkan" / "shaders";
    const fs::path vert_src = shader_dir / "mesh_skinned.vert";
    const fs::path frag_src = shader_dir / "mesh_skinned.frag";
    const fs::path vert_spv = shader_dir / "mesh_skinned.vert.spv";
    const fs::path frag_spv = shader_dir / "mesh_skinned.frag.spv";

    if (!fs::exists(vert_src) || !fs::exists(frag_src)) {
      std::cerr << "skinned shader sources missing\n";
      ++failures;
    }
    if (!fs::exists(vert_spv) || !fs::exists(frag_spv)) {
      std::cerr << "skinned shader SPIR-V missing\n";
      ++failures;
    }
  }
#endif

  rkg::log::shutdown();
  return failures == 0 ? 0 : 1;
}
