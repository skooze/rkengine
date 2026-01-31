#include "rkg/ecs.h"
#include "rkg/host_context.h"
#include "rkg/input.h"
#include "rkg/log.h"
#include "rkg/plugin_api.h"

#include <algorithm>
#include <cmath>

namespace {
rkg::HostContext* g_ctx = nullptr;
rkg::ecs::Entity g_ground = rkg::ecs::kInvalidEntity;

rkg::input::ActionState get_action(const char* action) {
  if (!g_ctx || !g_ctx->get_action_state) {
    return {};
  }
  return g_ctx->get_action_state(g_ctx->action_state_user, action);
}

void ensure_ground_plane() {
  if (!g_ctx || !g_ctx->registry) {
    return;
  }
  auto& registry = *g_ctx->registry;
  if (g_ground != rkg::ecs::kInvalidEntity && registry.get_collider(g_ground)) {
    return;
  }
  g_ground = registry.create_entity();
  rkg::ecs::Transform transform{};
  registry.set_transform(g_ground, transform);
  rkg::ecs::Collider collider{};
  collider.type = rkg::ecs::ColliderType::Plane;
  collider.normal[0] = 0.0f;
  collider.normal[1] = 1.0f;
  collider.normal[2] = 0.0f;
  collider.distance = 0.0f;
  registry.set_collider(g_ground, collider);
}

void update_character(rkg::ecs::Registry& registry,
                      rkg::ecs::Entity entity,
                      rkg::ecs::CharacterController& controller,
                      float dt) {
  auto* transform = registry.get_transform(entity);
  if (!transform) {
    return;
  }
  auto* velocity = registry.get_velocity(entity);
  if (!velocity) {
    rkg::ecs::Velocity initial{};
    registry.set_velocity(entity, initial);
    velocity = registry.get_velocity(entity);
    if (!velocity) {
      return;
    }
  }

  const auto forward = get_action("MoveForward");
  const auto back = get_action("MoveBack");
  const auto left = get_action("MoveLeft");
  const auto right = get_action("MoveRight");
  const auto jump = get_action("Jump");
  const auto sprint = get_action("Sprint");

  float dir_x = 0.0f;
  float dir_z = 0.0f;
  if (forward.held) dir_z += 1.0f;
  if (back.held) dir_z -= 1.0f;
  if (left.held) dir_x += 1.0f;
  if (right.held) dir_x -= 1.0f;

  const float length = std::sqrt(dir_x * dir_x + dir_z * dir_z);
  const float sprint_scale = sprint.held ? 1.75f : 1.0f;
  const float max_speed = controller.max_speed * sprint_scale;

  if (length > 0.0001f) {
    dir_x /= length;
    dir_z /= length;
    const float yaw = transform->rotation[1];
    const float cos_y = std::cos(yaw);
    const float sin_y = std::sin(yaw);
    const float world_x = dir_x * cos_y + dir_z * sin_y;
    const float world_z = -dir_x * sin_y + dir_z * cos_y;
    const float target_x = world_x * max_speed;
    const float target_z = world_z * max_speed;
    const float blend = std::min(1.0f, controller.accel * dt);
    velocity->linear[0] += (target_x - velocity->linear[0]) * blend;
    velocity->linear[2] += (target_z - velocity->linear[2]) * blend;
  } else {
    const float drop = std::max(0.0f, 1.0f - controller.friction * dt);
    velocity->linear[0] *= drop;
    velocity->linear[2] *= drop;
  }

  if (controller.grounded && (jump.pressed || (jump.held && !controller.jump_held))) {
    velocity->linear[1] = controller.jump_impulse;
    controller.grounded = false;
  }
  controller.jump_held = jump.held;

  velocity->linear[1] += controller.gravity * dt;

  transform->position[0] += velocity->linear[0] * dt;
  transform->position[1] += velocity->linear[1] * dt;
  transform->position[2] += velocity->linear[2] * dt;

  const float capsule_center_y = transform->position[1] + controller.center_offset;
  const float capsule_bottom = capsule_center_y - (controller.half_height + controller.radius);
  if (capsule_bottom < 0.0f) {
    transform->position[1] -= capsule_bottom;
    if (velocity->linear[1] < 0.0f) {
      velocity->linear[1] = 0.0f;
    }
    controller.grounded = true;
    controller.ground_height = 0.0f;
  } else {
    controller.grounded = false;
  }

  controller.vertical_velocity = velocity->linear[1];
}

bool physics_basic_init(void* host_context) {
  g_ctx = static_cast<rkg::HostContext*>(host_context);
  if (!g_ctx || !g_ctx->registry) {
    rkg::log::error("physics_basic init failed: missing host context or registry");
    return false;
  }
  ensure_ground_plane();
  rkg::log::info("physics_basic: init");
  return true;
}

void physics_basic_shutdown() {
  g_ctx = nullptr;
  g_ground = rkg::ecs::kInvalidEntity;
  rkg::log::info("physics_basic: shutdown");
}

void physics_basic_update(float dt_seconds) {
  if (!g_ctx || !g_ctx->registry || dt_seconds <= 0.0f) {
    return;
  }
  ensure_ground_plane();
  auto& registry = *g_ctx->registry;
  for (auto& kv : registry.character_controllers()) {
    update_character(registry, kv.first, kv.second, dt_seconds);
  }
}
} // namespace

extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_physics_basic(uint32_t host_api_version) {
  static rkg::RkgPluginApi api{
      rkg::kRkgPluginApiVersion,
      "physics_basic",
      rkg::PluginType::Physics,
      0,
      physics_basic_init,
      physics_basic_shutdown,
      physics_basic_update,
      nullptr};
  if (host_api_version != rkg::kRkgPluginApiVersion) {
    return nullptr;
  }
  return &api;
}
