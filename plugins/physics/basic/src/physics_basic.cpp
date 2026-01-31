#include "rkg/ecs.h"
#include "rkg/host_context.h"
#include "rkg/input.h"
#include "rkg/log.h"
#include "rkg/plugin_api.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {

// Optional compile-time movement debugging (off by default).
#ifndef RKG_MOVEMENT_DEBUG
#define RKG_MOVEMENT_DEBUG 0
#endif

constexpr float kEps = 1e-5f;
constexpr float kPi = 3.14159265358979323846f;

struct Vec3 {
  float x;
  float y;
  float z;
};

static Vec3 v3(float x = 0.0f, float y = 0.0f, float z = 0.0f) {
  return {x, y, z};
}

static Vec3 add(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vec3 sub(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vec3 mul(const Vec3& v, float s) {
  return {v.x * s, v.y * s, v.z * s};
}

static float dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static float length_sq(const Vec3& v) {
  return dot(v, v);
}

static float length(const Vec3& v) {
  return std::sqrt(length_sq(v));
}

static Vec3 normalize(const Vec3& v) {
  const float len = length(v);
  if (len < kEps) {
    return v3();
  }
  return mul(v, 1.0f / len);
}

static Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
  return add(a, mul(sub(b, a), t));
}

static float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

static Vec3 project_on_plane(const Vec3& v, const Vec3& n) {
  return sub(v, mul(n, dot(v, n)));
}

static Vec3 approach_vec(const Vec3& current, const Vec3& target, float max_delta) {
  Vec3 delta = sub(target, current);
  const float dlen = length(delta);
  if (dlen <= max_delta || dlen < kEps) {
    return target;
  }
  return add(current, mul(delta, max_delta / dlen));
}

static Vec3 from_array(const float v[3]) {
  return {v[0], v[1], v[2]};
}

static void to_array(const Vec3& v, float out[3]) {
  out[0] = v.x;
  out[1] = v.y;
  out[2] = v.z;
}

static Vec3 up_axis() {
  return {0.0f, 1.0f, 0.0f};
}

struct GroundHit {
  bool hit = false;
  bool walkable = false;
  Vec3 normal = v3(0.0f, 1.0f, 0.0f);
  float distance = std::numeric_limits<float>::max();
};

struct SweepHit {
  bool hit = false;
  float t = 1.0f;
  Vec3 normal = v3();
};

struct MotorInput {
  Vec3 dir_world = v3();
  float mag = 0.0f;
  bool jump_pressed = false;
  bool jump_held = false;
};

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

static MotorInput gather_input(const rkg::ecs::Transform& transform,
                               rkg::ecs::CharacterController& controller,
                               float dt) {
  MotorInput input{};
  const auto forward = get_action("MoveForward");
  const auto back = get_action("MoveBack");
  const auto left = get_action("MoveLeft");
  const auto right = get_action("MoveRight");
  const auto jump = get_action("Jump");
  const auto sprint = get_action("Sprint");

  input.jump_pressed = jump.pressed;
  input.jump_held = jump.held;

  Vec3 raw_dir = v3();
  float raw_mag = 0.0f;

  if (controller.use_desired_input) {
    Vec3 desired = from_array(controller.desired_move_dir);
    float desired_speed = controller.desired_move_speed;
    if (desired_speed > kEps) {
      raw_mag = clampf(desired_speed / std::max(controller.max_speed, kEps), 0.0f, 1.0f);
      raw_dir = normalize(desired);
    } else {
      const float len = length(desired);
      if (len > kEps) {
        raw_mag = clampf(len, 0.0f, 1.0f);
        raw_dir = mul(desired, 1.0f / len);
      }
    }
  } else {
    float dir_x = 0.0f;
    float dir_z = 0.0f;
    if (forward.held) dir_z += 1.0f;
    if (back.held) dir_z -= 1.0f;
    if (left.held) dir_x += 1.0f;
    if (right.held) dir_x -= 1.0f;

    float len = std::sqrt(dir_x * dir_x + dir_z * dir_z);
    if (len > kEps) {
      dir_x /= len;
      dir_z /= len;
      const float yaw = transform.rotation[1];
      const float cos_y = std::cos(yaw);
      const float sin_y = std::sin(yaw);
      const float world_x = dir_x * cos_y + dir_z * sin_y;
      const float world_z = -dir_x * sin_y + dir_z * cos_y;
      raw_dir = v3(world_x, 0.0f, world_z);
      raw_mag = 1.0f;
    }
    if (sprint.held) {
      raw_mag = std::min(1.0f, raw_mag * 1.0f);
    }
  }

  const float tau = std::max(controller.input_smooth_tau, 0.0f);
  if (tau > kEps) {
    const float alpha = 1.0f - std::exp(-dt / tau);
    Vec3 prev = from_array(controller.smoothed_input);
    Vec3 target = mul(raw_dir, raw_mag);
    Vec3 smoothed = lerp(prev, target, clampf(alpha, 0.0f, 1.0f));
    to_array(smoothed, controller.smoothed_input);
    const float sm_len = length(smoothed);
    input.mag = clampf(sm_len, 0.0f, 1.0f);
    input.dir_world = (sm_len > kEps) ? mul(smoothed, 1.0f / sm_len) : v3();
  } else {
    input.dir_world = raw_dir;
    input.mag = clampf(raw_mag, 0.0f, 1.0f);
  }

  return input;
}

static GroundHit find_ground(const rkg::ecs::Registry& registry,
                             rkg::ecs::Entity self,
                             const rkg::ecs::Transform& transform,
                             const rkg::ecs::CharacterController& controller) {
  GroundHit result{};
  const Vec3 up = up_axis();
  const Vec3 pos = from_array(transform.position);
  const Vec3 center = add(pos, mul(up, controller.center_offset));
  const Vec3 bottom = sub(center, mul(up, controller.half_height + controller.radius));
  const float slope_limit_cos = std::cos(controller.slope_limit_deg * (kPi / 180.0f));

  for (const auto& kv : registry.colliders()) {
    if (kv.first == self) {
      continue;
    }
    const auto& collider = kv.second;
    if (collider.type != rkg::ecs::ColliderType::Plane) {
      continue;
    }
    Vec3 normal = v3(collider.normal[0], collider.normal[1], collider.normal[2]);
    const float nlen = length(normal);
    if (nlen < kEps) {
      continue;
    }
    normal = mul(normal, 1.0f / nlen);

    float plane_d = collider.distance;
    if (const auto* plane_transform = registry.get_transform(kv.first)) {
      const Vec3 plane_pos = from_array(plane_transform->position);
      plane_d += dot(normal, plane_pos);
    }

    const float dist = dot(normal, bottom) - plane_d;
    if (dist < -controller.step_height) {
      continue;
    }
    const float cos_up = dot(normal, up);
    const bool walkable = cos_up >= slope_limit_cos;
    if (dist < result.distance) {
      result.hit = true;
      result.normal = normal;
      result.distance = dist;
      result.walkable = walkable;
    }
  }

  return result;
}

static bool sweep_sphere_aabb(const Vec3& center,
                              float radius,
                              const Vec3& delta,
                              const Vec3& box_center,
                              const Vec3& half_extents,
                              float& out_t,
                              Vec3& out_normal) {
  Vec3 min_b = sub(box_center, add(half_extents, v3(radius, radius, radius)));
  Vec3 max_b = add(box_center, add(half_extents, v3(radius, radius, radius)));

  float tmin = 0.0f;
  float tmax = 1.0f;
  Vec3 hit_normal = v3();

  const float dir[3] = {delta.x, delta.y, delta.z};
  const float orig[3] = {center.x, center.y, center.z};
  const float bmin[3] = {min_b.x, min_b.y, min_b.z};
  const float bmax[3] = {max_b.x, max_b.y, max_b.z};

  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(dir[axis]) < kEps) {
      if (orig[axis] < bmin[axis] || orig[axis] > bmax[axis]) {
        return false;
      }
      continue;
    }
    const float ood = 1.0f / dir[axis];
    float t1 = (bmin[axis] - orig[axis]) * ood;
    float t2 = (bmax[axis] - orig[axis]) * ood;
    float n_sign = 0.0f;
    if (t1 > t2) {
      std::swap(t1, t2);
      n_sign = 1.0f;
    } else {
      n_sign = -1.0f;
    }
    if (t1 > tmin) {
      tmin = t1;
      hit_normal = v3();
      if (axis == 0) hit_normal.x = n_sign;
      if (axis == 1) hit_normal.y = n_sign;
      if (axis == 2) hit_normal.z = n_sign;
    }
    if (t2 < tmax) {
      tmax = t2;
    }
    if (tmin > tmax) {
      return false;
    }
  }

  if (tmin < 0.0f || tmin > 1.0f) {
    return false;
  }
  out_t = tmin;
  out_normal = hit_normal;
  return true;
}

static bool sweep_capsule_world(const rkg::ecs::Registry& registry,
                                rkg::ecs::Entity self,
                                const Vec3& center,
                                float half_height,
                                float radius,
                                const Vec3& delta,
                                SweepHit& out_hit) {
  bool hit_any = false;
  float best_t = out_hit.t;
  Vec3 best_normal = out_hit.normal;
  const Vec3 up = up_axis();

  const Vec3 bottom = sub(center, mul(up, half_height));
  const Vec3 top = add(center, mul(up, half_height));

  for (const auto& kv : registry.colliders()) {
    if (kv.first == self) {
      continue;
    }
    const auto& collider = kv.second;
    if (collider.type != rkg::ecs::ColliderType::AABB) {
      continue;
    }
    Vec3 box_center = v3(collider.center[0], collider.center[1], collider.center[2]);
    if (const auto* t = registry.get_transform(kv.first)) {
      box_center = add(box_center, from_array(t->position));
    }
    Vec3 half_extents = v3(collider.half_extents[0], collider.half_extents[1], collider.half_extents[2]);

    float t_hit = 1.0f;
    Vec3 normal = v3();
    if (sweep_sphere_aabb(bottom, radius, delta, box_center, half_extents, t_hit, normal)) {
      if (t_hit < best_t) {
        best_t = t_hit;
        best_normal = normal;
        hit_any = true;
      }
    }
    if (sweep_sphere_aabb(top, radius, delta, box_center, half_extents, t_hit, normal)) {
      if (t_hit < best_t) {
        best_t = t_hit;
        best_normal = normal;
        hit_any = true;
      }
    }
  }

  if (hit_any) {
    out_hit.hit = true;
    out_hit.t = best_t;
    out_hit.normal = best_normal;
  }
  return hit_any;
}

static bool try_step_up(const rkg::ecs::Registry& registry,
                        rkg::ecs::Entity self,
                        rkg::ecs::CharacterController& controller,
                        Vec3& center,
                        const Vec3& move,
                        const Vec3& ground_normal) {
  if (controller.step_height <= 0.0f) {
    return false;
  }
  const Vec3 up = up_axis();
  const float slope_limit_cos = std::cos(controller.slope_limit_deg * (kPi / 180.0f));
  if (dot(ground_normal, up) < slope_limit_cos) {
    return false;
  }

  const Vec3 horiz = sub(move, mul(up, dot(move, up)));
  if (length(horiz) < kEps) {
    return false;
  }

  SweepHit hit{};
  Vec3 center_up = add(center, mul(up, controller.step_height + controller.skin_width));
  if (sweep_capsule_world(registry, self, center_up, controller.half_height, controller.radius,
                          v3(), hit)) {
    return false;
  }

  SweepHit fwd_hit{};
  if (sweep_capsule_world(registry, self, center_up, controller.half_height, controller.radius,
                          horiz, fwd_hit)) {
    return false;
  }

  Vec3 stepped = add(center_up, horiz);
  rkg::ecs::Transform temp{};
  to_array(sub(stepped, mul(up, controller.center_offset)), temp.position);
  GroundHit ground = find_ground(registry, self, temp, controller);
  if (!ground.hit || !ground.walkable) {
    return false;
  }

  if (ground.distance > controller.step_height + controller.skin_width) {
    return false;
  }

  const float snap = std::max(0.0f, ground.distance - controller.skin_width);
  stepped = sub(stepped, mul(ground.normal, snap));
  center = stepped;
  return true;
}

static Vec3 update_ground_velocity(const rkg::ecs::CharacterController& controller,
                                   const Vec3& velocity,
                                   const Vec3& desired_dir,
                                   float desired_speed,
                                   const Vec3& ground_normal,
                                   float dt) {
  const Vec3 up = up_axis();
  const Vec3 v_vert = mul(up, dot(velocity, up));
  Vec3 v_lat = sub(velocity, v_vert);
  Vec3 v_ground = sub(v_lat, mul(ground_normal, dot(v_lat, ground_normal)));

  const float speed = length(v_ground);
  const bool has_input = desired_speed > 0.01f;
  const Vec3 desired_vel = mul(desired_dir, desired_speed);

  if (has_input) {
    Vec3 delta = sub(desired_vel, v_ground);
    const float delta_len = length(delta);
    if (delta_len > kEps) {
      const float max_delta = controller.accel * dt;
      if (delta_len > max_delta) {
        v_ground = add(v_ground, mul(delta, max_delta / delta_len));
      } else {
        v_ground = desired_vel;
      }
    }
  }

  if (has_input && speed > controller.turn_speed_threshold) {
    const Vec3 v_dir = (speed > kEps) ? mul(v_ground, 1.0f / speed) : v3();
    const float desired_len = length(desired_vel);
    if (desired_len > kEps) {
      const Vec3 d_dir = mul(desired_vel, 1.0f / desired_len);
      const float angle = std::acos(clampf(dot(v_dir, d_dir), -1.0f, 1.0f));
      const float boost = controller.turn_boost_factor * (angle / kPi);
      const float extra = controller.accel * boost * dt;
      v_ground = add(v_ground, mul(d_dir, extra));
    }
  }

  const bool braking = !has_input || (dot(v_ground, desired_vel) < 0.0f) ||
                       (speed > desired_speed + 0.05f);
  if (braking) {
    const float friction = controller.friction * controller.braking_friction_factor;
    const float decel = controller.braking_deceleration;
    v_ground = sub(v_ground, mul(v_ground, friction * dt));
    const float v_len = length(v_ground);
    if (v_len > kEps) {
      Vec3 v_dir = mul(v_ground, 1.0f / v_len);
      v_ground = sub(v_ground, mul(v_dir, decel * dt));
      if (dot(v_ground, v_dir) < 0.0f) {
        v_ground = v3();
      }
    }
  }

  const float max_speed = controller.max_speed;
  const float v_len = length(v_ground);
  if (v_len > max_speed) {
    v_ground = mul(v_ground, max_speed / v_len);
  }

  return add(v_ground, v3(0.0f, v_vert.y, 0.0f));
}

static Vec3 update_air_velocity(const rkg::ecs::CharacterController& controller,
                                const Vec3& velocity,
                                const Vec3& desired_dir,
                                float desired_speed,
                                float dt) {
  const Vec3 up = up_axis();
  const float v_up = dot(velocity, up);
  Vec3 v_vert = mul(up, v_up);
  Vec3 v_horiz = sub(velocity, v_vert);

  float max_air_speed = controller.max_air_speed > 0.0f ? controller.max_air_speed : controller.max_speed;
  float max_air_accel = controller.max_air_accel > 0.0f ? controller.max_air_accel : controller.accel;
  float air_control = clampf(controller.air_control, 0.0f, 1.0f);

  const float horiz_speed = length(v_horiz);
  if (horiz_speed < controller.air_control_boost_velocity_threshold) {
    air_control = std::min(1.0f, air_control * controller.air_control_boost_multiplier);
  }

  const Vec3 desired_air_vel = mul(desired_dir, std::min(desired_speed, max_air_speed));
  v_horiz = approach_vec(v_horiz, desired_air_vel, max_air_accel * air_control * dt);

  v_vert = add(v_vert, mul(up, controller.gravity * dt));
  if (controller.gravity < 0.0f) {
    v_vert.y = std::max(v_vert.y, -controller.terminal_velocity);
  } else {
    v_vert.y = std::min(v_vert.y, controller.terminal_velocity);
  }

  return add(v_horiz, v_vert);
}

static void move_with_collisions(rkg::ecs::Registry& registry,
                                 rkg::ecs::Entity entity,
                                 rkg::ecs::CharacterController& controller,
                                 rkg::ecs::Transform& transform,
                                 Vec3& velocity,
                                 const Vec3& ground_normal,
                                 float dt) {
  const Vec3 up = up_axis();
  Vec3 position = from_array(transform.position);
  Vec3 center = add(position, mul(up, controller.center_offset));
  Vec3 remaining = mul(velocity, dt);
  Vec3 last_hit_normal = v3();

  for (int i = 0; i < controller.max_sweep_iterations; ++i) {
    if (length(remaining) < kEps) {
      break;
    }
    SweepHit hit{};
    if (!sweep_capsule_world(registry, entity, center, controller.half_height, controller.radius,
                             remaining, hit)) {
      center = add(center, remaining);
      break;
    }

    if (controller.mode == rkg::ecs::MovementMode::Grounded) {
      const float slope_limit_cos = std::cos(controller.slope_limit_deg * (kPi / 180.0f));
      if (dot(hit.normal, up) < slope_limit_cos) {
        if (try_step_up(registry, entity, controller, center, remaining, ground_normal)) {
          remaining = v3();
          continue;
        }
      }
    }

    center = add(center, mul(remaining, hit.t));
    center = add(center, mul(hit.normal, controller.skin_width));
    last_hit_normal = hit.normal;

    const float into = dot(remaining, hit.normal);
    remaining = sub(remaining, mul(hit.normal, into));
    const float vel_into = dot(velocity, hit.normal);
    if (vel_into < 0.0f) {
      velocity = sub(velocity, mul(hit.normal, vel_into));
    }
  }

#if RKG_MOVEMENT_DEBUG
  rkg::log::info("movement: last_hit_normal=(" +
                 std::to_string(last_hit_normal.x) + "," +
                 std::to_string(last_hit_normal.y) + "," +
                 std::to_string(last_hit_normal.z) + ")");
#endif

  position = sub(center, mul(up, controller.center_offset));
  to_array(position, transform.position);
}

static void update_character(rkg::ecs::Registry& registry,
                             rkg::ecs::Entity entity,
                             rkg::ecs::CharacterController& controller,
                             float dt) {
  auto* transform = registry.get_transform(entity);
  if (!transform) {
    return;
  }
  auto* velocity_comp = registry.get_velocity(entity);
  if (!velocity_comp) {
    rkg::ecs::Velocity initial{};
    registry.set_velocity(entity, initial);
    velocity_comp = registry.get_velocity(entity);
    if (!velocity_comp) {
      return;
    }
  }

  MotorInput input = gather_input(*transform, controller, dt);

  // Jump buffer & coyote time.
  if (input.jump_pressed) {
    controller.jump_buffer = controller.jump_buffer_time;
  } else {
    controller.jump_buffer = std::max(0.0f, controller.jump_buffer - dt);
  }

  GroundHit ground = find_ground(registry, entity, *transform, controller);
  const bool on_ground = ground.hit && ground.walkable &&
                         (ground.distance <= controller.skin_width + controller.ground_snap_max);

  if (on_ground) {
    controller.time_since_grounded = 0.0f;
    controller.grounded = true;
    controller.mode = rkg::ecs::MovementMode::Grounded;
  } else if (ground.hit && !ground.walkable) {
    controller.time_since_grounded += dt;
    controller.grounded = false;
    controller.mode = rkg::ecs::MovementMode::Sliding;
  } else {
    controller.time_since_grounded += dt;
    controller.grounded = false;
    controller.mode = rkg::ecs::MovementMode::Falling;
  }

  bool do_jump = false;
  if (controller.jump_buffer > 0.0f) {
    if (controller.grounded || controller.time_since_grounded <= controller.coyote_time) {
      do_jump = true;
      controller.jump_buffer = 0.0f;
    }
  }
  controller.jump_held = input.jump_held;

  Vec3 velocity = from_array(velocity_comp->linear);
  Vec3 desired_dir = input.dir_world;
  float desired_speed = input.mag * controller.max_speed;

  if (controller.mode == rkg::ecs::MovementMode::Grounded) {
    velocity = update_ground_velocity(controller, velocity, desired_dir, desired_speed,
                                      ground.normal, dt);
    if (!do_jump) {
      // Keep vertical velocity zeroed when grounded (snap handles contact).
      velocity.y = 0.0f;
    }
  } else if (controller.mode == rkg::ecs::MovementMode::Sliding) {
    velocity = update_air_velocity(controller, velocity, desired_dir, desired_speed, dt);
    const Vec3 up = up_axis();
    Vec3 gravity_vec = mul(up, controller.gravity);
    Vec3 g_along = sub(gravity_vec, mul(ground.normal, dot(gravity_vec, ground.normal)));
    velocity = add(velocity, mul(g_along, dt));
    const float into = dot(velocity, ground.normal);
    if (into < 0.0f) {
      velocity = sub(velocity, mul(ground.normal, into));
    }
  } else {
    velocity = update_air_velocity(controller, velocity, desired_dir, desired_speed, dt);
  }

  if (do_jump) {
    velocity.y = controller.jump_impulse;
    controller.grounded = false;
    controller.mode = rkg::ecs::MovementMode::Falling;
  }

  // Apply base velocity (moving platforms), if any.
  if (controller.grounded) {
    Vec3 base_vel = from_array(controller.external_base_velocity);
    velocity = add(velocity, base_vel);
  }

  move_with_collisions(registry, entity, controller, *transform, velocity, ground.normal, dt);

  // Post-move ground snap.
  GroundHit after_ground = find_ground(registry, entity, *transform, controller);
  if (after_ground.hit && after_ground.walkable) {
    if (after_ground.distance <= controller.ground_snap_max + controller.skin_width) {
      const float snap = std::max(0.0f, after_ground.distance - controller.skin_width);
      Vec3 pos = from_array(transform->position);
      pos = sub(pos, mul(after_ground.normal, snap));
      to_array(pos, transform->position);
      controller.grounded = true;
      controller.mode = rkg::ecs::MovementMode::Grounded;
      controller.ground_height = transform->position[1];
    }
  }

  to_array(velocity, velocity_comp->linear);
  controller.vertical_velocity = velocity.y;

#if RKG_MOVEMENT_DEBUG
  static int debug_frame = 0;
  if ((debug_frame++ % 60) == 0) {
    rkg::log::info("movement: pos=(" +
                   std::to_string(transform->position[0]) + "," +
                   std::to_string(transform->position[1]) + "," +
                   std::to_string(transform->position[2]) + ") vel=(" +
                   std::to_string(velocity.x) + "," +
                   std::to_string(velocity.y) + "," +
                   std::to_string(velocity.z) + ") grounded=" +
                   std::to_string(controller.grounded) +
                   " ground_dist=" + std::to_string(after_ground.distance));
  }
#endif
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
    auto& controller = kv.second;
    float remaining = dt_seconds;
    const float max_step = (controller.max_substep_dt > 0.0f) ? controller.max_substep_dt : dt_seconds;
    int steps = 0;
    while (remaining > 0.0f && steps < std::max(1, controller.max_substeps)) {
      const float step = std::min(remaining, max_step);
      update_character(registry, kv.first, controller, step);
      remaining -= step;
      ++steps;
    }
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
