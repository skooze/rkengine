#include "rkg/locomotion.h"

#include "rkg/log.h"
#include "rkg/math.h"
#include "rkg/movement_log.h"
#include "rkg/renderer_hooks.h"

#include <algorithm>
#include <cctype>
#include <functional>
#include <cmath>
#include <functional>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace rkg::locomotion {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kEps = 1e-5f;

struct Vec3 {
  float x;
  float y;
  float z;
};

struct CapsuleSeg {
  Vec3 a;
  Vec3 b;
  float r;
  ecs::Entity owner;
};

static std::vector<CapsuleSeg> g_external_caps;

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

static Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y,
          a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

static float length_sq(const Vec3& v) {
  return dot(v, v);
}

static float length(const Vec3& v) {
  return std::sqrt(length_sq(v));
}

static Vec3 normalize(const Vec3& v) {
  const float len = length(v);
  if (len < kEps) return v3();
  return mul(v, 1.0f / len);
}

static Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
  return add(a, mul(sub(b, a), t));
}

static float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

static float saturate(float v) {
  return clampf(v, 0.0f, 1.0f);
}

static float smoothstep01(float v) {
  v = saturate(v);
  return v * v * (3.0f - 2.0f * v);
}

static float unwrap_angle(float angle, float prev) {
  const float two_pi = 2.0f * kPi;
  float delta = angle - prev;
  if (delta > kPi) {
    angle -= two_pi;
  } else if (delta < -kPi) {
    angle += two_pi;
  }
  return angle;
}

static float frac(float v) {
  return v - std::floor(v);
}

static float wrap_pi(float angle) {
  while (angle > kPi) angle -= 2.0f * kPi;
  while (angle < -kPi) angle += 2.0f * kPi;
  return angle;
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

static std::string to_lower(std::string s) {
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static bool name_has(const std::string& name, const std::vector<std::string>& needles) {
  for (const auto& needle : needles) {
    if (name.find(needle) != std::string::npos) return true;
  }
  return false;
}

static uint32_t find_bone_index_by_name(const ecs::Skeleton& skel,
                                        const std::vector<std::string>& include,
                                        const std::vector<std::string>& exclude) {
  for (uint32_t i = 0; i < skel.bones.size(); ++i) {
    const std::string n = to_lower(skel.bones[i].name);
    if (n.empty()) continue;
    if (!include.empty() && !name_has(n, include)) continue;
    if (!exclude.empty() && name_has(n, exclude)) continue;
    return i;
  }
  return UINT32_MAX;
}

static uint32_t find_side_bone(const ecs::Skeleton& skel,
                               const std::vector<std::string>& side,
                               const std::vector<std::string>& part,
                               const std::vector<std::string>& exclude = {}) {
  for (uint32_t i = 0; i < skel.bones.size(); ++i) {
    const std::string n = to_lower(skel.bones[i].name);
    if (n.empty()) continue;
    if (!name_has(n, side)) continue;
    if (!name_has(n, part)) continue;
    if (!exclude.empty() && name_has(n, exclude)) continue;
    return i;
  }
  return UINT32_MAX;
}

static rkg::Mat4 mat4_extract_rotation(const rkg::Mat4& m) {
  rkg::Vec3 c0{m.m[0], m.m[1], m.m[2]};
  rkg::Vec3 c1{m.m[4], m.m[5], m.m[6]};
  rkg::Vec3 c2{m.m[8], m.m[9], m.m[10]};
  c0 = rkg::vec3_normalize(c0);
  c1 = rkg::vec3_normalize(c1);
  c2 = rkg::vec3_normalize(c2);
  rkg::Mat4 out = rkg::mat4_identity();
  out.m[0] = c0.x; out.m[1] = c0.y; out.m[2] = c0.z;
  out.m[4] = c1.x; out.m[5] = c1.y; out.m[6] = c1.z;
  out.m[8] = c2.x; out.m[9] = c2.y; out.m[10] = c2.z;
  return out;
}

static rkg::Mat4 mat4_transpose3x3(const rkg::Mat4& m) {
  rkg::Mat4 out = rkg::mat4_identity();
  out.m[0] = m.m[0];  out.m[1] = m.m[4];  out.m[2] = m.m[8];
  out.m[4] = m.m[1];  out.m[5] = m.m[5];  out.m[6] = m.m[9];
  out.m[8] = m.m[2];  out.m[9] = m.m[6];  out.m[10] = m.m[10];
  return out;
}

static rkg::Mat4 mat4_rotation_axis_angle(const rkg::Vec3& axis, float angle) {
  rkg::Mat4 out = rkg::mat4_identity();
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  const float t = 1.0f - c;
  const float x = axis.x;
  const float y = axis.y;
  const float z = axis.z;
  out.m[0] = t * x * x + c;
  out.m[1] = t * x * y + s * z;
  out.m[2] = t * x * z - s * y;
  out.m[4] = t * x * y - s * z;
  out.m[5] = t * y * y + c;
  out.m[6] = t * y * z + s * x;
  out.m[8] = t * x * z + s * y;
  out.m[9] = t * y * z - s * x;
  out.m[10] = t * z * z + c;
  return out;
}

static rkg::Mat4 mat4_rotation_from_to(const rkg::Vec3& from, const rkg::Vec3& to) {
  const rkg::Vec3 f = rkg::vec3_normalize(from);
  const rkg::Vec3 t = rkg::vec3_normalize(to);
  const float dot_ft = clampf(rkg::vec3_dot(f, t), -1.0f, 1.0f);
  if (dot_ft > 0.9999f) {
    return rkg::mat4_identity();
  }
  if (dot_ft < -0.9999f) {
    rkg::Vec3 axis = rkg::vec3_cross(f, {0.0f, 1.0f, 0.0f});
    const float axis_len = std::sqrt(rkg::vec3_dot(axis, axis));
    if (axis_len < 0.0001f) {
      axis = rkg::vec3_cross(f, {1.0f, 0.0f, 0.0f});
    }
    axis = rkg::vec3_normalize(axis);
    return mat4_rotation_axis_angle(axis, kPi);
  }
  rkg::Vec3 axis = rkg::vec3_cross(f, t);
  axis = rkg::vec3_normalize(axis);
  const float angle = std::acos(dot_ft);
  return mat4_rotation_axis_angle(axis, angle);
}

static rkg::Vec3 mat4_to_euler_xyz(const rkg::Mat4& m) {
  const float r00 = m.m[0];
  const float r10 = m.m[1];
  const float r11 = m.m[5];
  const float r12 = m.m[9];
  const float r20 = m.m[2];
  const float r21 = m.m[6];
  const float r22 = m.m[10];

  const float sy = clampf(-r20, -1.0f, 1.0f);
  const float y = std::asin(sy);
  const float cy = std::cos(y);
  float x = 0.0f;
  float z = 0.0f;
  if (std::fabs(cy) > 0.0001f) {
    x = std::atan2(r21, r22);
    z = std::atan2(r10, r00);
  } else {
    x = std::atan2(-r12, r11);
    z = 0.0f;
  }
  return {x, y, z};
}

static Vec3 mat4_mul_vec3(const rkg::Mat4& m, const Vec3& v) {
  return {
      m.m[0] * v.x + m.m[4] * v.y + m.m[8] * v.z,
      m.m[1] * v.x + m.m[5] * v.y + m.m[9] * v.z,
      m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z};
}

static Vec3 rotate_xyz_vec(const Vec3& v, const Vec3& rot) {
  const rkg::Vec3 r{rot.x, rot.y, rot.z};
  const rkg::Mat4 m = rkg::mat4_rotation_xyz(r);
  return mat4_mul_vec3(m, v);
}

static Vec3 to_world_point(const ecs::Transform& root, const Vec3& local) {
  Vec3 scaled = local;
  const float sx = (root.scale[0] != 0.0f) ? root.scale[0] : 1.0f;
  const float sy = (root.scale[1] != 0.0f) ? root.scale[1] : 1.0f;
  const float sz = (root.scale[2] != 0.0f) ? root.scale[2] : 1.0f;
  scaled.x *= sx;
  scaled.y *= sy;
  scaled.z *= sz;
  const Vec3 rotated = rotate_xyz_vec(scaled, {root.rotation[0], root.rotation[1], root.rotation[2]});
  return add(rotated, {root.position[0], root.position[1], root.position[2]});
}

static Vec3 to_local_point(const ecs::Transform& root, const Vec3& world) {
  Vec3 v = sub(world, {root.position[0], root.position[1], root.position[2]});
  const Vec3 inv_rot{-root.rotation[0], -root.rotation[1], -root.rotation[2]};
  v = rotate_xyz_vec(v, inv_rot);
  const float sx = (root.scale[0] != 0.0f) ? root.scale[0] : 1.0f;
  const float sy = (root.scale[1] != 0.0f) ? root.scale[1] : 1.0f;
  const float sz = (root.scale[2] != 0.0f) ? root.scale[2] : 1.0f;
  v.x /= sx;
  v.y /= sy;
  v.z /= sz;
  return v;
}

static Vec3 to_world_dir(const ecs::Transform& root, const Vec3& local_dir) {
  return rotate_xyz_vec(local_dir, {root.rotation[0], root.rotation[1], root.rotation[2]});
}

static Vec3 to_local_dir(const ecs::Transform& root, const Vec3& world_dir) {
  const Vec3 inv_rot{-root.rotation[0], -root.rotation[1], -root.rotation[2]};
  return rotate_xyz_vec(world_dir, inv_rot);
}

static Vec3 to_local_offset(const ecs::Transform& root, const Vec3& world_offset) {
  const Vec3 inv_rot{-root.rotation[0], -root.rotation[1], -root.rotation[2]};
  Vec3 v = rotate_xyz_vec(world_offset, inv_rot);
  const float sx = (root.scale[0] != 0.0f) ? root.scale[0] : 1.0f;
  const float sy = (root.scale[1] != 0.0f) ? root.scale[1] : 1.0f;
  const float sz = (root.scale[2] != 0.0f) ? root.scale[2] : 1.0f;
  v.x /= sx;
  v.y /= sy;
  v.z /= sz;
  return v;
}

static void compute_world_matrices(const ecs::Skeleton& skel,
                                   const std::vector<ecs::Transform>& locals,
                                   std::vector<rkg::Mat4>& world) {
  const size_t joint_count = skel.bones.size();
  world.assign(joint_count, rkg::mat4_identity());
  std::vector<uint8_t> visited(joint_count, 0);

  auto build_local = [&](size_t idx) {
    const ecs::Transform& t = locals[idx];
    const rkg::Vec3 pos{t.position[0], t.position[1], t.position[2]};
    const rkg::Vec3 rot{t.rotation[0], t.rotation[1], t.rotation[2]};
    const rkg::Vec3 scale{t.scale[0], t.scale[1], t.scale[2]};
    return rkg::mat4_mul(rkg::mat4_translation(pos),
                         rkg::mat4_mul(rkg::mat4_rotation_xyz(rot), rkg::mat4_scale(scale)));
  };

  std::function<rkg::Mat4(size_t)> eval = [&](size_t idx) -> rkg::Mat4 {
    if (idx >= joint_count) return rkg::mat4_identity();
    if (visited[idx]) return world[idx];
    visited[idx] = 1;
    rkg::Mat4 local = build_local(idx);
    const int parent = skel.bones[idx].parent_index;
    if (parent >= 0 && static_cast<size_t>(parent) < joint_count) {
      rkg::Mat4 parent_world = eval(static_cast<size_t>(parent));
      world[idx] = rkg::mat4_mul(parent_world, local);
    } else {
      world[idx] = local;
    }
    return world[idx];
  };

  for (size_t i = 0; i < joint_count; ++i) {
    eval(i);
  }
}

static void reset_pose_to_bind(ecs::Skeleton& skel) {
  for (auto& bone : skel.bones) {
    bone.local_pose = bone.bind_local;
  }
}

static bool update_foot_lock_internal(bool& locked,
                                      Vec3& lock_pos,
                                      const Vec3& foot_pos,
                                      bool grounded,
                                      float swing_phase,
                                      float lock_in,
                                      float lock_out,
                                      float dt) {
  if (!grounded) {
    locked = false;
  }
  if (!locked && grounded && swing_phase <= lock_in) {
    locked = true;
    lock_pos = foot_pos;
  }
  if (locked && swing_phase >= lock_out) {
    locked = false;
  }
  const float alpha = 1.0f - std::exp(-dt * 16.0f);
  const Vec3 desired = locked ? lock_pos : foot_pos;
  lock_pos = lerp(lock_pos, desired, alpha);
  return locked;
}

struct RayHit {
  bool hit = false;
  float t = 0.0f;
  Vec3 normal = v3(0.0f, 1.0f, 0.0f);
  Vec3 point = v3();
};

static bool raycast_aabb(const Vec3& origin,
                         const Vec3& dir,
                         const Vec3& box_center,
                         const Vec3& half_extents,
                         float max_t,
                         RayHit& out) {
  Vec3 min_b = sub(box_center, half_extents);
  Vec3 max_b = add(box_center, half_extents);
  float tmin = 0.0f;
  float tmax = max_t;
  Vec3 hit_normal = v3();
  const float o[3] = {origin.x, origin.y, origin.z};
  const float d[3] = {dir.x, dir.y, dir.z};
  const float bmin[3] = {min_b.x, min_b.y, min_b.z};
  const float bmax[3] = {max_b.x, max_b.y, max_b.z};

  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(d[axis]) < kEps) {
      if (o[axis] < bmin[axis] || o[axis] > bmax[axis]) {
        return false;
      }
      continue;
    }
    float inv = 1.0f / d[axis];
    float t1 = (bmin[axis] - o[axis]) * inv;
    float t2 = (bmax[axis] - o[axis]) * inv;
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

  if (tmin < 0.0f || tmin > max_t) return false;
  out.hit = true;
  out.t = tmin;
  out.normal = hit_normal;
  out.point = add(origin, mul(dir, tmin));
  return true;
}

static RayHit raycast_down(const ecs::Registry& registry,
                           const Vec3& origin,
                           float max_dist,
                           float radius) {
  RayHit best{};
  best.t = max_dist;
  const Vec3 dir = v3(0.0f, -1.0f, 0.0f);
  for (const auto& kv : registry.colliders()) {
    const auto& collider = kv.second;
    if (collider.type == ecs::ColliderType::Plane) {
      Vec3 normal = v3(collider.normal[0], collider.normal[1], collider.normal[2]);
      const float nlen = length(normal);
      if (nlen < kEps) continue;
      normal = mul(normal, 1.0f / nlen);
      float plane_d = collider.distance + radius;
      if (const auto* t = registry.get_transform(kv.first)) {
        plane_d += dot(normal, from_array(t->position));
      }
      const float denom = dot(normal, dir);
      if (std::abs(denom) < kEps) continue;
      const float t = (plane_d - dot(normal, origin)) / denom;
      if (t >= 0.0f && t <= best.t) {
        best.hit = true;
        best.t = t;
        best.normal = normal;
        best.point = sub(add(origin, mul(dir, t)), mul(normal, radius));
      }
    } else if (collider.type == ecs::ColliderType::AABB) {
      Vec3 center = v3(collider.center[0], collider.center[1], collider.center[2]);
      if (const auto* t = registry.get_transform(kv.first)) {
        center = add(center, from_array(t->position));
      }
      Vec3 half_extents = v3(collider.half_extents[0], collider.half_extents[1], collider.half_extents[2]);
      half_extents.x += radius;
      half_extents.y += radius;
      half_extents.z += radius;
      RayHit hit{};
      if (raycast_aabb(origin, dir, center, half_extents, best.t, hit)) {
        if (hit.t <= best.t) {
          best = hit;
          best.point = sub(best.point, mul(best.normal, radius));
        }
      }
    }
  }
  return best;
}

static Vec3 solve_two_bone_ik_pole(const Vec3& hip,
                                   const Vec3& knee,
                                   const Vec3& ankle,
                                   const Vec3& target,
                                   const Vec3& pole_point,
                                   Vec3& prev_y,
                                   float* continuity_out) {
  const float l1 = length(sub(knee, hip));
  const float l2 = length(sub(ankle, knee));
  Vec3 to_t = sub(target, hip);
  float d_raw = length(to_t);
  if (d_raw < 0.0001f) d_raw = 0.0001f;
  const float min_d = std::max(0.001f, std::fabs(l1 - l2) + 0.001f);
  const float max_d = std::max(0.001f, l1 + l2 - 0.001f);
  const float d = clampf(d_raw, min_d, max_d);
  const Vec3 x = mul(to_t, 1.0f / d_raw);

  Vec3 z_raw = cross(x, sub(pole_point, hip));
  if (length(z_raw) < 0.0001f) {
    z_raw = cross(x, v3(0.0f, 1.0f, 0.0f));
    if (length(z_raw) < 0.0001f) {
      z_raw = cross(x, v3(1.0f, 0.0f, 0.0f));
    }
  }
  Vec3 z = normalize(z_raw);
  Vec3 y = normalize(cross(z, x));
  if (dot(y, sub(pole_point, hip)) < 0.0f) {
    y = mul(y, -1.0f);
    z = mul(z, -1.0f);
  }

  float cont = dot(prev_y, y);
  if (cont < 0.0f) {
    y = mul(y, -1.0f);
    z = mul(z, -1.0f);
    cont = -cont;
  }
  prev_y = y;
  if (continuity_out) *continuity_out = cont;

  const float along = (l1 * l1 - l2 * l2 + d * d) / (2.0f * d);
  float h_sq = l1 * l1 - along * along;
  if (h_sq < 0.0f) h_sq = 0.0f;
  const float height = std::sqrt(h_sq);

  return add(hip, add(mul(x, along), mul(y, height)));
}

static void apply_bone_aim(ecs::Skeleton& skel,
                           const std::vector<rkg::Mat4>& world,
                           uint32_t bone_idx,
                           uint32_t child_idx,
                           const Vec3& target_child_pos) {
  if (bone_idx == UINT32_MAX || child_idx == UINT32_MAX) return;
  if (bone_idx >= skel.bones.size() || child_idx >= skel.bones.size()) return;
  const Vec3 bone_pos = {world[bone_idx].m[12], world[bone_idx].m[13], world[bone_idx].m[14]};
  const Vec3 current_child_pos = {world[child_idx].m[12], world[child_idx].m[13], world[child_idx].m[14]};
  const Vec3 current_dir = sub(current_child_pos, bone_pos);
  const Vec3 desired_dir = sub(target_child_pos, bone_pos);
  if (length(current_dir) < 0.0001f || length(desired_dir) < 0.0001f) return;

  const rkg::Mat4 bone_world_rot = mat4_extract_rotation(world[bone_idx]);
  rkg::Mat4 parent_world_rot = rkg::mat4_identity();
  const int parent_idx = skel.bones[bone_idx].parent_index;
  if (parent_idx >= 0 && static_cast<size_t>(parent_idx) < world.size()) {
    parent_world_rot = mat4_extract_rotation(world[static_cast<size_t>(parent_idx)]);
  }
  const rkg::Mat4 parent_world_inv = mat4_transpose3x3(parent_world_rot);
  const rkg::Mat4 delta = mat4_rotation_from_to({current_dir.x, current_dir.y, current_dir.z},
                                                {desired_dir.x, desired_dir.y, desired_dir.z});
  const rkg::Mat4 new_world_rot = rkg::mat4_mul(delta, bone_world_rot);
  const rkg::Mat4 local_rot = rkg::mat4_mul(parent_world_inv, new_world_rot);
  const rkg::Vec3 euler = mat4_to_euler_xyz(local_rot);
  const float prev_x = skel.bones[bone_idx].local_pose.rotation[0];
  const float prev_y = skel.bones[bone_idx].local_pose.rotation[1];
  const float prev_z = skel.bones[bone_idx].local_pose.rotation[2];
  skel.bones[bone_idx].local_pose.rotation[0] = unwrap_angle(euler.x, prev_x);
  skel.bones[bone_idx].local_pose.rotation[1] = unwrap_angle(euler.y, prev_y);
  skel.bones[bone_idx].local_pose.rotation[2] = unwrap_angle(euler.z, prev_z);
}

static bool build_bone_map(ecs::ProceduralGait& gait, const ecs::Skeleton& skel) {
  if (skel.bones.empty()) return false;
  const std::vector<std::string> left_tags = {"left", "_l", ".l", " l", "l_", "l-", " left"};
  const std::vector<std::string> right_tags = {"right", "_r", ".r", " r", "r_", "r-", " right"};
  const std::vector<std::string> hips_tags = {"hips", "pelvis", "hip", "root", "body", "center"};
  const std::vector<std::string> spine_tags = {"spine"};
  const std::vector<std::string> chest_tags = {"chest", "spine02", "spine_02", "spine2"};
  const std::vector<std::string> neck_tags = {"neck", "neck_01"};
  const std::vector<std::string> head_tags = {"head"};
  const std::vector<std::string> thigh_tags = {"upleg", "upperleg", "thigh"};
  const std::vector<std::string> calf_tags = {"leg", "calf", "lowerleg", "shin"};
  const std::vector<std::string> foot_tags = {"foot", "ankle"};
  const std::vector<std::string> toe_tags = {"toe", "ball"};
  const std::vector<std::string> shoulder_tags = {"shoulder", "clavicle", "collar", "scapula"};
  const std::vector<std::string> upper_arm_tags = {"upperarm", "uparm", "arm"};
  const std::vector<std::string> lower_arm_tags = {"forearm", "lowerarm"};

  gait.bone_root = 0;
  gait.bone_hips = find_bone_index_by_name(skel, hips_tags, {});
  if (gait.bone_hips == UINT32_MAX) {
    gait.bone_hips = gait.bone_root;
  }
  gait.bone_spine = find_bone_index_by_name(skel, spine_tags, {"spine02", "spine_02", "spine2"});
  gait.bone_chest = find_bone_index_by_name(skel, chest_tags, {});
  gait.bone_neck = find_bone_index_by_name(skel, neck_tags, {});
  gait.bone_head = find_bone_index_by_name(skel, head_tags, {});

  gait.bone_l_thigh = find_side_bone(skel, left_tags, thigh_tags);
  gait.bone_r_thigh = find_side_bone(skel, right_tags, thigh_tags);
  gait.bone_l_calf = find_side_bone(skel, left_tags, calf_tags, thigh_tags);
  gait.bone_r_calf = find_side_bone(skel, right_tags, calf_tags, thigh_tags);
  gait.bone_l_foot = find_side_bone(skel, left_tags, foot_tags);
  gait.bone_r_foot = find_side_bone(skel, right_tags, foot_tags);
  gait.bone_l_toe = find_side_bone(skel, left_tags, toe_tags);
  gait.bone_r_toe = find_side_bone(skel, right_tags, toe_tags);
  gait.bone_l_shoulder = find_side_bone(skel, left_tags, shoulder_tags, upper_arm_tags);
  gait.bone_r_shoulder = find_side_bone(skel, right_tags, shoulder_tags, upper_arm_tags);
  gait.bone_l_upper_arm = find_side_bone(skel, left_tags, upper_arm_tags, lower_arm_tags);
  gait.bone_r_upper_arm = find_side_bone(skel, right_tags, upper_arm_tags, lower_arm_tags);
  gait.bone_l_lower_arm = find_side_bone(skel, left_tags, lower_arm_tags);
  gait.bone_r_lower_arm = find_side_bone(skel, right_tags, lower_arm_tags);

  if (gait.bone_l_thigh == UINT32_MAX || gait.bone_r_thigh == UINT32_MAX) {
    const uint32_t any_thigh = find_bone_index_by_name(skel, thigh_tags, {});
    if (gait.bone_l_thigh == UINT32_MAX) gait.bone_l_thigh = any_thigh;
    if (gait.bone_r_thigh == UINT32_MAX) gait.bone_r_thigh = any_thigh;
  }
  if (gait.bone_l_calf == UINT32_MAX || gait.bone_r_calf == UINT32_MAX) {
    const uint32_t any_calf = find_bone_index_by_name(skel, calf_tags, thigh_tags);
    if (gait.bone_l_calf == UINT32_MAX) gait.bone_l_calf = any_calf;
    if (gait.bone_r_calf == UINT32_MAX) gait.bone_r_calf = any_calf;
  }

  gait.map_valid = (gait.bone_hips != UINT32_MAX &&
                    gait.bone_l_thigh != UINT32_MAX && gait.bone_l_calf != UINT32_MAX &&
                    gait.bone_l_foot != UINT32_MAX && gait.bone_r_thigh != UINT32_MAX &&
                    gait.bone_r_calf != UINT32_MAX && gait.bone_r_foot != UINT32_MAX);
  return gait.map_valid;
}

static void compute_rig_metrics(ecs::ProceduralGait& gait, const ecs::Skeleton& skel) {
  std::vector<ecs::Transform> locals(skel.bones.size());
  for (size_t i = 0; i < skel.bones.size(); ++i) {
    locals[i] = skel.bones[i].local_pose;
  }
  std::vector<rkg::Mat4> world;
  compute_world_matrices(skel, locals, world);

  auto safe_pos = [&](uint32_t idx) -> Vec3 {
    if (idx == UINT32_MAX || idx >= world.size()) return v3();
    return {world[idx].m[12], world[idx].m[13], world[idx].m[14]};
  };

  Vec3 l_hip = safe_pos(gait.bone_l_thigh);
  Vec3 l_knee = safe_pos(gait.bone_l_calf);
  Vec3 l_foot = safe_pos(gait.bone_l_foot);
  Vec3 r_hip = safe_pos(gait.bone_r_thigh);
  Vec3 r_knee = safe_pos(gait.bone_r_calf);
  Vec3 r_foot = safe_pos(gait.bone_r_foot);
  Vec3 hips_pos = safe_pos(gait.bone_hips);

  float l_leg = length(sub(l_knee, l_hip)) + length(sub(l_foot, l_knee));
  float r_leg = length(sub(r_knee, r_hip)) + length(sub(r_foot, r_knee));
  float leg_len = (l_leg > kEps) ? l_leg : r_leg;
  if (leg_len < kEps) leg_len = 1.0f;
  gait.leg_length = leg_len;

  float hip_width = length(sub(l_hip, r_hip));
  if (hip_width < kEps) hip_width = leg_len * 0.35f;
  gait.hip_width = hip_width;

  Vec3 l_toe = safe_pos(gait.bone_l_toe);
  Vec3 r_toe = safe_pos(gait.bone_r_toe);
  float foot_len = length(sub(l_toe, l_foot));
  if (foot_len < kEps) foot_len = length(sub(r_toe, r_foot));
  if (foot_len < kEps) foot_len = leg_len * 0.15f;
  gait.foot_length = foot_len;

  Vec3 l_dir = normalize(sub(l_foot, l_hip));
  Vec3 r_dir = normalize(sub(r_foot, r_hip));
  Vec3 l_knee_off = sub(l_knee, l_hip);
  Vec3 r_knee_off = sub(r_knee, r_hip);
  Vec3 l_pole = sub(l_knee_off, mul(l_dir, dot(l_knee_off, l_dir)));
  Vec3 r_pole = sub(r_knee_off, mul(r_dir, dot(r_knee_off, r_dir)));
  l_pole = normalize(l_pole);
  r_pole = normalize(r_pole);
  if (length(l_pole) < kEps) l_pole = v3(0.0f, 0.0f, 1.0f);
  if (length(r_pole) < kEps) r_pole = v3(0.0f, 0.0f, 1.0f);
  to_array(l_pole, gait.knee_plane_l);
  to_array(r_pole, gait.knee_plane_r);
  to_array(l_pole, gait.knee_prev_y_l);
  to_array(r_pole, gait.knee_prev_y_r);

  Vec3 home_l = sub(l_foot, hips_pos);
  Vec3 home_r = sub(r_foot, hips_pos);
  to_array(home_l, gait.foot_home_l);
  to_array(home_r, gait.foot_home_r);

  Vec3 right_bind = normalize(sub(r_hip, l_hip));
  if (length(right_bind) < kEps) right_bind = v3(1.0f, 0.0f, 0.0f);
  float sign_l = (dot(home_l, right_bind) < 0.0f) ? -1.0f : 1.0f;
  float sign_r = (dot(home_r, right_bind) < 0.0f) ? -1.0f : 1.0f;
  if (sign_l == sign_r) {
    sign_l = -1.0f;
    sign_r = 1.0f;
  }
  gait.side_sign_l = sign_l;
  gait.side_sign_r = sign_r;
}

static void add_rot(ecs::Skeleton& skel, uint32_t idx, float rx, float ry, float rz) {
  if (idx == UINT32_MAX || idx >= skel.bones.size()) return;
  skel.bones[idx].local_pose.rotation[0] += rx;
  skel.bones[idx].local_pose.rotation[1] += ry;
  skel.bones[idx].local_pose.rotation[2] += rz;
}

static void add_pos(ecs::Skeleton& skel, uint32_t idx, float px, float py, float pz) {
  if (idx == UINT32_MAX || idx >= skel.bones.size()) return;
  skel.bones[idx].local_pose.position[0] += px;
  skel.bones[idx].local_pose.position[1] += py;
  skel.bones[idx].local_pose.position[2] += pz;
}

static float closest_points_on_segments(const Vec3& p1, const Vec3& q1,
                                        const Vec3& p2, const Vec3& q2,
                                        Vec3& c1, Vec3& c2,
                                        float& s, float& t) {
  const Vec3 d1 = sub(q1, p1);
  const Vec3 d2 = sub(q2, p2);
  const Vec3 r = sub(p1, p2);
  const float a = dot(d1, d1);
  const float e = dot(d2, d2);
  const float f = dot(d2, r);
  s = 0.0f;
  t = 0.0f;

  if (a <= kEps && e <= kEps) {
    c1 = p1;
    c2 = p2;
    return length(sub(c1, c2));
  }
  if (a <= kEps) {
    t = clampf(f / e, 0.0f, 1.0f);
    c1 = p1;
    c2 = add(p2, mul(d2, t));
    return length(sub(c1, c2));
  }
  const float c = dot(d1, r);
  if (e <= kEps) {
    s = clampf(-c / a, 0.0f, 1.0f);
    c1 = add(p1, mul(d1, s));
    c2 = p2;
    return length(sub(c1, c2));
  }
  const float b = dot(d1, d2);
  const float denom = a * e - b * b;
  if (denom > kEps) {
    s = clampf((b * f - c * e) / denom, 0.0f, 1.0f);
  } else {
    s = 0.0f;
  }
  t = (b * s + f) / e;
  if (t < 0.0f) {
    t = 0.0f;
    s = clampf(-c / a, 0.0f, 1.0f);
  } else if (t > 1.0f) {
    t = 1.0f;
    s = clampf((b - c) / a, 0.0f, 1.0f);
  }
  c1 = add(p1, mul(d1, s));
  c2 = add(p2, mul(d2, t));
  return length(sub(c1, c2));
}

static float capsule_radius_from_name(const std::string& name, float leg_len) {
  const std::string n = to_lower(name);
  if (name.empty()) return 0.07f * leg_len;
  if (name_has(n, {"hips", "pelvis", "spine", "chest", "torso"})) return 0.14f * leg_len;
  if (name_has(n, {"thigh", "upleg", "upperleg"})) return 0.11f * leg_len;
  if (name_has(n, {"calf", "lowerleg", "shin"})) return 0.09f * leg_len;
  if (name_has(n, {"upperarm", "uparm", "arm"})) return 0.08f * leg_len;
  if (name_has(n, {"forearm", "lowerarm"})) return 0.07f * leg_len;
  if (name_has(n, {"hand", "wrist"})) return 0.06f * leg_len;
  if (name_has(n, {"head", "neck"})) return 0.12f * leg_len;
  return 0.07f * leg_len;
}

static float joint_limit_from_name(const std::string& name) {
  const std::string n = to_lower(name);
  if (name_has(n, {"spine", "chest", "neck"})) return 0.45f;
  if (name_has(n, {"head"})) return 0.60f;
  if (name_has(n, {"upperarm", "uparm"})) return 1.15f;
  if (name_has(n, {"forearm", "lowerarm"})) return 1.05f;
  if (name_has(n, {"thigh", "upleg", "upperleg"})) return 1.10f;
  if (name_has(n, {"calf", "lowerleg", "shin"})) return 1.00f;
  return 1.20f;
}

static void ensure_self_collision_cache(ecs::ProceduralGait& gait,
                                        const ecs::Skeleton& skel,
                                        const std::vector<rkg::Mat4>& world,
                                        float leg_len) {
  const size_t count = skel.bones.size();
  if (gait.bone_rest_len.size() == count &&
      gait.bone_radius.size() == count &&
      gait.bone_max_angle.size() == count &&
      gait.bone_collide.size() == count) {
    return;
  }
  gait.bone_rest_len.assign(count, 0.0f);
  gait.bone_radius.assign(count, 0.0f);
  gait.bone_max_angle.assign(count, 0.0f);
  gait.bone_collide.assign(count, 0);
  for (size_t i = 0; i < count; ++i) {
    const int parent = skel.bones[i].parent_index;
    if (parent < 0 || static_cast<size_t>(parent) >= count) continue;
    const Vec3 p = {world[parent].m[12], world[parent].m[13], world[parent].m[14]};
    const Vec3 c = {world[i].m[12], world[i].m[13], world[i].m[14]};
    const float rest = length(sub(c, p));
    gait.bone_rest_len[i] = rest;
    gait.bone_radius[i] = capsule_radius_from_name(skel.bones[i].name, leg_len);
    gait.bone_max_angle[i] = joint_limit_from_name(skel.bones[i].name);
    gait.bone_collide[i] = (rest > 0.05f * leg_len) ? 1u : 0u;
  }
}

static void build_external_capsules(ecs::Registry& registry, std::vector<CapsuleSeg>& out) {
  out.clear();
  for (auto& kv : registry.procedural_gaits()) {
    const ecs::Entity entity = kv.first;
    auto& gait = kv.second;
    if (!gait.enabled || !gait.enable_self_collision) continue;
    const auto* skeleton = registry.get_skeleton(entity);
    const auto* transform = registry.get_transform(entity);
    if (!skeleton || !transform || skeleton->bones.empty()) continue;
    std::vector<ecs::Transform> locals(skeleton->bones.size());
    for (size_t i = 0; i < skeleton->bones.size(); ++i) {
      locals[i] = skeleton->bones[i].local_pose;
    }
    std::vector<rkg::Mat4> world;
    compute_world_matrices(*skeleton, locals, world);
    ensure_self_collision_cache(gait, *skeleton, world, std::max(gait.leg_length, 1.0f));

    const float sx = std::abs(transform->scale[0]);
    const float sy = std::abs(transform->scale[1]);
    const float sz = std::abs(transform->scale[2]);
    float rig_scale = (sx + sy + sz) * (1.0f / 3.0f);
    if (rig_scale < 0.0001f) rig_scale = 1.0f;

    for (size_t i = 0; i < skeleton->bones.size(); ++i) {
      if (!gait.bone_collide[i]) continue;
      const int parent = skeleton->bones[i].parent_index;
      if (parent < 0 || static_cast<size_t>(parent) >= skeleton->bones.size()) continue;
      const Vec3 p = {world[parent].m[12], world[parent].m[13], world[parent].m[14]};
      const Vec3 c = {world[i].m[12], world[i].m[13], world[i].m[14]};
      CapsuleSeg seg{};
      seg.a = to_world_point(*transform, p);
      seg.b = to_world_point(*transform, c);
      seg.r = gait.bone_radius[i] * rig_scale;
      seg.owner = entity;
      out.push_back(seg);
    }
  }
}

struct DebugLines {
  std::vector<float> pos;
  std::vector<float> color;
  uint32_t count = 0;
  void clear() {
    pos.clear();
    color.clear();
    count = 0;
  }
  void append_from(const rkg::VulkanViewportLineList* list) {
    if (!list || list->line_count == 0) return;
    const uint32_t max_copy = std::min(list->line_count,
                                       rkg::VulkanViewportLineList::kMaxLines - count);
    pos.reserve((count + max_copy) * 6);
    color.reserve((count + max_copy) * 4);
    for (uint32_t i = 0; i < max_copy; ++i) {
      const uint32_t p = i * 6;
      pos.push_back(list->positions[p + 0]);
      pos.push_back(list->positions[p + 1]);
      pos.push_back(list->positions[p + 2]);
      pos.push_back(list->positions[p + 3]);
      pos.push_back(list->positions[p + 4]);
      pos.push_back(list->positions[p + 5]);
      const uint32_t c = i * 4;
      color.push_back(list->colors[c + 0]);
      color.push_back(list->colors[c + 1]);
      color.push_back(list->colors[c + 2]);
      color.push_back(list->colors[c + 3]);
      ++count;
    }
  }
  void add_line(const Vec3& a, const Vec3& b, const Vec3& c) {
    if (count >= rkg::VulkanViewportLineList::kMaxLines) return;
    pos.push_back(a.x); pos.push_back(a.y); pos.push_back(a.z);
    pos.push_back(b.x); pos.push_back(b.y); pos.push_back(b.z);
    color.push_back(c.x); color.push_back(c.y); color.push_back(c.z); color.push_back(1.0f);
    ++count;
  }
};

static Vec3 rotate_y(const Vec3& v, float angle) {
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  return {v.x * c + v.z * s, v.y, -v.x * s + v.z * c};
}

static void solve_self_collision(ecs::ProceduralGait& gait,
                                 ecs::Skeleton& skel,
                                 const ecs::Registry& registry,
                                 ecs::Entity self,
                                 const ecs::Transform& root,
                                 const std::vector<CapsuleSeg>* external_caps,
                                 std::vector<ecs::Transform>& locals,
                                 std::vector<rkg::Mat4>& world,
                                 float leg_len,
                                 float dt) {
  if (!gait.enable_self_collision) return;
  if (skel.bones.empty()) return;

  ensure_self_collision_cache(gait, skel, world, leg_len);
  const size_t count = skel.bones.size();
  std::vector<Vec3> pos(count);
  std::vector<Vec3> target(count);
  std::vector<Vec3> rest_dir(count);
  std::vector<int> child_count(count, 0);
  std::vector<int> first_child(count, -1);

  for (size_t i = 0; i < count; ++i) {
    pos[i] = {world[i].m[12], world[i].m[13], world[i].m[14]};
    target[i] = pos[i];
    const int parent = skel.bones[i].parent_index;
    if (parent >= 0 && static_cast<size_t>(parent) < count) {
      if (child_count[parent] == 0) first_child[parent] = static_cast<int>(i);
      ++child_count[parent];
    }
  }
  for (size_t i = 0; i < count; ++i) {
    const int parent = skel.bones[i].parent_index;
    if (parent < 0 || static_cast<size_t>(parent) >= count) continue;
    Vec3 d = sub(target[i], target[parent]);
    rest_dir[i] = normalize(d);
  }

  const int root_bone = (gait.bone_root != UINT32_MAX) ? static_cast<int>(gait.bone_root) : 0;
  const int iters = std::max(1, gait.self_collision_iters);
  const float stiffness = clampf(gait.self_collision_stiffness, 0.0f, 1.0f);

  for (int iter = 0; iter < iters; ++iter) {
    // length constraints
    for (size_t i = 0; i < count; ++i) {
      if (!gait.bone_collide[i]) continue;
      const int parent = skel.bones[i].parent_index;
      if (parent < 0 || static_cast<size_t>(parent) >= count) continue;
      const float rest = gait.bone_rest_len[i];
      Vec3 delta = sub(pos[i], pos[parent]);
      const float len = length(delta);
      if (len < kEps) continue;
      const float diff = (len - rest) / len;
      const Vec3 corr = mul(delta, 0.5f * diff * stiffness);
      if (parent != root_bone) pos[parent] = add(pos[parent], corr);
      pos[i] = sub(pos[i], corr);
    }

    // self-collision between bone capsules
    for (size_t i = 0; i < count; ++i) {
      if (!gait.bone_collide[i]) continue;
      const int parent_i = skel.bones[i].parent_index;
      if (parent_i < 0 || static_cast<size_t>(parent_i) >= count) continue;
      const float ri = gait.bone_radius[i];
      const Vec3 a0 = pos[parent_i];
      const Vec3 a1 = pos[i];
      for (size_t j = i + 1; j < count; ++j) {
        if (!gait.bone_collide[j]) continue;
        const int parent_j = skel.bones[j].parent_index;
        if (parent_j < 0 || static_cast<size_t>(parent_j) >= count) continue;
        if (parent_i == static_cast<int>(j) || parent_j == static_cast<int>(i)) continue;
        if (parent_i == parent_j) continue;
        const float rj = gait.bone_radius[j];
        const Vec3 b0 = pos[parent_j];
        const Vec3 b1 = pos[j];
        Vec3 c1{};
        Vec3 c2{};
        float s = 0.0f;
        float t = 0.0f;
        const float dist = closest_points_on_segments(a0, a1, b0, b1, c1, c2, s, t);
        const float min_dist = ri + rj;
        if (dist < min_dist) {
          Vec3 n = (dist > kEps) ? mul(sub(c1, c2), 1.0f / dist) : v3(1.0f, 0.0f, 0.0f);
          const float push = (min_dist - dist) * 0.5f * stiffness;
          const Vec3 d = mul(n, push);
          if (parent_i != root_bone) pos[parent_i] = add(pos[parent_i], d);
          pos[i] = add(pos[i], d);
          if (parent_j != root_bone) pos[parent_j] = sub(pos[parent_j], d);
          pos[j] = sub(pos[j], d);
        }
      }
    }

    // collide endpoints with world colliders (approximate as spheres)
    for (size_t i = 0; i < count; ++i) {
      if (!gait.bone_collide[i]) continue;
      const int parent = skel.bones[i].parent_index;
      if (parent < 0 || static_cast<size_t>(parent) >= count) continue;
      const float r = gait.bone_radius[i];
      Vec3 points_local[2] = {pos[parent], pos[i]};
      Vec3 points[2] = {to_world_point(root, points_local[0]),
                        to_world_point(root, points_local[1])};
      for (const auto& kv : registry.colliders()) {
        const auto& collider = kv.second;
        if (collider.type == ecs::ColliderType::Plane) {
          const Vec3 normal = v3(collider.normal[0], collider.normal[1], collider.normal[2]);
          float d = collider.distance;
          if (const auto* plane_transform = registry.get_transform(kv.first)) {
            const Vec3 plane_pos = from_array(plane_transform->position);
            d += dot(normal, plane_pos);
          }
          for (auto& p : points) {
            const float dist = dot(p, normal) - d;
            if (dist < r) {
              p = add(p, mul(normal, (r - dist)));
            }
          }
        } else if (collider.type == ecs::ColliderType::AABB) {
          const Vec3 center = v3(collider.center[0], collider.center[1], collider.center[2]);
          const Vec3 half = v3(collider.half_extents[0], collider.half_extents[1], collider.half_extents[2]);
          for (auto& p : points) {
            Vec3 local = sub(p, center);
            Vec3 clamped = v3(clampf(local.x, -half.x, half.x),
                              clampf(local.y, -half.y, half.y),
                              clampf(local.z, -half.z, half.z));
            Vec3 closest = add(center, clamped);
            Vec3 delta = sub(p, closest);
            const float dist_sq = length_sq(delta);
            if (dist_sq < r * r) {
              if (dist_sq > kEps) {
                const float dist = std::sqrt(dist_sq);
                p = add(p, mul(delta, (r - dist) / dist));
              } else {
                const float px = half.x - std::abs(local.x);
                const float py = half.y - std::abs(local.y);
                const float pz = half.z - std::abs(local.z);
                if (px <= py && px <= pz) {
                  const float sign = (local.x >= 0.0f) ? 1.0f : -1.0f;
                  p.x = center.x + sign * (half.x + r);
                } else if (py <= pz) {
                  const float sign = (local.y >= 0.0f) ? 1.0f : -1.0f;
                  p.y = center.y + sign * (half.y + r);
                } else {
                  const float sign = (local.z >= 0.0f) ? 1.0f : -1.0f;
                  p.z = center.z + sign * (half.z + r);
                }
              }
            }
          }
        }
      }
      points_local[0] = to_local_point(root, points[0]);
      points_local[1] = to_local_point(root, points[1]);
      pos[parent] = points_local[0];
      pos[i] = points_local[1];
    }

    // collide with external capsules (other entities), push only this skeleton
    if (external_caps && !external_caps->empty()) {
      for (size_t i = 0; i < count; ++i) {
        if (!gait.bone_collide[i]) continue;
        const int parent_i = skel.bones[i].parent_index;
        if (parent_i < 0 || static_cast<size_t>(parent_i) >= count) continue;
        const float ri = gait.bone_radius[i];
        const Vec3 a0 = to_world_point(root, pos[parent_i]);
        const Vec3 a1 = to_world_point(root, pos[i]);
        for (const auto& ext : *external_caps) {
          if (ext.owner == self) continue;
          Vec3 c1{};
          Vec3 c2{};
          float s = 0.0f;
          float t = 0.0f;
          const float dist = closest_points_on_segments(a0, a1, ext.a, ext.b, c1, c2, s, t);
          const float min_dist = ri + ext.r;
          if (dist < min_dist) {
            Vec3 n = (dist > kEps) ? mul(sub(c1, c2), 1.0f / dist) : v3(1.0f, 0.0f, 0.0f);
            const float push = (min_dist - dist) * stiffness;
            const Vec3 d = mul(n, push);
            Vec3 local_push = to_local_offset(root, d);
            if (parent_i != root_bone) pos[parent_i] = add(pos[parent_i], mul(local_push, 0.5f));
            pos[i] = add(pos[i], mul(local_push, 0.5f));
          }
        }
      }
    }

    // joint cone limits (keep near animated direction)
    for (size_t i = 0; i < count; ++i) {
      if (!gait.bone_collide[i]) continue;
      const int parent = skel.bones[i].parent_index;
      if (parent < 0 || static_cast<size_t>(parent) >= count) continue;
      const float max_ang = gait.bone_max_angle[i];
      if (max_ang <= 0.0f) continue;
      Vec3 dir = sub(pos[i], pos[parent]);
      const float len = length(dir);
      if (len < kEps) continue;
      dir = mul(dir, 1.0f / len);
      const Vec3 rest = rest_dir[i];
      if (length(rest) < kEps) continue;
      const float cosang = clampf(dot(dir, rest), -1.0f, 1.0f);
      const float angle = std::acos(cosang);
      if (angle > max_ang) {
        const float t = max_ang / angle;
        Vec3 new_dir = normalize(lerp(rest, dir, t));
        pos[i] = add(pos[parent], mul(new_dir, gait.bone_rest_len[i]));
      }
    }

    // stay near animated pose
    for (size_t i = 0; i < count; ++i) {
      if (!gait.bone_collide[i]) continue;
      const Vec3 delta = sub(target[i], pos[i]);
      pos[i] = add(pos[i], mul(delta, 0.15f));
    }
  }

  // apply adjustments back to skeleton (only for single-child bones)
  for (int pass = 0; pass < 2; ++pass) {
    for (size_t i = 0; i < count; ++i) {
      locals[i] = skel.bones[i].local_pose;
    }
    compute_world_matrices(skel, locals, world);
    for (size_t i = 0; i < count; ++i) {
      const int parent = skel.bones[i].parent_index;
      if (parent < 0 || static_cast<size_t>(parent) >= count) continue;
      if (!gait.bone_collide[i]) continue;
      if (child_count[parent] != 1) continue;
      apply_bone_aim(skel, world, static_cast<uint32_t>(parent), static_cast<uint32_t>(i), pos[i]);
    }
  }

  for (size_t i = 0; i < count; ++i) {
    locals[i] = skel.bones[i].local_pose;
  }
  compute_world_matrices(skel, locals, world);
  (void)dt;
}

} // namespace

void update_procedural_gait(ecs::Registry& registry, ecs::Entity entity, float dt) {
  auto* gait = registry.get_procedural_gait(entity);
  auto* skeleton = registry.get_skeleton(entity);
  auto* transform = registry.get_transform(entity);
  if (!gait || !skeleton || !transform) return;
  if (!gait->enabled) return;

  if (!gait->map_valid) {
    if (!build_bone_map(*gait, *skeleton)) {
      return;
    }
    compute_rig_metrics(*gait, *skeleton);
  }

  reset_pose_to_bind(*skeleton);

  auto* controller = registry.get_character_controller(entity);
  auto* velocity = registry.get_velocity(entity);
  const Vec3 vel = velocity ? from_array(velocity->linear) : v3();
  Vec3 planar = v3(vel.x, 0.0f, vel.z);
  float speed = length(planar);

  const float smooth_tau = std::max(gait->input_smooth_tau, 0.0f);
  const float smooth_alpha = (smooth_tau > 0.0f) ? (1.0f - std::exp(-dt / smooth_tau)) : 1.0f;
  const float speed_raw = speed;
  gait->speed_smoothed += (speed - gait->speed_smoothed) * smooth_alpha;
  speed = gait->speed_smoothed;

  const bool grounded = controller ? controller->grounded : false;
  float input_mag_raw = 0.0f;
  if (controller) {
    input_mag_raw = controller->raw_input_mag;
    if (input_mag_raw <= 0.0f) {
      Vec3 in{controller->smoothed_input[0], 0.0f, controller->smoothed_input[2]};
      input_mag_raw = length(in);
    }
  }
  const bool sprinting = controller ? controller->is_sprinting : false;
  float max_speed = gait->walk_speed;
  if (controller) {
    max_speed = controller->max_speed;
  }
  if (sprinting) {
    max_speed = (controller ? controller->max_speed * controller->sprint_multiplier : gait->sprint_speed);
  }
  if (max_speed < 0.01f) max_speed = gait->walk_speed;

  const float speed_norm = saturate(speed / std::max(max_speed, 0.01f));
  float gait_speed_ref = gait->walk_speed;
  if (sprinting) gait_speed_ref = gait->sprint_speed;
  if (gait_speed_ref < 0.01f) gait_speed_ref = max_speed;
  const float speed_gait_norm = saturate(speed / std::max(gait_speed_ref, 0.01f));
  gait->yaw = transform->rotation[1];
  const float yaw_delta = wrap_pi(gait->yaw - gait->last_yaw);
  gait->last_yaw = gait->yaw;
  const float yaw_rate_raw = yaw_delta / std::max(dt, 0.0001f);
  const float yaw_alpha = 1.0f - std::exp(-dt * 12.0f);
  gait->yaw_rate += (yaw_rate_raw - gait->yaw_rate) * yaw_alpha;
  gait->yaw_rate = clampf(gait->yaw_rate, -6.0f, 6.0f);

  if (grounded && !gait->was_grounded) {
    gait->landing_timer = 0.18f;
  }
  gait->was_grounded = grounded;
  if (gait->landing_timer > 0.0f) {
    gait->landing_timer = std::max(0.0f, gait->landing_timer - dt);
  }

  const bool turn_in_place = gait->enable_turn_in_place &&
                             grounded &&
                             speed_raw < gait->turn_in_place_speed &&
                             input_mag_raw < 0.15f &&
                             std::abs(gait->yaw_rate) > 0.6f;

  const float sx = std::abs(transform->scale[0]);
  const float sy = std::abs(transform->scale[1]);
  const float sz = std::abs(transform->scale[2]);
  float rig_scale = (sx + sy + sz) * (1.0f / 3.0f);
  if (rig_scale < 0.0001f) rig_scale = 1.0f;
  const float inv_rig_scale = 1.0f / rig_scale;
  const float leg_len_e = gait->leg_length;
  const float leg_len_world = leg_len_e * rig_scale;

  const float walk_cadence = 1.6f;
  const float run_cadence = 2.6f;
  const float cadence = walk_cadence + (run_cadence - walk_cadence) * speed_gait_norm;

  const float speed_e = speed * inv_rig_scale;
  const float stride_scale = std::max(gait->stride_scale, 0.1f);
  float stride_len_e = (cadence > 0.1f) ? (speed_e / cadence) : leg_len_e;
  stride_len_e *= stride_scale;
  stride_len_e = clampf(stride_len_e, 0.50f * leg_len_e, 1.05f * leg_len_e);
  float cycle_rate_hz = (stride_len_e > kEps) ? (speed_e / stride_len_e) : 0.0f;
  if (turn_in_place) {
    const float turn_scale = clampf(std::abs(gait->yaw_rate) / 1.2f, 0.5f, 1.5f);
    cycle_rate_hz = cadence * gait->turn_step_rate * turn_scale;
  } else {
    const float max_cycle = cadence * 1.05f;
    if (cycle_rate_hz > max_cycle) cycle_rate_hz = max_cycle;
  }
  if (speed > 0.01f || turn_in_place) {
    const float gait_speed_scale = 1.0f;
    gait->phase += cycle_rate_hz * dt * 2.0f * kPi * gait_speed_scale;
    if (gait->phase > 2.0f * kPi) gait->phase = std::fmod(gait->phase, 2.0f * kPi);
  }

  const float cycle = frac(gait->phase / (2.0f * kPi));
  float stance_fraction = 0.65f + (0.55f - 0.65f) * speed_gait_norm;
  if (turn_in_place) {
    stance_fraction = 0.55f;
  }
  const float u_l = frac(cycle + 0.0f);
  const float u_r = frac(cycle + 0.5f);
  const bool left_stance = grounded && (u_l < stance_fraction);
  const bool right_stance = grounded && (u_r < stance_fraction);
  const bool left_swing = grounded && !left_stance;
  const bool right_swing = grounded && !right_stance;
  const float left_stance_u = left_stance ? (u_l / std::max(stance_fraction, 0.001f)) : 0.0f;
  const float right_stance_u = right_stance ? (u_r / std::max(stance_fraction, 0.001f)) : 0.0f;
  const float left_swing_u = left_swing ? ((u_l - stance_fraction) / std::max(1.0f - stance_fraction, 0.001f)) : 0.0f;
  const float right_swing_u = right_swing ? ((u_r - stance_fraction) / std::max(1.0f - stance_fraction, 0.001f)) : 0.0f;

  float stance_sign = 0.0f;
  float stance_u = 0.0f;
  if (left_stance && !right_stance) {
    stance_sign = -1.0f;
    stance_u = left_stance_u;
  } else if (right_stance && !left_stance) {
    stance_sign = 1.0f;
    stance_u = right_stance_u;
  } else if (left_stance && right_stance) {
    if (u_l <= u_r) {
      stance_sign = -1.0f;
      stance_u = left_stance_u;
    } else {
      stance_sign = 1.0f;
      stance_u = right_stance_u;
    }
  }

  const Vec3 forward = {std::sin(gait->yaw), 0.0f, std::cos(gait->yaw)};
  const Vec3 right = {std::cos(gait->yaw), 0.0f, -std::sin(gait->yaw)};

  Vec3 accel = v3();
  if (dt > 0.0f) {
    const Vec3 last_vel = from_array(gait->last_velocity);
    accel = mul(sub(vel, last_vel), 1.0f / std::max(dt, 0.0001f));
  }
  to_array(vel, gait->last_velocity);
  const float accel_fwd = dot(accel, forward);
  const float accel_side = dot(accel, right);
  const float lean_fwd_raw = clampf(accel_fwd / std::max(max_speed, 0.1f), -1.0f, 1.0f) * gait->pelvis_lean_scale;
  const float lean_side_raw = clampf(accel_side / std::max(max_speed, 0.1f), -1.0f, 1.0f) * gait->pelvis_lean_scale;
  const float lean_alpha = 1.0f - std::exp(-dt * 10.0f);
  gait->lean_fwd += (lean_fwd_raw - gait->lean_fwd) * lean_alpha;
  gait->lean_side += (lean_side_raw - gait->lean_side) * lean_alpha;

  const float landing_alpha = (gait->landing_timer > 0.0f) ? (gait->landing_timer / 0.18f) : 0.0f;
  const float landing_drop = -gait->landing_compress * leg_len_e * landing_alpha;

  const float pelvis_bob = gait->pelvis_bob_scale * leg_len_e * speed_norm;
  const float pelvis_width_ref_e = std::max(gait->hip_width, 0.20f * leg_len_e);
  float step_half = 0.36f * pelvis_width_ref_e + (0.28f * pelvis_width_ref_e - 0.36f * pelvis_width_ref_e) * speed_gait_norm;
  step_half = std::max(step_half, 0.10f * leg_len_e);
  step_half = std::min(step_half, 0.22f * leg_len_e);
  float pelvis_sway_amp = gait->pelvis_sway_scale * (0.35f * step_half);
  pelvis_sway_amp *= (0.6f + 0.4f * speed_norm);
  pelvis_sway_amp *= 0.20f;
  const float pelvis_roll = gait->pelvis_roll_scale * speed_norm;
  const float sway_phase = 2.0f * kPi * cycle;
  const float sway = std::sin(sway_phase);
  const float rock = sway;
  const float twist = std::cos(sway_phase);
  const float pelvis_x = pelvis_sway_amp * sway;
  const float pelvis_y = -pelvis_bob * std::sin(kPi * stance_u);
  const float strafe_factor =
      1.0f - 0.4f * std::min(1.0f, std::abs(dot(planar, right)) / std::max(max_speed, 0.1f));
  const float swing_scale = 0.25f + 0.75f * speed_norm;
  const float arm_amp = (gait->arm_swing_scale * speed_norm + 0.02f) * strafe_factor * swing_scale;
  const float pelvis_yaw = 0.06f * arm_amp * twist;
  const float torso_yaw = -0.08f * arm_amp * twist;
  const float rock_amp = (0.08f + 0.22f * speed_norm);
  const float torso_roll = 0.5f * rock_amp * rock;
  const float chest_roll = 1.0f * rock_amp * rock;

  if (gait->enable_pelvis_motion) {
    add_pos(*skeleton, gait->bone_hips, pelvis_x, pelvis_y + landing_drop, 0.0f);
    const float pelvis_roll_term = -pelvis_roll * rock * 0.10f;
    add_rot(*skeleton, gait->bone_hips, -gait->lean_fwd * 0.25f, pelvis_yaw, pelvis_roll_term);
  }

  const float lean_side_spine = 0.0f;
  const float lean_side_chest = 0.0f;
  const float lean_side_neck = 0.0f;
  const float lean_side_head = 0.0f;
  const float breathe = std::sin(gait->idle_time * 2.0f * kPi * 0.22f);
  const float breathe_amp = 0.03f * gait->idle_blend;
  const float breathe_term = breathe_amp * breathe;
  add_rot(*skeleton, gait->bone_spine, gait->lean_fwd * 0.2f + breathe_term * 0.4f, torso_yaw * 0.35f,
          lean_side_spine + torso_roll * 0.45f);
  add_rot(*skeleton, gait->bone_chest, gait->lean_fwd * 0.15f + breathe_term, torso_yaw * 0.75f,
          lean_side_chest + chest_roll);
  add_rot(*skeleton, gait->bone_neck, gait->lean_fwd * 0.06f + breathe_term * 0.25f, torso_yaw * 0.2f,
          lean_side_neck + torso_roll * 0.25f);
  add_rot(*skeleton, gait->bone_head, gait->lean_fwd * 0.04f + breathe_term * 0.15f, torso_yaw * 0.1f,
          lean_side_head + torso_roll * 0.18f);

  if (gait->enable_arm_swing) {
    const float phase_offset = 0.25f;
    const float arm_phase_l = 2.0f * kPi * u_l + phase_offset;
    const float arm_phase_r = 2.0f * kPi * u_r + phase_offset;
    const float swing_l = std::sin(arm_phase_l);
    const float swing_r = std::sin(arm_phase_r);
    const float swing_l_90 = std::sin(arm_phase_l + 0.5f * kPi);
    const float swing_r_90 = std::sin(arm_phase_r + 0.5f * kPi);
    const float arm_pitch_amp = 0.60f * arm_amp;
    const float arm_yaw = 0.02f * arm_amp;
    const float arm_roll = 0.03f * arm_amp;
    const float arm_lift = 0.03f * arm_amp;
    const float elbow_amp = 0.12f * arm_amp;
    const float elbow_phase = 0.20f;
    const float shoulder_pitch_amp = 0.20f * arm_amp;
    const float shoulder_yaw_amp = 0.08f * arm_amp;
    const float shoulder_roll_amp = 0.10f * arm_amp;
    const float relax = gait->idle_blend;
    const float relax_pitch = 0.18f * relax;
    const float relax_yaw = 0.02f * relax;
    const float relax_elbow = 0.08f * relax;
    const float arm_out = gait->arm_out * (0.3f + 0.7f * relax);
    const float idle_arm = 0.06f * relax * std::sin(gait->idle_time * 2.0f * kPi * 0.25f);
    add_rot(*skeleton, gait->bone_l_shoulder, shoulder_pitch_amp * swing_r + relax_pitch * 0.5f + idle_arm * 0.4f,
            shoulder_yaw_amp * swing_r_90 + relax_yaw, shoulder_roll_amp * swing_r + arm_out * 0.5f);
    add_rot(*skeleton, gait->bone_r_shoulder, shoulder_pitch_amp * swing_l + relax_pitch * 0.5f - idle_arm * 0.4f,
            -shoulder_yaw_amp * swing_l_90 - relax_yaw, -shoulder_roll_amp * swing_l - arm_out * 0.5f);
    add_rot(*skeleton, gait->bone_l_upper_arm, arm_pitch_amp * swing_r + arm_lift + relax_pitch + idle_arm,
            arm_yaw * swing_r_90, gait->arm_tuck + arm_roll * swing_r + arm_out);
    add_rot(*skeleton, gait->bone_r_upper_arm, arm_pitch_amp * swing_l + arm_lift + relax_pitch - idle_arm,
            -arm_yaw * swing_l_90, -gait->arm_tuck - arm_roll * swing_l - arm_out);
    add_rot(*skeleton, gait->bone_l_lower_arm, elbow_amp * std::sin(arm_phase_r + elbow_phase) + relax_elbow, 0.0f, 0.0f);
    add_rot(*skeleton, gait->bone_r_lower_arm, elbow_amp * std::sin(arm_phase_l + elbow_phase) + relax_elbow, 0.0f, 0.0f);
  }

  std::vector<ecs::Transform> locals(skeleton->bones.size());
  for (size_t i = 0; i < skeleton->bones.size(); ++i) {
    locals[i] = skeleton->bones[i].local_pose;
  }
  std::vector<rkg::Mat4> world;
  compute_world_matrices(*skeleton, locals, world);

  auto safe_pos = [&](uint32_t idx) -> Vec3 {
    if (idx == UINT32_MAX || idx >= world.size()) return v3();
    return {world[idx].m[12], world[idx].m[13], world[idx].m[14]};
  };
  const Vec3 l_foot = safe_pos(gait->bone_l_foot);
  const Vec3 r_foot = safe_pos(gait->bone_r_foot);

  const Vec3 hips_e = safe_pos(gait->bone_hips);
  const Vec3 root_offset_e = {gait->pelvis_offset[0], 0.0f, gait->pelvis_offset[2]};
  const Vec3 hips_w = to_world_point(*transform, add(hips_e, root_offset_e));
  to_array(hips_w, gait->debug_hips_world);
  auto to_world_point_root = [&](const Vec3& local) {
    return to_world_point(*transform, add(local, root_offset_e));
  };
  auto to_local_point_root = [&](const Vec3& world) {
    Vec3 local = to_local_point(*transform, world);
    return sub(local, root_offset_e);
  };
  Vec3 intent_world = forward;
  float intent_mag = 0.0f;
  if (controller) {
    Vec3 in{controller->smoothed_input[0], 0.0f, controller->smoothed_input[2]};
    const float in_len = length(in);
    if (in_len > 0.0001f) {
      intent_world = mul(in, 1.0f / in_len);
      intent_mag = clampf(in_len, 0.0f, 1.0f);
    }
    if (in_len < 0.02f) {
      intent_mag = 0.0f;
    }
  }
  const bool idle = grounded && !turn_in_place && speed < (0.15f * max_speed) && intent_mag < 0.05f;
  gait->idle_time += dt;
  const float idle_target = idle ? 1.0f : 0.0f;
  const float idle_alpha = 1.0f - std::exp(-dt * 4.0f);
  gait->idle_blend += (idle_target - gait->idle_blend) * idle_alpha;
  Vec3 frame_fwd_e = from_array(gait->frame_fwd_entity);
  if (length(frame_fwd_e) < 0.0001f) frame_fwd_e = v3(0.0f, 0.0f, 1.0f);
  const float frame_rate = (intent_mag > 0.05f) ? 8.0f : 20.0f;
  const float frame_alpha = 1.0f - std::exp(-dt * frame_rate);
  Vec3 desired_frame_e = frame_fwd_e;
  Vec3 fwd_e = normalize(to_local_dir(*transform, forward));
  if (length(fwd_e) < 0.0001f) fwd_e = v3(0.0f, 0.0f, 1.0f);
  if (intent_mag > 0.05f) {
    Vec3 intent_e = normalize(to_local_dir(*transform, intent_world));
    if (length(intent_e) > 0.0001f) {
      const float forwardness = std::abs(dot(intent_e, fwd_e));
      const float intent_blend = saturate((forwardness - 0.2f) / 0.6f);
      desired_frame_e = normalize(lerp(fwd_e, intent_e, intent_blend));
    } else {
      desired_frame_e = fwd_e;
    }
  } else if (idle) {
    desired_frame_e = fwd_e;
  }
  frame_fwd_e = normalize(lerp(frame_fwd_e, desired_frame_e, frame_alpha));
  to_array(frame_fwd_e, gait->frame_fwd_entity);

  Vec3 desired_side_e = normalize(cross(v3(0.0f, 1.0f, 0.0f), frame_fwd_e));
  if (length(desired_side_e) < 0.0001f) desired_side_e = v3(1.0f, 0.0f, 0.0f);
  Vec3 prev_side_e = from_array(gait->frame_side_entity);
  if (length(prev_side_e) > 0.0001f && dot(prev_side_e, desired_side_e) < 0.0f) {
    desired_side_e = mul(desired_side_e, -1.0f);
  }
  desired_side_e = sub(desired_side_e, mul(frame_fwd_e, dot(desired_side_e, frame_fwd_e)));
  Vec3 side_e = normalize(desired_side_e);
  if (length(side_e) < 0.0001f) {
    side_e = (length(prev_side_e) > 0.0001f) ? prev_side_e : v3(1.0f, 0.0f, 0.0f);
  }
  Vec3 side_world = normalize(to_world_dir(*transform, side_e));
  if (length(side_world) < 0.0001f) side_world = right;
  to_array(side_world, gait->frame_side_world);
  Vec3 frame_fwd_w = normalize(to_world_dir(*transform, frame_fwd_e));
  if (length(frame_fwd_w) < 0.0001f) frame_fwd_w = forward;
  to_array(frame_fwd_w, gait->frame_fwd_world);

  if (gait->bone_l_thigh != UINT32_MAX && gait->bone_r_thigh != UINT32_MAX) {
    const Vec3 l_hip_now = safe_pos(gait->bone_l_thigh);
    const Vec3 r_hip_now = safe_pos(gait->bone_r_thigh);
    Vec3 right_bind = normalize(sub(r_hip_now, l_hip_now));
    if (length(right_bind) > 0.0001f && dot(right_bind, side_e) < 0.0f) {
      side_e = mul(side_e, -1.0f);
      side_world = mul(side_world, -1.0f);
      to_array(side_world, gait->frame_side_world);
    }
  }
  to_array(side_e, gait->frame_side_entity);

  const float relax_speed = clampf(1.0f - speed_norm * 0.9f, 0.0f, 1.0f);
  const float arm_rest_base = clampf(gait->arm_relax, 0.0f, 1.0f);
  const float arm_rest_weight = std::max(arm_rest_base, std::max(relax_speed, gait->idle_blend));
  const float arm_side_idle = gait->arm_out + 0.08f;
  const float arm_side_run = std::max(0.04f, gait->arm_out * 0.5f);
  const float arm_side_bias = arm_side_run + (arm_side_idle - arm_side_run) * relax_speed;
  const float arm_fwd_bias = 0.10f;
  if (arm_rest_weight > 0.01f &&
      gait->bone_l_upper_arm != UINT32_MAX && gait->bone_l_lower_arm != UINT32_MAX &&
      gait->bone_r_upper_arm != UINT32_MAX && gait->bone_r_lower_arm != UINT32_MAX) {
    std::vector<ecs::Transform> arm_locals(skeleton->bones.size());
    for (size_t i = 0; i < skeleton->bones.size(); ++i) {
      arm_locals[i] = skeleton->bones[i].local_pose;
    }
    std::vector<rkg::Mat4> arm_world;
    compute_world_matrices(*skeleton, arm_locals, arm_world);
    auto arm_pos = [&](uint32_t idx) -> Vec3 {
      if (idx == UINT32_MAX || idx >= arm_world.size()) return v3();
      return {arm_world[idx].m[12], arm_world[idx].m[13], arm_world[idx].m[14]};
    };
    auto rest_dir_for = [&](float side_sign) {
      Vec3 rest_dir = add(mul(v3(0.0f, -1.0f, 0.0f), 1.0f),
                          add(mul(frame_fwd_e, arm_fwd_bias),
                              mul(side_e, side_sign * arm_side_bias)));
      rest_dir = normalize(rest_dir);
      const float out_dot = dot(rest_dir, side_e) * side_sign;
      if (out_dot < 0.06f) {
        rest_dir = normalize(add(rest_dir, mul(side_e, side_sign * (0.06f - out_dot))));
      }
      return rest_dir;
    };
    auto apply_shoulder_rest = [&](uint32_t shoulder_idx, uint32_t upper, const Vec3& rest_dir) {
      if (shoulder_idx == UINT32_MAX || shoulder_idx >= arm_world.size()) return;
      const Vec3 shoulder_pos = arm_pos(shoulder_idx);
      const Vec3 upper_pos = arm_pos(upper);
      const float shoulder_len = std::max(length(sub(upper_pos, shoulder_pos)), 0.001f);
      const Vec3 desired_upper = add(shoulder_pos, mul(rest_dir, shoulder_len));
      const float shoulder_weight = clampf(arm_rest_weight * 0.85f, 0.0f, 1.0f);
      const Vec3 shoulder_target = lerp(upper_pos, desired_upper, shoulder_weight);
      apply_bone_aim(*skeleton, arm_world, shoulder_idx, upper, shoulder_target);
    };
    const Vec3 rest_dir_l = rest_dir_for(-1.0f);
    const Vec3 rest_dir_r = rest_dir_for(1.0f);
    if (gait->bone_l_shoulder != UINT32_MAX) {
      apply_shoulder_rest(gait->bone_l_shoulder, gait->bone_l_upper_arm, rest_dir_l);
    }
    if (gait->bone_r_shoulder != UINT32_MAX) {
      apply_shoulder_rest(gait->bone_r_shoulder, gait->bone_r_upper_arm, rest_dir_r);
    }
    for (size_t i = 0; i < skeleton->bones.size(); ++i) {
      arm_locals[i] = skeleton->bones[i].local_pose;
    }
    compute_world_matrices(*skeleton, arm_locals, arm_world);
    auto apply_upper_rest = [&](uint32_t upper, uint32_t lower, const Vec3& rest_dir) {
      const Vec3 shoulder = arm_pos(upper);
      const Vec3 elbow = arm_pos(lower);
      const float len = std::max(length(sub(elbow, shoulder)), 0.001f);
      const Vec3 desired = add(shoulder, mul(rest_dir, len));
      const float upper_weight = clampf(arm_rest_weight * 0.9f, 0.0f, 1.0f);
      const Vec3 target = lerp(elbow, desired, upper_weight);
      apply_bone_aim(*skeleton, arm_world, upper, lower, target);
    };
    apply_upper_rest(gait->bone_l_upper_arm, gait->bone_l_lower_arm, rest_dir_l);
    apply_upper_rest(gait->bone_r_upper_arm, gait->bone_r_lower_arm, rest_dir_r);
  }

  const Vec3 home_l_e = add(hips_e, from_array(gait->foot_home_l));
  const Vec3 home_r_e = add(hips_e, from_array(gait->foot_home_r));
  const Vec3 home_l_w = to_world_point_root(home_l_e);
  const Vec3 home_r_w = to_world_point_root(home_r_e);

  const float v_side = dot(planar, side_world);
  const float step_len = 0.5f * stride_len_e;
  const float step_len_small = 0.12f * leg_len_e;
  const float step_height_e = gait->step_height_scale * leg_len_e;
  const float step_time = 1.0f / std::max(cadence, 0.1f);
  const float angle_step = clampf(gait->yaw_rate * step_time * 0.35f, -0.35f, 0.35f);
  const float home_width_e = 0.5f * std::abs(dot(sub(home_r_e, home_l_e), side_e));
  const float width_ref_e = std::max(std::max(gait->hip_width, home_width_e), 0.20f * leg_len_e);
  float step_half_walk = 0.36f * width_ref_e;
  float step_half_run = 0.28f * width_ref_e;
  float step_half_local = step_half_walk + (step_half_run - step_half_walk) * speed_gait_norm;
  step_half_local = std::max(step_half_local, 0.10f * leg_len_e);
  step_half_local = std::min(step_half_local, 0.22f * leg_len_e);
  const float strafe_half = gait->lateral_step_scale * 0.20f * width_ref_e *
                            std::abs(v_side / std::max(max_speed, 0.1f));
  const float step_half_desired = step_half_local + strafe_half;
  const float side_cap = 0.20f * leg_len_e;
  const float min_side = std::min(0.90f * step_half_desired, side_cap);
  const float max_side = side_cap;
  const Vec3 home_l_rad = sub(home_l_e, hips_e);
  const Vec3 home_r_rad = sub(home_r_e, hips_e);
  const float home_rad_l = length(Vec3{home_l_rad.x, 0.0f, home_l_rad.z});
  const float home_rad_r = length(Vec3{home_r_rad.x, 0.0f, home_r_rad.z});
  const float home_rad_e = 0.5f * (home_rad_l + home_rad_r);
  const float min_rad = std::max(0.60f * home_rad_e, 0.25f * leg_len_e);

  // Ensure side axis points from left to right based on bind/home foot offsets.
  if (dot(sub(home_r_e, home_l_e), side_e) < 0.0f) {
    side_e = mul(side_e, -1.0f);
    side_world = mul(side_world, -1.0f);
    to_array(side_e, gait->frame_side_entity);
    to_array(side_world, gait->frame_side_world);
  }
  const float side_sign_l = -1.0f;
  const float side_sign_r = 1.0f;

  auto clamp_target_e = [&](const Vec3& target, float side_sign) {
    Vec3 out = target;
    float midline = dot(sub(out, hips_e), side_e);
    if (side_sign < 0.0f && midline > -min_side) {
      out = add(out, mul(side_e, (-min_side - midline)));
    } else if (side_sign > 0.0f && midline < min_side) {
      out = add(out, mul(side_e, (min_side - midline)));
    }
    midline = dot(sub(out, hips_e), side_e);
    if (midline < -max_side) {
      out = add(out, mul(side_e, (-max_side - midline)));
    } else if (midline > max_side) {
      out = add(out, mul(side_e, (max_side - midline)));
    }
    Vec3 rad = sub(out, hips_e);
    rad.y = 0.0f;
    float rad_len = length(rad);
    if (rad_len < min_rad) {
      Vec3 rad_dir = (rad_len > kEps) ? mul(rad, 1.0f / rad_len) : mul(side_e, side_sign);
      Vec3 xz = add(hips_e, mul(rad_dir, min_rad));
      out.x = xz.x;
      out.z = xz.z;
      rad = sub(out, hips_e);
      rad.y = 0.0f;
      rad_len = length(rad);
    }
    const float max_rad = std::max(min_rad, 0.95f * leg_len_e);
    if (rad_len > max_rad) {
      Vec3 rad_dir = (rad_len > kEps) ? mul(rad, 1.0f / rad_len) : mul(side_e, side_sign);
      Vec3 xz = add(hips_e, mul(rad_dir, max_rad));
      out.x = xz.x;
      out.z = xz.z;
    }
    return out;
  };

  auto recenter_home = [&](const Vec3& base_e, float side_sign) {
    Vec3 out = base_e;
    Vec3 off = sub(out, hips_e);
    const float mid = dot(off, side_e);
    const float desired_mid = side_sign * step_half_local;
    out = add(out, mul(side_e, desired_mid - mid));
    Vec3 rad = sub(out, hips_e);
    rad.y = 0.0f;
    float rad_len = length(rad);
    if (rad_len < min_rad) {
      Vec3 rad_dir = (rad_len > kEps) ? mul(rad, 1.0f / rad_len) : mul(side_e, side_sign);
      Vec3 xz = add(hips_e, mul(rad_dir, min_rad));
      out.x = xz.x;
      out.z = xz.z;
    }
    return out;
  };

  const Vec3 home_l_e_adj = recenter_home(home_l_e, side_sign_l);
  const Vec3 home_r_e_adj = recenter_home(home_r_e, side_sign_r);

  auto compute_step_target_e = [&](float side_sign, const Vec3& base_e) {
    Vec3 base = base_e;
    Vec3 desired = base;
    if (turn_in_place) {
      Vec3 off = sub(base, hips_e);
      const float off_y = off.y;
      off.y = 0.0f;
      off = rotate_y(off, angle_step);
      off.y = off_y;
      desired = add(hips_e, off);
      desired = add(desired, mul(frame_fwd_e, step_len_small));
    } else {
      desired = add(desired, mul(frame_fwd_e, step_len));
    }

    float midline = dot(sub(desired, hips_e), side_e);
    const float desired_mid = side_sign * step_half_desired;
    desired = add(desired, mul(side_e, desired_mid - midline));
    midline = dot(sub(desired, hips_e), side_e);
    if (side_sign < 0.0f && midline > -min_side) {
      desired = add(desired, mul(side_e, (-min_side - midline)));
    } else if (side_sign > 0.0f && midline < min_side) {
      desired = add(desired, mul(side_e, (min_side - midline)));
    }
    midline = dot(sub(desired, hips_e), side_e);
    if (midline < -max_side) {
      desired = add(desired, mul(side_e, (-max_side - midline)));
    } else if (midline > max_side) {
      desired = add(desired, mul(side_e, (max_side - midline)));
    }

    Vec3 rad = sub(desired, hips_e);
    rad.y = 0.0f;
    float rad_len = length(rad);
    if (rad_len < min_rad) {
      Vec3 rad_dir = (rad_len > kEps) ? mul(rad, 1.0f / rad_len) : mul(side_e, side_sign);
      Vec3 xz = add(hips_e, mul(rad_dir, min_rad));
      desired.x = xz.x;
      desired.z = xz.z;
      rad = sub(desired, hips_e);
      rad.y = 0.0f;
      rad_len = length(rad);
    }
    const float max_rad = std::max(min_rad, 0.95f * leg_len_e);
    if (rad_len > max_rad) {
      Vec3 rad_dir = (rad_len > kEps) ? mul(rad, 1.0f / rad_len) : mul(side_e, side_sign);
      Vec3 xz = add(hips_e, mul(rad_dir, max_rad));
      desired.x = xz.x;
      desired.z = xz.z;
    }

    const float ray_height = std::max(leg_len_world * 0.8f, 0.2f);
    const float foot_radius = std::max(0.02f, gait->foot_length * rig_scale * 0.25f);
    Vec3 desired_w = to_world_point_root(desired);
    RayHit hit = raycast_down(registry, add(desired_w, mul(up_axis(), ray_height)),
                              ray_height * 1.6f, foot_radius);
    if (hit.hit) {
      desired_w.y = hit.point.y;
      desired = to_local_point_root(desired_w);
    }
    return desired;
  };

  Vec3 l_lock_w = from_array(gait->left_lock_pos);
  Vec3 r_lock_w = from_array(gait->right_lock_pos);
  const bool left_swing_run = left_swing && !idle;
  const bool right_swing_run = right_swing && !idle;
  const bool left_stance_run = idle ? true : left_stance;
  const bool right_stance_run = idle ? true : right_stance;
  const bool left_was_swing = gait->left_step_active;
  const bool right_was_swing = gait->right_step_active;
  const bool left_just_landed = grounded && left_was_swing && !left_swing_run;
  const bool right_just_landed = grounded && right_was_swing && !right_swing_run;

  if (!grounded) {
    gait->left_step_active = false;
    gait->right_step_active = false;
    gait->left_locked = false;
    gait->right_locked = false;
  } else {
    if (idle) {
      Vec3 home_l_w = to_world_point_root(home_l_e_adj);
      Vec3 home_r_w = to_world_point_root(home_r_e_adj);
      const float idle_alpha = 1.0f - std::exp(-dt * 6.0f);
      if (!gait->left_locked) {
        l_lock_w = home_l_w;
      } else {
        l_lock_w = lerp(l_lock_w, home_l_w, idle_alpha);
      }
      if (!gait->right_locked) {
        r_lock_w = home_r_w;
      } else {
        r_lock_w = lerp(r_lock_w, home_r_w, idle_alpha);
      }
      to_array(l_lock_w, gait->left_lock_pos);
      to_array(r_lock_w, gait->right_lock_pos);
      Vec3 l_lock_e = to_local_point_root(l_lock_w);
      Vec3 r_lock_e = to_local_point_root(r_lock_w);
      to_array(l_lock_e, gait->left_step_pos);
      to_array(r_lock_e, gait->right_step_pos);
      to_array(l_lock_e, gait->left_swing_start_pos);
      to_array(r_lock_e, gait->right_swing_start_pos);
      gait->left_step_active = false;
      gait->right_step_active = false;
      gait->left_locked = true;
      gait->right_locked = true;
    }
    if (left_swing_run && !left_was_swing) {
      Vec3 start_e = gait->left_locked ? to_local_point_root(l_lock_w) : l_foot;
      to_array(start_e, gait->left_swing_start_pos);
      Vec3 end_e = compute_step_target_e(side_sign_l, home_l_e_adj);
      to_array(end_e, gait->left_step_pos);
      gait->left_step_active = true;
      gait->left_locked = false;
    }
    if (right_swing_run && !right_was_swing) {
      Vec3 start_e = gait->right_locked ? to_local_point_root(r_lock_w) : r_foot;
      to_array(start_e, gait->right_swing_start_pos);
      Vec3 end_e = compute_step_target_e(side_sign_r, home_r_e_adj);
      to_array(end_e, gait->right_step_pos);
      gait->right_step_active = true;
      gait->right_locked = false;
    }
    if (!left_swing_run && left_was_swing) {
      gait->left_step_active = false;
      Vec3 end_e = from_array(gait->left_step_pos);
      end_e = clamp_target_e(end_e, side_sign_l);
      l_lock_w = to_world_point_root(end_e);
      to_array(l_lock_w, gait->left_lock_pos);
      gait->left_locked = true;
    }
    if (!right_swing_run && right_was_swing) {
      gait->right_step_active = false;
      Vec3 end_e = from_array(gait->right_step_pos);
      end_e = clamp_target_e(end_e, side_sign_r);
      r_lock_w = to_world_point_root(end_e);
      to_array(r_lock_w, gait->right_lock_pos);
      gait->right_locked = true;
    }
    if (left_stance_run && !gait->left_locked && !gait->left_step_active) {
      l_lock_w = to_world_point_root(l_foot);
      Vec3 clamp_e = clamp_target_e(to_local_point_root(l_lock_w), side_sign_l);
      l_lock_w = to_world_point_root(clamp_e);
      to_array(l_lock_w, gait->left_lock_pos);
      gait->left_locked = true;
    }
    if (right_stance_run && !gait->right_locked && !gait->right_step_active) {
      r_lock_w = to_world_point_root(r_foot);
      Vec3 clamp_e = clamp_target_e(to_local_point_root(r_lock_w), side_sign_r);
      r_lock_w = to_world_point_root(clamp_e);
      to_array(r_lock_w, gait->right_lock_pos);
      gait->right_locked = true;
    }
  }

  auto swing_foot = [&](const Vec3& start, const Vec3& end, float t) {
    const float t_ease = t * t * (3.0f - 2.0f * t);
    Vec3 out = lerp(start, end, t_ease);
    float arc = std::sin(kPi * t);
    arc = std::pow(std::max(arc, 0.0f), 0.75f);
    out.y += step_height_e * arc;
    out.y += step_height_e * 0.15f * smoothstep01((t - 0.7f) / 0.3f);
    return out;
  };

  Vec3 l_target_e = l_foot;
  Vec3 r_target_e = r_foot;
  Vec3 l_target_w = to_world_point_root(l_foot);
  Vec3 r_target_w = to_world_point_root(r_foot);
  if (grounded) {
    if (left_stance_run) {
      l_target_w = l_lock_w;
      l_target_e = to_local_point_root(l_target_w);
    } else if (left_swing_run) {
      const Vec3 start_e = from_array(gait->left_swing_start_pos);
      const Vec3 end_e = from_array(gait->left_step_pos);
      l_target_e = swing_foot(start_e, end_e, left_swing_u);
    }
    if (right_stance_run) {
      r_target_w = r_lock_w;
      r_target_e = to_local_point_root(r_target_w);
    } else if (right_swing_run) {
      const Vec3 start_e = from_array(gait->right_swing_start_pos);
      const Vec3 end_e = from_array(gait->right_step_pos);
      r_target_e = swing_foot(start_e, end_e, right_swing_u);
    }
  }
  if (left_stance_run) {
    l_target_e = to_local_point_root(l_target_w);
  }
  if (right_stance_run) {
    r_target_e = to_local_point_root(r_target_w);
  }
  if (left_swing_run) {
    l_target_w = to_world_point_root(l_target_e);
  }
  if (right_swing_run) {
    r_target_w = to_world_point_root(r_target_e);
  }

  const bool left_locked_now =
      grounded && gait->enable_foot_lock && left_stance_run && gait->left_locked;
  const bool right_locked_now =
      grounded && gait->enable_foot_lock && right_stance_run && gait->right_locked;

  if (left_locked_now) {
    l_target_w = l_lock_w;
    l_target_e = to_local_point_root(l_target_w);
  }
  if (right_locked_now) {
    r_target_w = r_lock_w;
    r_target_e = to_local_point_root(r_target_w);
  }

  const float target_alpha = 1.0f - std::exp(-dt * 12.0f);
  Vec3 l_smooth = from_array(gait->smooth_left_target);
  Vec3 r_smooth = from_array(gait->smooth_right_target);
  if (length(l_smooth) < 0.0001f) l_smooth = l_target_e;
  if (length(r_smooth) < 0.0001f) r_smooth = r_target_e;
  if (left_swing_run || left_locked_now) {
    l_smooth = l_target_e;
  } else {
    l_smooth = lerp(l_smooth, l_target_e, target_alpha);
  }
  if (right_swing_run || right_locked_now) {
    r_smooth = r_target_e;
  } else {
    r_smooth = lerp(r_smooth, r_target_e, target_alpha);
  }
  to_array(l_smooth, gait->smooth_left_target);
  to_array(r_smooth, gait->smooth_right_target);
  if (!left_locked_now) {
    l_target_e = clamp_target_e(l_smooth, side_sign_l);
  } else {
    l_target_e = l_smooth;
  }
  if (!right_locked_now) {
    r_target_e = clamp_target_e(r_smooth, side_sign_r);
  } else {
    r_target_e = r_smooth;
  }
  l_target_w = to_world_point_root(l_target_e);
  r_target_w = to_world_point_root(r_target_e);
  if (left_locked_now) {
    l_target_w = l_lock_w;
  }
  if (right_locked_now) {
    r_target_w = r_lock_w;
  }

  to_array(l_target_e, gait->debug_left_target);
  to_array(r_target_e, gait->debug_right_target);
  to_array(frame_fwd_w, gait->debug_forward);
  to_array(side_world, gait->debug_right);
  gait->debug_target_lat_l = dot(sub(l_target_e, hips_e), side_e);
  gait->debug_target_lat_r = dot(sub(r_target_e, hips_e), side_e);

  float debug_lock_off_len = 0.0f;
  float debug_lock_max = 0.0f;
  if (gait->enable_ik) {
    Vec3 lock_offset_e = v3();
    int lock_count = 0;
    if (left_locked_now) {
      const Vec3 foot_w = to_world_point_root(l_foot);
      const Vec3 delta_w = sub(l_lock_w, foot_w);
      const Vec3 delta_e = to_local_offset(*transform, delta_w);
      lock_offset_e = add(lock_offset_e, delta_e);
      ++lock_count;
    }
    if (right_locked_now) {
      const Vec3 foot_w = to_world_point_root(r_foot);
      const Vec3 delta_w = sub(r_lock_w, foot_w);
      const Vec3 delta_e = to_local_offset(*transform, delta_w);
      lock_offset_e = add(lock_offset_e, delta_e);
      ++lock_count;
    }
    debug_lock_max = leg_len_e * 0.5f;
    if (lock_count > 0) {
      lock_offset_e = mul(lock_offset_e, 1.0f / static_cast<float>(lock_count));
      lock_offset_e.y = 0.0f;
      debug_lock_max = leg_len_e * (0.45f + 0.55f * speed_gait_norm);
      const float off_len = length(lock_offset_e);
      debug_lock_off_len = off_len;
      if (off_len > debug_lock_max && off_len > kEps) {
        lock_offset_e = mul(lock_offset_e, debug_lock_max / off_len);
      }
    } else {
      lock_offset_e = v3();
    }

    const float ik_weight_l = grounded ? (left_stance_run ? 1.0f : (left_swing_run ? 1.0f : 0.0f)) : 0.0f;
    const float ik_weight_r = grounded ? (right_stance_run ? 1.0f : (right_swing_run ? 1.0f : 0.0f)) : 0.0f;
    const Vec3 target_l = lerp(l_foot, l_target_e, ik_weight_l);
    const Vec3 target_r = lerp(r_foot, r_target_e, ik_weight_r);

    const float max_pelvis_drop = leg_len_e * 0.12f;
    const float desired_drop = std::min(0.0f, std::min(target_l.y - l_foot.y, target_r.y - r_foot.y));
    const float clamped_drop = clampf(desired_drop, -max_pelvis_drop, 0.0f);
    const float pelvis_alpha = 1.0f - std::exp(-dt * gait->ik_blend_speed);
    gait->pelvis_offset[0] += (lock_offset_e.x - gait->pelvis_offset[0]) * pelvis_alpha;
    gait->pelvis_offset[1] += (clamped_drop - gait->pelvis_offset[1]) * pelvis_alpha;
    gait->pelvis_offset[2] += (lock_offset_e.z - gait->pelvis_offset[2]) * pelvis_alpha;
    const uint32_t root_idx = (gait->bone_root != UINT32_MAX) ? gait->bone_root : gait->bone_hips;
    add_pos(*skeleton, root_idx, gait->pelvis_offset[0], 0.0f, gait->pelvis_offset[2]);
    add_pos(*skeleton, gait->bone_hips, 0.0f, gait->pelvis_offset[1], 0.0f);

    for (size_t i = 0; i < skeleton->bones.size(); ++i) {
      locals[i] = skeleton->bones[i].local_pose;
    }
    compute_world_matrices(*skeleton, locals, world);
    const Vec3 l_hip2 = safe_pos(gait->bone_l_thigh);
    const Vec3 l_knee2 = safe_pos(gait->bone_l_calf);
    const Vec3 l_foot2 = safe_pos(gait->bone_l_foot);
    const Vec3 r_hip2 = safe_pos(gait->bone_r_thigh);
    const Vec3 r_knee2 = safe_pos(gait->bone_r_calf);
    const Vec3 r_foot2 = safe_pos(gait->bone_r_foot);
    const Vec3 hips_e2 = safe_pos(gait->bone_hips);

    const float width_ref_e = std::max(gait->hip_width, 0.25f * gait->leg_length);
    const float pole_out_e = 0.45f * width_ref_e;
    const Vec3 up_e = v3(0.0f, 1.0f, 0.0f);
    auto compute_pole_dir = [&](const Vec3& hip, const Vec3& foot, float side_sign,
                                const Vec3& fallback_dir) -> Vec3 {
      Vec3 leg_dir = normalize(sub(foot, hip));
      if (length(leg_dir) < 0.0001f) leg_dir = v3(0.0f, -1.0f, 0.0f);
      Vec3 fwd = frame_fwd_e;
      if (length(fwd) < 0.0001f) fwd = v3(0.0f, 0.0f, 1.0f);
      Vec3 fwd_plane = sub(fwd, mul(leg_dir, dot(fwd, leg_dir)));
      if (length(fwd_plane) < 0.0001f) {
        fwd_plane = sub(up_e, mul(leg_dir, dot(up_e, leg_dir)));
      }
      fwd_plane = normalize(fwd_plane);
      Vec3 out = mul(side_e, side_sign);
      Vec3 out_plane = sub(out, mul(leg_dir, dot(out, leg_dir)));
      if (length(out_plane) < 0.0001f) out_plane = fwd_plane;
      out_plane = normalize(out_plane);
      const float bias = clampf(gait->knee_plane_bias, 0.0f, 1.0f);
      Vec3 desired = normalize(lerp(out_plane, fwd_plane, bias));
      if (length(desired) < 0.0001f) desired = fallback_dir;
      return desired;
    };
    Vec3 l_pole_prev = normalize(from_array(gait->knee_plane_l));
    Vec3 r_pole_prev = normalize(from_array(gait->knee_plane_r));
    if (length(l_pole_prev) < 0.0001f) l_pole_prev = v3(-1.0f, 0.0f, 0.0f);
    if (length(r_pole_prev) < 0.0001f) r_pole_prev = v3(1.0f, 0.0f, 0.0f);
    const Vec3 l_pole_target = compute_pole_dir(l_hip2, l_foot2, -1.0f, l_pole_prev);
    const Vec3 r_pole_target = compute_pole_dir(r_hip2, r_foot2, 1.0f, r_pole_prev);
    if (dot(l_pole_prev, l_pole_target) < 0.0f) l_pole_prev = mul(l_pole_prev, -1.0f);
    if (dot(r_pole_prev, r_pole_target) < 0.0f) r_pole_prev = mul(r_pole_prev, -1.0f);
    const float pole_alpha = 1.0f - std::exp(-dt * 12.0f);
    const Vec3 l_pole_dir = normalize(lerp(l_pole_prev, l_pole_target, pole_alpha));
    const Vec3 r_pole_dir = normalize(lerp(r_pole_prev, r_pole_target, pole_alpha));
    to_array(l_pole_dir, gait->knee_plane_l);
    to_array(r_pole_dir, gait->knee_plane_r);
    const Vec3 l_pole_e = add(l_hip2, mul(l_pole_dir, pole_out_e));
    const Vec3 r_pole_e = add(r_hip2, mul(r_pole_dir, pole_out_e));
    Vec3 l_prev_y = normalize(from_array(gait->knee_prev_y_l));
    Vec3 r_prev_y = normalize(from_array(gait->knee_prev_y_r));
    if (length(l_prev_y) < 0.0001f) l_prev_y = v3(0.0f, 0.0f, 1.0f);
    if (length(r_prev_y) < 0.0001f) r_prev_y = v3(0.0f, 0.0f, 1.0f);
    float cont_l = 1.0f;
    float cont_r = 1.0f;

    const Vec3 l_knee_target = solve_two_bone_ik_pole(l_hip2, l_knee2, l_foot2, target_l, l_pole_e,
                                                      l_prev_y, &cont_l);
    const Vec3 r_knee_target = solve_two_bone_ik_pole(r_hip2, r_knee2, r_foot2, target_r, r_pole_e,
                                                      r_prev_y, &cont_r);
    to_array(l_prev_y, gait->knee_prev_y_l);
    to_array(r_prev_y, gait->knee_prev_y_r);
    gait->debug_continuity_l = cont_l;
    gait->debug_continuity_r = cont_r;

    const Vec3 l_pole_dir_e = normalize(sub(l_pole_e, l_hip2));
    const Vec3 r_pole_dir_e = normalize(sub(r_pole_e, r_hip2));
    to_array(l_pole_dir_e, gait->debug_left_pole);
    to_array(r_pole_dir_e, gait->debug_right_pole);

    gait->debug_knee_lat_l = dot(sub(l_knee_target, hips_e2), side_e);
    gait->debug_knee_lat_r = dot(sub(r_knee_target, hips_e2), side_e);
    apply_bone_aim(*skeleton, world, gait->bone_l_thigh, gait->bone_l_calf, l_knee_target);
    apply_bone_aim(*skeleton, world, gait->bone_r_thigh, gait->bone_r_calf, r_knee_target);

    for (size_t i = 0; i < skeleton->bones.size(); ++i) {
      locals[i] = skeleton->bones[i].local_pose;
    }
    compute_world_matrices(*skeleton, locals, world);
    apply_bone_aim(*skeleton, world, gait->bone_l_calf, gait->bone_l_foot, target_l);
    apply_bone_aim(*skeleton, world, gait->bone_r_calf, gait->bone_r_foot, target_r);

    if (left_just_landed || right_just_landed) {
      for (size_t i = 0; i < skeleton->bones.size(); ++i) {
        locals[i] = skeleton->bones[i].local_pose;
      }
      compute_world_matrices(*skeleton, locals, world);
      if (left_just_landed) {
        const Vec3 foot_e = safe_pos(gait->bone_l_foot);
        const Vec3 foot_w = to_world_point(*transform, foot_e);
        to_array(foot_w, gait->left_lock_pos);
        gait->left_locked = true;
      }
      if (right_just_landed) {
        const Vec3 foot_e = safe_pos(gait->bone_r_foot);
        const Vec3 foot_w = to_world_point(*transform, foot_e);
        to_array(foot_w, gait->right_lock_pos);
        gait->right_locked = true;
      }
    }
  }

  if (gait->enable_self_collision) {
    for (size_t i = 0; i < skeleton->bones.size(); ++i) {
      locals[i] = skeleton->bones[i].local_pose;
    }
    compute_world_matrices(*skeleton, locals, world);
    solve_self_collision(*gait, *skeleton, registry, entity, *transform, &g_external_caps,
                         locals, world, leg_len_e, dt);
  }

  if (rkg::movement_log::enabled()) {
    std::vector<ecs::Transform> log_locals(skeleton->bones.size());
    for (size_t i = 0; i < skeleton->bones.size(); ++i) {
      log_locals[i] = skeleton->bones[i].local_pose;
    }
    std::vector<rkg::Mat4> log_world;
    compute_world_matrices(*skeleton, log_locals, log_world);

    {
      static std::unordered_map<ecs::Entity, std::vector<Vec3>> last_world_pos;
      static std::unordered_map<ecs::Entity, std::vector<Vec3>> last_local_rot;
      auto& prev_pos = last_world_pos[entity];
      auto& prev_rot = last_local_rot[entity];
      if (prev_pos.size() != log_world.size()) {
        prev_pos.assign(log_world.size(), v3(1e9f, 1e9f, 1e9f));
        prev_rot.assign(log_world.size(), v3(1e9f, 1e9f, 1e9f));
      }
      const float pos_eps_sq = 1e-6f;
      const float rot_eps_sq = 1e-6f;
      for (size_t i = 0; i < log_world.size(); ++i) {
        const Vec3 pos = {log_world[i].m[12], log_world[i].m[13], log_world[i].m[14]};
        const Vec3 rot = {log_locals[i].rotation[0], log_locals[i].rotation[1], log_locals[i].rotation[2]};
        const Vec3 dpos = sub(pos, prev_pos[i]);
        const Vec3 drot = sub(rot, prev_rot[i]);
        if (length_sq(dpos) > pos_eps_sq || length_sq(drot) > rot_eps_sq) {
          std::ostringstream bone_line;
          bone_line.setf(std::ios::fixed);
          bone_line.precision(4);
          const std::string& name = skeleton->bones[i].name;
          bone_line << "bone entity=" << entity
                    << " idx=" << i
                    << " name=" << (name.empty() ? std::string("unnamed") : name)
                    << " pos=(" << pos.x << "," << pos.y << "," << pos.z << ")"
                    << " local=(" << log_locals[i].position[0] << ","
                    << log_locals[i].position[1] << "," << log_locals[i].position[2] << ")"
                    << " rot=(" << rot.x << "," << rot.y << "," << rot.z << ")";
          rkg::movement_log::write(bone_line.str());
          prev_pos[i] = pos;
          prev_rot[i] = rot;
        }
      }
    }

    static float gait_log_accum = 0.0f;
    gait_log_accum += dt;
    if (gait_log_accum >= 0.1f) {
      gait_log_accum = 0.0f;
      auto safe_world_pos = [&](uint32_t idx) -> Vec3 {
        if (idx == UINT32_MAX || idx >= log_world.size()) return v3();
        return {log_world[idx].m[12], log_world[idx].m[13], log_world[idx].m[14]};
      };
      const Vec3 l_foot_e_log = safe_world_pos(gait->bone_l_foot);
      const Vec3 r_foot_e_log = safe_world_pos(gait->bone_r_foot);
      const Vec3 l_foot_w_log = to_world_point(*transform, l_foot_e_log);
      const Vec3 r_foot_w_log = to_world_point(*transform, r_foot_e_log);
      const Vec3 l_lock_w_log = from_array(gait->left_lock_pos);
      const Vec3 r_lock_w_log = from_array(gait->right_lock_pos);
      const Vec3 l_err = sub(l_lock_w_log, l_foot_w_log);
      const Vec3 r_err = sub(r_lock_w_log, r_foot_w_log);
      const float l_err_len = length(l_err);
      const float r_err_len = length(r_err);
      const Vec3 entity_pos = {transform->position[0], transform->position[1], transform->position[2]};
      const Vec3 hips_e_log = safe_pos(gait->bone_hips);
      Vec3 rad_l = sub(l_target_e, hips_e_log);
      Vec3 rad_r = sub(r_target_e, hips_e_log);
      rad_l.y = 0.0f;
      rad_r.y = 0.0f;
      const float target_rad_l = length(rad_l);
      const float target_rad_r = length(rad_r);
      const Vec3 l_pole_e = from_array(gait->debug_left_pole);
      const Vec3 r_pole_e = from_array(gait->debug_right_pole);
      std::ostringstream line;
      line.setf(std::ios::fixed);
      line.precision(3);
      line << "gait entity=" << entity
           << " yaw=" << gait->yaw
           << " yaw_rate=" << gait->yaw_rate
           << " phase=" << gait->phase
           << " cycle=" << cycle
           << " stance_frac=" << stance_fraction
           << " u=(" << u_l << "," << u_r << ")"
           << " stance_u=(" << left_stance_u << "," << right_stance_u << ")"
           << " swing_u=(" << left_swing_u << "," << right_swing_u << ")"
           << " step_active=(" << gait->left_step_active << "," << gait->right_step_active << ")"
           << " lock_now=(" << left_locked_now << "," << right_locked_now << ")"
           << " speed=" << speed
           << " cadence=" << cadence
           << " stride_e=" << stride_len_e
           << " step_time=" << step_time
           << " grounded=" << grounded
          << " left_stance=" << left_stance_run
          << " right_stance=" << right_stance_run
          << " left_swing=" << left_swing_run
          << " right_swing=" << right_swing_run
           << " l_target_e=(" << l_target_e.x << "," << l_target_e.y << "," << l_target_e.z << ")"
           << " r_target_e=(" << r_target_e.x << "," << r_target_e.y << "," << r_target_e.z << ")"
           << " l_pole_e=(" << l_pole_e.x << "," << l_pole_e.y << "," << l_pole_e.z << ")"
           << " r_pole_e=(" << r_pole_e.x << "," << r_pole_e.y << "," << r_pole_e.z << ")"
           << " knee_lat=(" << gait->debug_knee_lat_l << "," << gait->debug_knee_lat_r << ")"
           << " target_lat=(" << gait->debug_target_lat_l << "," << gait->debug_target_lat_r << ")"
           << " target_rad=(" << target_rad_l << "," << target_rad_r << ")"
           << " min_side=" << min_side
           << " min_rad=" << min_rad
           << " cont=(" << gait->debug_continuity_l << "," << gait->debug_continuity_r << ")"
           << " stance=(" << left_stance_run << "," << right_stance_run << ")"
           << " swing=(" << left_swing_run << "," << right_swing_run << ")"
           << " max=" << max_speed
           << " lockL=" << gait->left_locked
           << " lockR=" << gait->right_locked
           << " l_foot_w=(" << l_foot_w_log.x << "," << l_foot_w_log.y << "," << l_foot_w_log.z << ")"
           << " r_foot_w=(" << r_foot_w_log.x << "," << r_foot_w_log.y << "," << r_foot_w_log.z << ")"
           << " l_lock_w=(" << l_lock_w_log.x << "," << l_lock_w_log.y << "," << l_lock_w_log.z << ")"
           << " r_lock_w=(" << r_lock_w_log.x << "," << r_lock_w_log.y << "," << r_lock_w_log.z << ")"
           << " pos=(" << entity_pos.x << "," << entity_pos.y << "," << entity_pos.z << ")"
           << " l_tgt_w=(" << l_target_w.x << "," << l_target_w.y << "," << l_target_w.z << ")"
           << " r_tgt_w=(" << r_target_w.x << "," << r_target_w.y << "," << r_target_w.z << ")"
           << " l_err=(" << l_err.x << "," << l_err.y << "," << l_err.z << ")"
           << " r_err=(" << r_err.x << "," << r_err.y << "," << r_err.z << ")"
           << " l_err_len=" << l_err_len
           << " r_err_len=" << r_err_len
           << " lock_off=" << debug_lock_off_len
           << " lock_max=" << debug_lock_max
           << " root_off=(" << gait->pelvis_offset[0] << "," << gait->pelvis_offset[2] << ")";
      rkg::movement_log::write(line.str());
    }
  }
}

void update_procedural_gaits(ecs::Registry& registry, float dt) {
  static DebugLines lines;
  lines.clear();
  bool any_debug = false;
  build_external_capsules(registry, g_external_caps);
  for (auto& kv : registry.procedural_gaits()) {
    auto& gait = kv.second;
    if (!gait.enabled) continue;
    update_procedural_gait(registry, kv.first, dt);
    if (!gait.debug_draw) continue;
    if (!any_debug) {
      lines.append_from(rkg::get_vulkan_viewport_line_list());
      any_debug = true;
    }
    auto* skeleton = registry.get_skeleton(kv.first);
    if (!skeleton) continue;
    auto* transform = registry.get_transform(kv.first);
    if (!transform) continue;
    std::vector<ecs::Transform> locals(skeleton->bones.size());
    for (size_t i = 0; i < skeleton->bones.size(); ++i) {
      locals[i] = skeleton->bones[i].local_pose;
    }
    std::vector<rkg::Mat4> world;
    compute_world_matrices(*skeleton, locals, world);
    auto safe_pos = [&](uint32_t idx) -> Vec3 {
      if (idx == UINT32_MAX || idx >= world.size()) return v3();
      return {world[idx].m[12], world[idx].m[13], world[idx].m[14]};
    };
    Vec3 l_foot = safe_pos(gait.bone_l_foot);
    Vec3 r_foot = safe_pos(gait.bone_r_foot);
    Vec3 l_foot_w = to_world_point(*transform, l_foot);
    Vec3 r_foot_w = to_world_point(*transform, r_foot);
    Vec3 l_target_e = from_array(gait.debug_left_target);
    Vec3 r_target_e = from_array(gait.debug_right_target);
    const Vec3 root_offset_e = {gait.pelvis_offset[0], 0.0f, gait.pelvis_offset[2]};
    auto to_world_point_root = [&](const Vec3& local) {
      return to_world_point(*transform, add(local, root_offset_e));
    };
    Vec3 l_target_w = to_world_point_root(l_target_e);
    Vec3 r_target_w = to_world_point_root(r_target_e);
    lines.add_line(l_foot_w, l_target_w, v3(0.2f, 0.8f, 0.2f));
    lines.add_line(r_foot_w, r_target_w, v3(0.2f, 0.8f, 0.2f));
    auto add_cross = [&](const Vec3& p, float size, const Vec3& c) {
      lines.add_line(add(p, v3(-size, 0.0f, 0.0f)), add(p, v3(size, 0.0f, 0.0f)), c);
      lines.add_line(add(p, v3(0.0f, 0.0f, -size)), add(p, v3(0.0f, 0.0f, size)), c);
    };
    if (gait.left_locked) {
      const Vec3 l_lock_w = from_array(gait.left_lock_pos);
      lines.add_line(l_foot_w, l_lock_w, v3(0.9f, 0.2f, 0.2f));
      add_cross(l_lock_w, 0.05f, v3(0.9f, 0.2f, 0.2f));
    }
    if (gait.right_locked) {
      const Vec3 r_lock_w = from_array(gait.right_lock_pos);
      lines.add_line(r_foot_w, r_lock_w, v3(0.2f, 0.4f, 0.9f));
      add_cross(r_lock_w, 0.05f, v3(0.2f, 0.4f, 0.9f));
    }
    Vec3 l_hip = safe_pos(gait.bone_l_thigh);
    Vec3 l_knee = safe_pos(gait.bone_l_calf);
    Vec3 r_hip = safe_pos(gait.bone_r_thigh);
    Vec3 r_knee = safe_pos(gait.bone_r_calf);
    Vec3 l_hip_w = to_world_point(*transform, l_hip);
    Vec3 l_knee_w = to_world_point(*transform, l_knee);
    Vec3 r_hip_w = to_world_point(*transform, r_hip);
    Vec3 r_knee_w = to_world_point(*transform, r_knee);
    lines.add_line(l_hip_w, l_knee_w, v3(0.8f, 0.6f, 0.1f));
    lines.add_line(l_knee_w, l_foot_w, v3(0.8f, 0.6f, 0.1f));
    lines.add_line(r_hip_w, r_knee_w, v3(0.8f, 0.6f, 0.1f));
    lines.add_line(r_knee_w, r_foot_w, v3(0.8f, 0.6f, 0.1f));
    Vec3 l_pole_e = from_array(gait.debug_left_pole);
    Vec3 r_pole_e = from_array(gait.debug_right_pole);
    Vec3 l_pole_w = to_world_dir(*transform, l_pole_e);
    Vec3 r_pole_w = to_world_dir(*transform, r_pole_e);
    lines.add_line(l_hip_w, add(l_hip_w, mul(l_pole_w, 0.2f)), v3(0.2f, 0.6f, 1.0f));
    lines.add_line(r_hip_w, add(r_hip_w, mul(r_pole_w, 0.2f)), v3(0.2f, 0.6f, 1.0f));
    Vec3 hips_w = from_array(gait.debug_hips_world);
    Vec3 home_l_off = from_array(gait.foot_home_l);
    Vec3 home_r_off = from_array(gait.foot_home_r);
    Vec3 fwd = from_array(gait.debug_forward);
    Vec3 right = from_array(gait.debug_right);
    Vec3 hips_e = safe_pos(gait.bone_hips);
    Vec3 home_l_e = add(hips_e, home_l_off);
    Vec3 home_r_e = add(hips_e, home_r_off);
    Vec3 home_l_w = to_world_point(*transform, home_l_e);
    Vec3 home_r_w = to_world_point(*transform, home_r_e);
    lines.add_line(hips_w, home_l_w, v3(0.9f, 0.5f, 0.1f));
    lines.add_line(hips_w, home_r_w, v3(0.9f, 0.5f, 0.1f));
    lines.add_line(hips_w, l_target_w, v3(0.4f, 0.9f, 0.4f));
    lines.add_line(hips_w, r_target_w, v3(0.4f, 0.9f, 0.4f));
    lines.add_line(hips_w, add(hips_w, mul(fwd, 0.3f)), v3(0.2f, 0.9f, 0.9f));
    lines.add_line(hips_w, add(hips_w, mul(right, 0.3f)), v3(0.9f, 0.2f, 0.2f));
  }

  if (any_debug && lines.count > 0) {
    rkg::set_vulkan_viewport_line_list(lines.pos.data(), lines.color.data(), lines.count);
  }
}

ecs::Transform debug_solve_two_bone_ik(const ecs::Transform& hip,
                                       const ecs::Transform& knee,
                                       const ecs::Transform& ankle,
                                       const ecs::Transform& target,
                                       const ecs::Transform& plane_hint) {
  const Vec3 hip_p = from_array(hip.position);
  const Vec3 knee_p = from_array(knee.position);
  const Vec3 ankle_p = from_array(ankle.position);
  const Vec3 tgt = from_array(target.position);
  const Vec3 pole_point = from_array(plane_hint.position);
  Vec3 prev_y = v3(0.0f, 0.0f, 1.0f);
  const Vec3 knee_out = solve_two_bone_ik_pole(hip_p, knee_p, ankle_p, tgt, pole_point, prev_y, nullptr);
  ecs::Transform out{};
  to_array(knee_out, out.position);
  return out;
}

bool debug_update_foot_lock(bool& locked,
                            float lock_pos[3],
                            const float foot_pos[3],
                            bool grounded,
                            float swing_phase,
                            float lock_in,
                            float lock_out,
                            float dt) {
  Vec3 lock = from_array(lock_pos);
  const Vec3 foot = from_array(foot_pos);
  const bool out = update_foot_lock_internal(locked, lock, foot, grounded,
                                             swing_phase, lock_in, lock_out, dt);
  to_array(lock, lock_pos);
  return out;
}

} // namespace rkg::locomotion
