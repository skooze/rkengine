#include "rkg/locomotion.h"

#include "rkg/log.h"
#include "rkg/math.h"
#include "rkg/renderer_hooks.h"

#include <algorithm>
#include <cctype>
#include <cmath>
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

static Vec3 project_on_plane(const Vec3& v, const Vec3& n) {
  return sub(v, mul(n, dot(v, n)));
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
  const float r01 = m.m[4];
  const float r02 = m.m[8];
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

static rkg::Vec3 mat4_get_translation(const rkg::Mat4& m) {
  return {m.m[12], m.m[13], m.m[14]};
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
                           float max_dist) {
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
      float plane_d = collider.distance;
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
        best.point = add(origin, mul(dir, t));
      }
    } else if (collider.type == ecs::ColliderType::AABB) {
      Vec3 center = v3(collider.center[0], collider.center[1], collider.center[2]);
      if (const auto* t = registry.get_transform(kv.first)) {
        center = add(center, from_array(t->position));
      }
      Vec3 half_extents = v3(collider.half_extents[0], collider.half_extents[1], collider.half_extents[2]);
      RayHit hit{};
      if (raycast_aabb(origin, dir, center, half_extents, best.t, hit)) {
        if (hit.t <= best.t) {
          best = hit;
        }
      }
    }
  }
  return best;
}

static Vec3 solve_two_bone_ik(const Vec3& hip,
                              const Vec3& knee,
                              const Vec3& ankle,
                              const Vec3& target,
                              const Vec3& plane_normal) {
  const float l1 = length(sub(knee, hip));
  const float l2 = length(sub(ankle, knee));
  Vec3 dir = sub(target, hip);
  float d = length(dir);
  if (d < 0.0001f) d = 0.0001f;
  const float min_d = std::max(0.001f, std::fabs(l1 - l2) + 0.001f);
  const float max_d = std::max(0.001f, l1 + l2 - 0.001f);
  d = clampf(d, min_d, max_d);
  dir = mul(dir, 1.0f / d);

  const float x = (l1 * l1 - l2 * l2 + d * d) / (2.0f * d);
  float h_sq = l1 * l1 - x * x;
  if (h_sq < 0.0f) h_sq = 0.0f;
  const float h = std::sqrt(h_sq);

  Vec3 axis = cross(plane_normal, dir);
  axis = normalize(axis);
  if (length(axis) < 0.0001f) {
    axis = normalize(cross(v3(0.0f, 1.0f, 0.0f), dir));
  }
  const float side = dot(sub(knee, hip), axis) >= 0.0f ? 1.0f : -1.0f;
  axis = mul(axis, side);

  return add(hip, add(mul(dir, x), mul(axis, h)));
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
  skel.bones[bone_idx].local_pose.rotation[0] = euler.x;
  skel.bones[bone_idx].local_pose.rotation[1] = euler.y;
  skel.bones[bone_idx].local_pose.rotation[2] = euler.z;
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
    locals[i] = skel.bones[i].bind_local;
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

struct DebugLines {
  std::vector<float> pos;
  std::vector<float> color;
  uint32_t count = 0;
  void clear() {
    pos.clear();
    color.clear();
    count = 0;
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
  gait->speed_smoothed += (speed - gait->speed_smoothed) * smooth_alpha;
  speed = gait->speed_smoothed;

  const bool grounded = controller ? controller->grounded : false;
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
  gait->yaw = transform->rotation[1];
  gait->yaw_rate = wrap_pi(gait->yaw - gait->last_yaw) / std::max(dt, 0.0001f);
  gait->last_yaw = gait->yaw;

  if (grounded && !gait->was_grounded) {
    gait->landing_timer = 0.18f;
  }
  gait->was_grounded = grounded;
  if (gait->landing_timer > 0.0f) {
    gait->landing_timer = std::max(0.0f, gait->landing_timer - dt);
  }

  const bool turn_in_place = gait->enable_turn_in_place &&
                             grounded &&
                             speed < gait->turn_in_place_speed &&
                             std::abs(gait->yaw_rate) > 0.35f;

  float stride = gait->leg_length * gait->stride_scale;
  if (stride < 0.05f) stride = 0.05f;
  float step_rate = turn_in_place ? gait->turn_step_rate : (speed / stride);
  step_rate = clampf(step_rate, 0.0f, 4.0f);
  if (speed > 0.01f || turn_in_place) {
    gait->phase += step_rate * dt * 2.0f * kPi;
    if (gait->phase > 2.0f * kPi) gait->phase = std::fmod(gait->phase, 2.0f * kPi);
  }

  const float phase_l = gait->phase;
  const float phase_r = gait->phase + kPi;
  const float swing_l = 0.5f - 0.5f * std::cos(phase_l);
  const float swing_r = 0.5f - 0.5f * std::cos(phase_r);
  const float lift_l = std::max(0.0f, std::sin(phase_l));
  const float lift_r = std::max(0.0f, std::sin(phase_r));
  const float bob = std::sin(gait->phase * 2.0f);

  const float pelvis_bob = gait->pelvis_bob_scale * gait->leg_length * speed_norm;
  const float pelvis_sway = gait->pelvis_sway_scale * gait->leg_length * speed_norm;
  const float pelvis_roll = gait->pelvis_roll_scale * speed_norm;

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
  const float lean_fwd = clampf(accel_fwd / std::max(max_speed, 0.1f), -1.0f, 1.0f) * gait->pelvis_lean_scale;
  const float lean_side = clampf(accel_side / std::max(max_speed, 0.1f), -1.0f, 1.0f) * gait->pelvis_lean_scale;

  const float landing_alpha = (gait->landing_timer > 0.0f) ? (gait->landing_timer / 0.18f) : 0.0f;
  const float landing_drop = -gait->landing_compress * gait->leg_length * landing_alpha;

  if (gait->enable_pelvis_motion) {
    const float sway_dir = (swing_l > 0.5f) ? 1.0f : -1.0f;
    add_pos(*skeleton, gait->bone_hips,
            pelvis_sway * sway_dir, pelvis_bob * bob + landing_drop, 0.0f);
    add_rot(*skeleton, gait->bone_hips,
            -lean_fwd * 0.25f, 0.0f, -pelvis_roll * sway_dir);
  }

  add_rot(*skeleton, gait->bone_spine, lean_fwd * 0.2f, 0.0f, lean_side * 0.2f);
  add_rot(*skeleton, gait->bone_chest, lean_fwd * 0.15f, 0.0f, lean_side * 0.15f);
  add_rot(*skeleton, gait->bone_neck, lean_fwd * 0.06f, 0.0f, lean_side * 0.04f);
  add_rot(*skeleton, gait->bone_head, lean_fwd * 0.04f, 0.0f, lean_side * 0.03f);

  const float thigh_amp = (0.70f * speed_norm) + 0.05f;
  const float calf_amp = 0.90f * speed_norm;
  add_rot(*skeleton, gait->bone_l_thigh, thigh_amp * std::sin(phase_l), 0.0f, 0.0f);
  add_rot(*skeleton, gait->bone_r_thigh, thigh_amp * std::sin(phase_r), 0.0f, 0.0f);
  add_rot(*skeleton, gait->bone_l_calf, calf_amp * lift_l * lift_l, 0.0f, 0.0f);
  add_rot(*skeleton, gait->bone_r_calf, calf_amp * lift_r * lift_r, 0.0f, 0.0f);

  if (gait->enable_arm_swing) {
    const float arm_amp = gait->arm_swing_scale * speed_norm + 0.02f;
    add_rot(*skeleton, gait->bone_l_upper_arm, arm_amp * std::sin(phase_r), 0.0f, gait->arm_tuck);
    add_rot(*skeleton, gait->bone_r_upper_arm, arm_amp * std::sin(phase_l), 0.0f, -gait->arm_tuck);
    add_rot(*skeleton, gait->bone_l_lower_arm, 0.35f * arm_amp * std::sin(phase_r), 0.0f, 0.0f);
    add_rot(*skeleton, gait->bone_r_lower_arm, 0.35f * arm_amp * std::sin(phase_l), 0.0f, 0.0f);
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
  const Vec3 l_hip = safe_pos(gait->bone_l_thigh);
  const Vec3 l_knee = safe_pos(gait->bone_l_calf);
  const Vec3 l_foot = safe_pos(gait->bone_l_foot);
  const Vec3 r_hip = safe_pos(gait->bone_r_thigh);
  const Vec3 r_knee = safe_pos(gait->bone_r_calf);
  const Vec3 r_foot = safe_pos(gait->bone_r_foot);

  const Vec3 root_pos = from_array(transform->position);
  const Vec3 l_foot_w = to_world_point(*transform, l_foot);
  const Vec3 r_foot_w = to_world_point(*transform, r_foot);
  Vec3 move_dir = (speed > 0.05f) ? normalize(planar) : forward;
  Vec3 lateral = right;
  const float step_len = stride * 0.5f * speed_norm;
  const float lateral_offset = gait->hip_width * 0.5f;
  const float step_height = gait->step_height_scale * gait->leg_length;

  auto compute_step_target = [&](float side_sign) {
    Vec3 offset = add(mul(move_dir, step_len), mul(lateral, lateral_offset * side_sign));
    if (turn_in_place) {
      const float turn_dir = (gait->yaw_rate >= 0.0f) ? 1.0f : -1.0f;
      offset = rotate_y(mul(right, lateral_offset * side_sign), turn_dir * 0.4f);
    }
    Vec3 desired = add(root_pos, offset);
    const float ray_height = gait->leg_length * 0.6f;
    RayHit hit = raycast_down(registry, add(desired, mul(up_axis(), ray_height)), ray_height * 2.0f);
    if (hit.hit) {
      desired.y = hit.point.y;
    }
    return desired;
  };

  Vec3 l_step_w = compute_step_target(-1.0f);
  Vec3 r_step_w = compute_step_target(1.0f);
  Vec3 l_step = to_local_point(*transform, l_step_w);
  Vec3 r_step = to_local_point(*transform, r_step_w);

  Vec3 l_lock_w = from_array(gait->left_lock_pos);
  Vec3 r_lock_w = from_array(gait->right_lock_pos);
  update_foot_lock_internal(gait->left_locked, l_lock_w, l_foot_w, grounded, swing_l,
                            gait->foot_lock_in, gait->foot_lock_out, dt);
  update_foot_lock_internal(gait->right_locked, r_lock_w, r_foot_w, grounded, swing_r,
                            gait->foot_lock_in, gait->foot_lock_out, dt);
  to_array(l_lock_w, gait->left_lock_pos);
  to_array(r_lock_w, gait->right_lock_pos);

  Vec3 l_lock = to_local_point(*transform, l_lock_w);
  Vec3 r_lock = to_local_point(*transform, r_lock_w);

  auto blend_foot = [&](const Vec3& lock_pos, const Vec3& step_pos, float swing) {
    const float t = smoothstep01(swing);
    Vec3 out = lerp(lock_pos, step_pos, t);
    out.y += std::sin(kPi * t) * step_height;
    return out;
  };

  Vec3 l_target = gait->left_locked ? l_lock : blend_foot(l_lock, l_step, swing_l);
  Vec3 r_target = gait->right_locked ? r_lock : blend_foot(r_lock, r_step, swing_r);

  if (!grounded) {
    l_target = l_foot;
    r_target = r_foot;
  }

  if (gait->enable_ik) {
    const float ik_weight_l = grounded ? (1.0f - swing_l) : 0.0f;
    const float ik_weight_r = grounded ? (1.0f - swing_r) : 0.0f;
    const Vec3 target_l = lerp(l_foot, l_target, ik_weight_l);
    const Vec3 target_r = lerp(r_foot, r_target, ik_weight_r);

    const float max_pelvis_drop = gait->leg_length * 0.12f;
    const float desired_drop = std::min(0.0f, std::min(target_l.y - l_foot.y, target_r.y - r_foot.y));
    const float clamped_drop = clampf(desired_drop, -max_pelvis_drop, 0.0f);
    const float pelvis_alpha = 1.0f - std::exp(-dt * gait->ik_blend_speed);
    gait->pelvis_offset[1] += (clamped_drop - gait->pelvis_offset[1]) * pelvis_alpha;
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

    Vec3 l_plane = cross(sub(l_knee2, l_hip2), sub(l_foot2, l_knee2));
    Vec3 r_plane = cross(sub(r_knee2, r_hip2), sub(r_foot2, r_knee2));
    l_plane = normalize(l_plane);
    r_plane = normalize(r_plane);
    if (length(l_plane) < 0.0001f) l_plane = v3(0.0f, 0.0f, 1.0f);
    if (length(r_plane) < 0.0001f) r_plane = v3(0.0f, 0.0f, 1.0f);

    const Vec3 l_knee_target = solve_two_bone_ik(l_hip2, l_knee2, l_foot2, target_l, l_plane);
    const Vec3 r_knee_target = solve_two_bone_ik(r_hip2, r_knee2, r_foot2, target_r, r_plane);
    apply_bone_aim(*skeleton, world, gait->bone_l_thigh, gait->bone_l_calf, l_knee_target);
    apply_bone_aim(*skeleton, world, gait->bone_r_thigh, gait->bone_r_calf, r_knee_target);

    for (size_t i = 0; i < skeleton->bones.size(); ++i) {
      locals[i] = skeleton->bones[i].local_pose;
    }
    compute_world_matrices(*skeleton, locals, world);
    apply_bone_aim(*skeleton, world, gait->bone_l_calf, gait->bone_l_foot, target_l);
    apply_bone_aim(*skeleton, world, gait->bone_r_calf, gait->bone_r_foot, target_r);
  }
}

void update_procedural_gaits(ecs::Registry& registry, float dt) {
  static DebugLines lines;
  lines.clear();
  for (auto& kv : registry.procedural_gaits()) {
    auto& gait = kv.second;
    if (!gait.enabled) continue;
    update_procedural_gait(registry, kv.first, dt);
    if (!gait.debug_draw) continue;
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
    Vec3 l_lock_w = from_array(gait.left_lock_pos);
    Vec3 r_lock_w = from_array(gait.right_lock_pos);
    lines.add_line(l_foot_w, l_lock_w, v3(0.2f, 0.8f, 0.2f));
    lines.add_line(r_foot_w, r_lock_w, v3(0.2f, 0.8f, 0.2f));
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
  }

  if (lines.count > 0) {
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
  const Vec3 plane = normalize(from_array(plane_hint.position));
  const Vec3 knee_out = solve_two_bone_ik(hip_p, knee_p, ankle_p, tgt, plane);
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
