#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace rkg::ecs {

using Entity = uint32_t;
static constexpr Entity kInvalidEntity = 0;

struct Transform {
  float position[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float scale[3] = {1.0f, 1.0f, 1.0f};
};

enum class MeshId : uint32_t {
  Cube = 0,
  Quad = 1,
  Unknown = 2
};

struct Renderable {
  MeshId mesh = MeshId::Cube;
  float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct Velocity {
  float linear[3] = {0.0f, 0.0f, 0.0f};
};

struct RigidBody {
  float mass = 1.0f;
  bool is_kinematic = false;
  float linear_velocity[3] = {0.0f, 0.0f, 0.0f};
  float force_accumulator[3] = {0.0f, 0.0f, 0.0f};
};

enum class ColliderType : uint8_t {
  Plane = 0,
  AABB = 1,
  Capsule = 2
};

enum class MovementMode : uint8_t {
  Grounded = 0,
  Falling = 1,
  Sliding = 2
};

struct Collider {
  ColliderType type = ColliderType::Plane;
  float center[3] = {0.0f, 0.0f, 0.0f};
  float half_extents[3] = {0.5f, 0.5f, 0.5f}; // AABB
  float normal[3] = {0.0f, 1.0f, 0.0f};       // Plane
  float distance = 0.0f;                      // Plane
  float radius = 0.5f;                        // Capsule
  float half_height = 1.0f;                   // Capsule (half height of cylinder)
};

struct CharacterController {
  float radius = 0.35f;
  float half_height = 0.9f;
  float center_offset = 0.0f;
  float max_speed = 1.8f;
  float sprint_multiplier = 2.0f;
  float accel = 20.0f;
  float friction = 8.0f;
  float braking_deceleration = 16.0f;
  float braking_friction_factor = 2.0f;
  float max_air_speed = 1.8f;
  float max_air_accel = 12.0f;
  float air_control = 0.35f;
  float air_control_boost_multiplier = 2.0f;
  float air_control_boost_velocity_threshold = 1.0f;
  float gravity = -9.8f;
  float terminal_velocity = 55.0f;
  float jump_impulse = 5.0f;
  float rotation_rate_deg_per_sec = 720.0f;
  float min_speed_to_rotate = 0.2f;
  float slope_limit_deg = 45.0f;
  float step_height = 0.35f;
  float skin_width = 0.02f;
  float ground_snap_max = 0.2f;
  float max_depenetration_velocity = 10.0f;
  int max_sweep_iterations = 4;
  int max_substeps = 8;
  float max_substep_dt = 1.0f / 60.0f;
  float turn_boost_factor = 0.5f;
  float turn_speed_threshold = 1.0f;
  float input_smooth_tau = 0.08f;
  bool enable_procedural_gait = true;
  bool grounded = false;
  MovementMode mode = MovementMode::Falling;
  bool is_sprinting = false;
  float vertical_velocity = 0.0f;
  float ground_height = 0.0f;
  bool jump_held = false;
  float time_since_grounded = 0.0f;
  float coyote_time = 0.1f;
  float jump_buffer_time = 0.1f;
  float jump_buffer = 0.0f;
  float just_jumped_time = 0.0f;
  float just_jumped_no_snap_window = 0.1f;
  float desired_move_dir[3] = {0.0f, 0.0f, 0.0f};
  float desired_move_speed = 0.0f;
  bool use_desired_input = false;
  float smoothed_input[3] = {0.0f, 0.0f, 0.0f};
  float raw_input_mag = 0.0f;
  float external_base_velocity[3] = {0.0f, 0.0f, 0.0f};
};

struct ProceduralGait {
  bool enabled = true;
  bool debug_draw = false;
  bool enable_ik = true;
  bool enable_foot_lock = true;
  bool enable_arm_swing = true;
  bool enable_pelvis_motion = true;
  bool enable_turn_in_place = true;

  float walk_speed = 2.2f;
  float sprint_speed = 4.0f;
  float stride_scale = 0.9f;
  float step_height_scale = 0.3f;
  float pelvis_bob_scale = 0.03f;
  float pelvis_sway_scale = 0.02f;
  float pelvis_roll_scale = 0.15f;
  float pelvis_lean_scale = 0.25f;
  float arm_swing_scale = 0.45f;
  float arm_tuck = -0.2f;
  float turn_in_place_speed = 0.25f;
  float turn_step_rate = 0.8f;
  float foot_lock_in = 0.25f;
  float foot_lock_out = 0.65f;
  float ik_blend_speed = 8.0f;
  float lateral_step_scale = 0.35f;
  float knee_plane_bias = 0.9f;
  float landing_compress = 0.04f;
  float landing_recover = 10.0f;
  float input_smooth_tau = 0.05f;
  float lean_fwd = 0.0f;
  float lean_side = 0.0f;

  float phase = 0.0f;
  float speed_smoothed = 0.0f;
  float yaw = 0.0f;
  float last_yaw = 0.0f;
  float yaw_rate = 0.0f;
  float landing_timer = 0.0f;
  bool was_grounded = false;
  float last_velocity[3] = {0.0f, 0.0f, 0.0f};
  float pelvis_offset[3] = {0.0f, 0.0f, 0.0f};
  bool left_locked = false;
  bool right_locked = false;
  float left_lock_pos[3] = {0.0f, 0.0f, 0.0f};
  float right_lock_pos[3] = {0.0f, 0.0f, 0.0f};
  bool left_step_active = false;
  bool right_step_active = false;
  float left_step_pos[3] = {0.0f, 0.0f, 0.0f};
  float right_step_pos[3] = {0.0f, 0.0f, 0.0f};
  float left_swing_start_pos[3] = {0.0f, 0.0f, 0.0f};
  float right_swing_start_pos[3] = {0.0f, 0.0f, 0.0f};
  float smooth_left_target[3] = {0.0f, 0.0f, 0.0f};
  float smooth_right_target[3] = {0.0f, 0.0f, 0.0f};
  float foot_home_l[3] = {0.0f, 0.0f, 0.0f};
  float foot_home_r[3] = {0.0f, 0.0f, 0.0f};
  float side_sign_l = -1.0f;
  float side_sign_r = 1.0f;
  float knee_prev_y_l[3] = {0.0f, 0.0f, 1.0f};
  float knee_prev_y_r[3] = {0.0f, 0.0f, 1.0f};
  float debug_left_target[3] = {0.0f, 0.0f, 0.0f};
  float debug_right_target[3] = {0.0f, 0.0f, 0.0f};
  float debug_left_pole[3] = {0.0f, 0.0f, 0.0f};
  float debug_right_pole[3] = {0.0f, 0.0f, 0.0f};
  float debug_forward[3] = {0.0f, 0.0f, 0.0f};
  float debug_right[3] = {0.0f, 0.0f, 0.0f};
  float debug_hips_world[3] = {0.0f, 0.0f, 0.0f};
  float frame_side_world[3] = {1.0f, 0.0f, 0.0f};
  float frame_fwd_world[3] = {0.0f, 0.0f, 1.0f};
  float frame_side_entity[3] = {1.0f, 0.0f, 0.0f};
  float frame_fwd_entity[3] = {0.0f, 0.0f, 1.0f};
  float debug_knee_lat_l = 0.0f;
  float debug_knee_lat_r = 0.0f;
  float debug_target_lat_l = 0.0f;
  float debug_target_lat_r = 0.0f;
  float debug_continuity_l = 1.0f;
  float debug_continuity_r = 1.0f;

  uint32_t bone_root = UINT32_MAX;
  uint32_t bone_hips = UINT32_MAX;
  uint32_t bone_spine = UINT32_MAX;
  uint32_t bone_chest = UINT32_MAX;
  uint32_t bone_neck = UINT32_MAX;
  uint32_t bone_head = UINT32_MAX;
  uint32_t bone_l_thigh = UINT32_MAX;
  uint32_t bone_l_calf = UINT32_MAX;
  uint32_t bone_l_foot = UINT32_MAX;
  uint32_t bone_l_toe = UINT32_MAX;
  uint32_t bone_r_thigh = UINT32_MAX;
  uint32_t bone_r_calf = UINT32_MAX;
  uint32_t bone_r_foot = UINT32_MAX;
  uint32_t bone_r_toe = UINT32_MAX;
  uint32_t bone_l_shoulder = UINT32_MAX;
  uint32_t bone_r_shoulder = UINT32_MAX;
  uint32_t bone_l_upper_arm = UINT32_MAX;
  uint32_t bone_l_lower_arm = UINT32_MAX;
  uint32_t bone_r_upper_arm = UINT32_MAX;
  uint32_t bone_r_lower_arm = UINT32_MAX;

  bool map_valid = false;
  float leg_length = 1.0f;
  float hip_width = 0.3f;
  float foot_length = 0.25f;
  float knee_plane_l[3] = {0.0f, 0.0f, 1.0f};
  float knee_plane_r[3] = {0.0f, 0.0f, 1.0f};
};

struct Bone {
  std::string name;
  int parent_index = -1;
  Transform bind_local;
  Transform local_pose;
};

struct Skeleton {
  std::vector<Bone> bones;
  std::vector<Transform> world_pose;
};

struct SkeletonRef {
  std::string asset;
};

void compute_skeleton_world_pose(const Transform& root, Skeleton& skeleton);

class Registry {
 public:
  Entity create_entity();
  void destroy_entity(Entity entity);
  void set_transform(Entity entity, const Transform& transform);
  Transform* get_transform(Entity entity);
  const Transform* get_transform(Entity entity) const;
  void set_renderable(Entity entity, const Renderable& renderable);
  Renderable* get_renderable(Entity entity);
  const Renderable* get_renderable(Entity entity) const;
  void remove_renderable(Entity entity);
  void set_velocity(Entity entity, const Velocity& velocity);
  Velocity* get_velocity(Entity entity);
  const Velocity* get_velocity(Entity entity) const;
  void remove_velocity(Entity entity);
  void set_rigid_body(Entity entity, const RigidBody& body);
  RigidBody* get_rigid_body(Entity entity);
  const RigidBody* get_rigid_body(Entity entity) const;
  void remove_rigid_body(Entity entity);
  void set_collider(Entity entity, const Collider& collider);
  Collider* get_collider(Entity entity);
  const Collider* get_collider(Entity entity) const;
  void remove_collider(Entity entity);
  void set_character_controller(Entity entity, const CharacterController& controller);
  CharacterController* get_character_controller(Entity entity);
  const CharacterController* get_character_controller(Entity entity) const;
  void remove_character_controller(Entity entity);
  void set_skeleton(Entity entity, const Skeleton& skeleton);
  Skeleton* get_skeleton(Entity entity);
  const Skeleton* get_skeleton(Entity entity) const;
  void remove_skeleton(Entity entity);
  void set_skeleton_ref(Entity entity, const SkeletonRef& ref);
  SkeletonRef* get_skeleton_ref(Entity entity);
  const SkeletonRef* get_skeleton_ref(Entity entity) const;
  void remove_skeleton_ref(Entity entity);
  void set_procedural_gait(Entity entity, const ProceduralGait& gait);
  ProceduralGait* get_procedural_gait(Entity entity);
  const ProceduralGait* get_procedural_gait(Entity entity) const;
  void remove_procedural_gait(Entity entity);
  size_t entity_count() const;
  std::vector<Entity> entities() const;
  std::unordered_map<Entity, Velocity>& velocities();
  const std::unordered_map<Entity, Velocity>& velocities() const;
  std::unordered_map<Entity, RigidBody>& rigid_bodies();
  const std::unordered_map<Entity, RigidBody>& rigid_bodies() const;
  std::unordered_map<Entity, Collider>& colliders();
  const std::unordered_map<Entity, Collider>& colliders() const;
  std::unordered_map<Entity, CharacterController>& character_controllers();
  const std::unordered_map<Entity, CharacterController>& character_controllers() const;
  std::unordered_map<Entity, Skeleton>& skeletons();
  const std::unordered_map<Entity, Skeleton>& skeletons() const;
  std::unordered_map<Entity, SkeletonRef>& skeleton_refs();
  const std::unordered_map<Entity, SkeletonRef>& skeleton_refs() const;
  std::unordered_map<Entity, ProceduralGait>& procedural_gaits();
  const std::unordered_map<Entity, ProceduralGait>& procedural_gaits() const;

 private:
  Entity next_id_ = 1;
  std::unordered_map<Entity, Transform> transforms_;
  std::unordered_map<Entity, Renderable> renderables_;
  std::unordered_map<Entity, Velocity> velocities_;
  std::unordered_map<Entity, RigidBody> rigid_bodies_;
  std::unordered_map<Entity, Collider> colliders_;
  std::unordered_map<Entity, CharacterController> character_controllers_;
  std::unordered_map<Entity, Skeleton> skeletons_;
  std::unordered_map<Entity, SkeletonRef> skeleton_refs_;
  std::unordered_map<Entity, ProceduralGait> procedural_gaits_;
};

} // namespace rkg::ecs
