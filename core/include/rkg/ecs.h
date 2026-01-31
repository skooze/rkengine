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
  float max_speed = 4.0f;
  float accel = 20.0f;
  float friction = 8.0f;
  float braking_deceleration = 16.0f;
  float braking_friction_factor = 2.0f;
  float max_air_speed = 4.0f;
  float max_air_accel = 12.0f;
  float air_control = 0.35f;
  float air_control_boost_multiplier = 2.0f;
  float air_control_boost_velocity_threshold = 1.0f;
  float gravity = -9.8f;
  float terminal_velocity = 55.0f;
  float jump_impulse = 5.0f;
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
  bool grounded = false;
  MovementMode mode = MovementMode::Falling;
  float vertical_velocity = 0.0f;
  float ground_height = 0.0f;
  bool jump_held = false;
  float time_since_grounded = 0.0f;
  float coyote_time = 0.1f;
  float jump_buffer_time = 0.1f;
  float jump_buffer = 0.0f;
  float desired_move_dir[3] = {0.0f, 0.0f, 0.0f};
  float desired_move_speed = 0.0f;
  bool use_desired_input = false;
  float smoothed_input[3] = {0.0f, 0.0f, 0.0f};
  float external_base_velocity[3] = {0.0f, 0.0f, 0.0f};
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
};

} // namespace rkg::ecs
