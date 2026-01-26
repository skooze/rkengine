#include "rkg/ecs.h"

#include "rkg/math.h"

#include <functional>

namespace rkg::ecs {

Entity Registry::create_entity() {
  return next_id_++;
}

void Registry::destroy_entity(Entity entity) {
  transforms_.erase(entity);
  renderables_.erase(entity);
  velocities_.erase(entity);
  rigid_bodies_.erase(entity);
  colliders_.erase(entity);
  character_controllers_.erase(entity);
  skeletons_.erase(entity);
}

void Registry::set_transform(Entity entity, const Transform& transform) {
  transforms_[entity] = transform;
}

Transform* Registry::get_transform(Entity entity) {
  auto it = transforms_.find(entity);
  if (it == transforms_.end()) {
    return nullptr;
  }
  return &it->second;
}

const Transform* Registry::get_transform(Entity entity) const {
  auto it = transforms_.find(entity);
  if (it == transforms_.end()) {
    return nullptr;
  }
  return &it->second;
}

void Registry::set_renderable(Entity entity, const Renderable& renderable) {
  renderables_[entity] = renderable;
}

Renderable* Registry::get_renderable(Entity entity) {
  auto it = renderables_.find(entity);
  if (it == renderables_.end()) {
    return nullptr;
  }
  return &it->second;
}

const Renderable* Registry::get_renderable(Entity entity) const {
  auto it = renderables_.find(entity);
  if (it == renderables_.end()) {
    return nullptr;
  }
  return &it->second;
}

void compute_skeleton_world_pose(const Transform& root, Skeleton& skeleton) {
  const size_t bone_count = skeleton.bones.size();
  if (bone_count == 0) {
    skeleton.world_pose.clear();
    return;
  }
  if (skeleton.world_pose.size() != bone_count) {
    skeleton.world_pose.resize(bone_count);
  }

  const rkg::Vec3 root_pos{root.position[0], root.position[1], root.position[2]};
  const rkg::Vec3 root_rot{root.rotation[0], root.rotation[1], root.rotation[2]};
  const rkg::Vec3 root_scale{root.scale[0], root.scale[1], root.scale[2]};
  const Mat4 root_model = mat4_mul(mat4_translation(root_pos),
                                   mat4_mul(mat4_rotation_xyz(root_rot), mat4_scale(root_scale)));

  std::vector<Mat4> world_mats(bone_count);
  std::vector<uint8_t> computed(bone_count, 0);

  auto local_matrix = [&](size_t i) {
    const Bone& bone = skeleton.bones[i];
    const rkg::Vec3 local_pos{bone.local_pose.position[0], bone.local_pose.position[1], bone.local_pose.position[2]};
    const rkg::Vec3 local_rot{bone.local_pose.rotation[0], bone.local_pose.rotation[1], bone.local_pose.rotation[2]};
    const rkg::Vec3 local_scale{bone.local_pose.scale[0], bone.local_pose.scale[1], bone.local_pose.scale[2]};
    return mat4_mul(mat4_translation(local_pos),
                    mat4_mul(mat4_rotation_xyz(local_rot), mat4_scale(local_scale)));
  };

  std::function<void(size_t)> eval = [&](size_t i) {
    if (computed[i]) {
      return;
    }
    const Bone& bone = skeleton.bones[i];
    const Mat4 local = local_matrix(i);
    if (bone.parent_index >= 0 && static_cast<size_t>(bone.parent_index) < bone_count) {
      const size_t parent = static_cast<size_t>(bone.parent_index);
      if (parent != i) {
        eval(parent);
        world_mats[i] = mat4_mul(world_mats[parent], local);
      } else {
        world_mats[i] = mat4_mul(root_model, local);
      }
    } else {
      world_mats[i] = mat4_mul(root_model, local);
    }
    computed[i] = 1;
  };

  for (size_t i = 0; i < bone_count; ++i) {
    eval(i);
  }

  for (size_t i = 0; i < bone_count; ++i) {
    const Bone& bone = skeleton.bones[i];
    Transform world{};
    world.position[0] = world_mats[i].m[12];
    world.position[1] = world_mats[i].m[13];
    world.position[2] = world_mats[i].m[14];
    if (bone.parent_index < 0) {
      world.rotation[0] = root.rotation[0];
      world.rotation[1] = root.rotation[1];
      world.rotation[2] = root.rotation[2];
    } else {
      world.rotation[0] = bone.local_pose.rotation[0];
      world.rotation[1] = bone.local_pose.rotation[1];
      world.rotation[2] = bone.local_pose.rotation[2];
    }
    world.scale[0] = bone.local_pose.scale[0];
    world.scale[1] = bone.local_pose.scale[1];
    world.scale[2] = bone.local_pose.scale[2];
    skeleton.world_pose[i] = world;
  }
}

void Registry::remove_renderable(Entity entity) {
  renderables_.erase(entity);
}

void Registry::set_velocity(Entity entity, const Velocity& velocity) {
  velocities_[entity] = velocity;
}

Velocity* Registry::get_velocity(Entity entity) {
  auto it = velocities_.find(entity);
  if (it == velocities_.end()) {
    return nullptr;
  }
  return &it->second;
}

const Velocity* Registry::get_velocity(Entity entity) const {
  auto it = velocities_.find(entity);
  if (it == velocities_.end()) {
    return nullptr;
  }
  return &it->second;
}

void Registry::remove_velocity(Entity entity) {
  velocities_.erase(entity);
}

void Registry::set_rigid_body(Entity entity, const RigidBody& body) {
  rigid_bodies_[entity] = body;
}

RigidBody* Registry::get_rigid_body(Entity entity) {
  auto it = rigid_bodies_.find(entity);
  if (it == rigid_bodies_.end()) {
    return nullptr;
  }
  return &it->second;
}

const RigidBody* Registry::get_rigid_body(Entity entity) const {
  auto it = rigid_bodies_.find(entity);
  if (it == rigid_bodies_.end()) {
    return nullptr;
  }
  return &it->second;
}

void Registry::remove_rigid_body(Entity entity) {
  rigid_bodies_.erase(entity);
}

void Registry::set_collider(Entity entity, const Collider& collider) {
  colliders_[entity] = collider;
}

Collider* Registry::get_collider(Entity entity) {
  auto it = colliders_.find(entity);
  if (it == colliders_.end()) {
    return nullptr;
  }
  return &it->second;
}

const Collider* Registry::get_collider(Entity entity) const {
  auto it = colliders_.find(entity);
  if (it == colliders_.end()) {
    return nullptr;
  }
  return &it->second;
}

void Registry::remove_collider(Entity entity) {
  colliders_.erase(entity);
}

void Registry::set_character_controller(Entity entity, const CharacterController& controller) {
  character_controllers_[entity] = controller;
}

CharacterController* Registry::get_character_controller(Entity entity) {
  auto it = character_controllers_.find(entity);
  if (it == character_controllers_.end()) {
    return nullptr;
  }
  return &it->second;
}

const CharacterController* Registry::get_character_controller(Entity entity) const {
  auto it = character_controllers_.find(entity);
  if (it == character_controllers_.end()) {
    return nullptr;
  }
  return &it->second;
}

void Registry::remove_character_controller(Entity entity) {
  character_controllers_.erase(entity);
}

void Registry::set_skeleton(Entity entity, const Skeleton& skeleton) {
  skeletons_[entity] = skeleton;
}

Skeleton* Registry::get_skeleton(Entity entity) {
  auto it = skeletons_.find(entity);
  if (it == skeletons_.end()) {
    return nullptr;
  }
  return &it->second;
}

const Skeleton* Registry::get_skeleton(Entity entity) const {
  auto it = skeletons_.find(entity);
  if (it == skeletons_.end()) {
    return nullptr;
  }
  return &it->second;
}

void Registry::remove_skeleton(Entity entity) {
  skeletons_.erase(entity);
}

size_t Registry::entity_count() const {
  return transforms_.size();
}

std::vector<Entity> Registry::entities() const {
  std::vector<Entity> out;
  out.reserve(transforms_.size());
  for (const auto& kv : transforms_) {
    out.push_back(kv.first);
  }
  return out;
}

std::unordered_map<Entity, Velocity>& Registry::velocities() {
  return velocities_;
}

const std::unordered_map<Entity, Velocity>& Registry::velocities() const {
  return velocities_;
}

std::unordered_map<Entity, RigidBody>& Registry::rigid_bodies() {
  return rigid_bodies_;
}

const std::unordered_map<Entity, RigidBody>& Registry::rigid_bodies() const {
  return rigid_bodies_;
}

std::unordered_map<Entity, Collider>& Registry::colliders() {
  return colliders_;
}

const std::unordered_map<Entity, Collider>& Registry::colliders() const {
  return colliders_;
}

std::unordered_map<Entity, CharacterController>& Registry::character_controllers() {
  return character_controllers_;
}

const std::unordered_map<Entity, CharacterController>& Registry::character_controllers() const {
  return character_controllers_;
}

std::unordered_map<Entity, Skeleton>& Registry::skeletons() {
  return skeletons_;
}

const std::unordered_map<Entity, Skeleton>& Registry::skeletons() const {
  return skeletons_;
}

} // namespace rkg::ecs
