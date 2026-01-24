#include "rkg/ecs.h"

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
