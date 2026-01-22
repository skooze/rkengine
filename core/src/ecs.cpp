#include "rkg/ecs.h"

namespace rkg::ecs {

Entity Registry::create_entity() {
  return next_id_++;
}

void Registry::destroy_entity(Entity entity) {
  transforms_.erase(entity);
  renderables_.erase(entity);
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

} // namespace rkg::ecs
