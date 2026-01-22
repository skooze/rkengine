#pragma once

#include <cstdint>
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

class Registry {
 public:
  Entity create_entity();
  void destroy_entity(Entity entity);
  void set_transform(Entity entity, const Transform& transform);
  Transform* get_transform(Entity entity);
  const Transform* get_transform(Entity entity) const;
  size_t entity_count() const;
  std::vector<Entity> entities() const;

 private:
  Entity next_id_ = 1;
  std::unordered_map<Entity, Transform> transforms_;
};

} // namespace rkg::ecs
