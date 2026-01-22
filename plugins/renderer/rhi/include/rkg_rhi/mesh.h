#pragma once

#include <array>
#include <vector>

namespace rkg::rhi {

struct Vertex {
  float position[3];
  float color[4];
};

struct Mesh {
  std::vector<Vertex> vertices;
  std::vector<uint32_t> indices;
};

struct Material {
  float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
};

inline Mesh make_builtin_quad() {
  Mesh mesh;
  mesh.vertices = {
      {{-0.5f, -0.5f, 0.0f}, {1.0f, 0.2f, 0.2f, 1.0f}},
      {{0.5f, -0.5f, 0.0f}, {0.2f, 1.0f, 0.2f, 1.0f}},
      {{0.5f, 0.5f, 0.0f}, {0.2f, 0.2f, 1.0f, 1.0f}},
      {{-0.5f, 0.5f, 0.0f}, {1.0f, 1.0f, 0.2f, 1.0f}},
  };
  mesh.indices = {0, 1, 2, 2, 3, 0};
  return mesh;
}

} // namespace rkg::rhi
