#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rkg::asset {

struct TextureInfo {
  std::string name;
  std::string file;
  std::string mime_type;
  int width = 0;
  int height = 0;
  int channels = 0;
};

struct MaterialInfo {
  std::string name;
  float base_color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  int base_color_texture = -1;
};

struct MeshPrimitiveInfo {
  uint32_t vertex_count = 0;
  uint32_t index_count = 0;
  bool has_normals = false;
  bool has_uv0 = false;
  bool has_tangents = false;
};

struct ImportedAssetSummary {
  std::string name;
  uint32_t mesh_count = 0;
  uint32_t primitive_count = 0;
  uint32_t exported_primitives = 0;
  uint32_t total_primitives = 0;
  uint32_t material_count = 0;
  uint32_t texture_count = 0;
  uint32_t skin_count = 0;
  uint32_t joint_count = 0;
};

struct ImportOptions {
  bool overwrite = false;
  bool write_textures = true;
  bool validate_only = false;
};

struct ImportResult {
  bool ok = false;
  std::string error;
  std::string warning;
  ImportedAssetSummary asset;
  MeshPrimitiveInfo mesh;
  std::vector<MaterialInfo> materials;
  std::vector<TextureInfo> textures;
};

}  // namespace rkg::asset
