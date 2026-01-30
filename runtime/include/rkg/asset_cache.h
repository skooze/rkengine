#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "rkg/ecs.h"

namespace rkg::runtime {

struct MeshInfo {
  uint32_t vertex_count = 0;
  uint32_t index_count = 0;
  bool has_normals = false;
  bool has_uv0 = false;
  bool has_tangents = false;
  bool has_joints = false;
  bool has_weights = false;
};

struct MaterialInfo {
  std::string name;
  std::array<float, 4> base_color{1.0f, 1.0f, 1.0f, 1.0f};
  std::string base_color_texture;
};

struct SkeletonInfo {
  std::vector<rkg::ecs::Bone> bones;
  std::vector<std::array<float, 16>> inverse_bind_mats;
};

struct AssetRecord {
  std::string name;
  std::filesystem::path dir;
  MeshInfo mesh;
  std::vector<MaterialInfo> materials;
  std::vector<std::filesystem::path> textures;
  SkeletonInfo skeleton;
};

class AssetCache {
 public:
  bool load_from_content_root(const std::filesystem::path& content_root, std::string& error);
  bool load_asset_dir(const std::filesystem::path& asset_dir, std::string& error);

  const AssetRecord* find(const std::string& name) const;
  const std::unordered_map<std::string, AssetRecord>& assets() const { return assets_; }

 private:
  std::unordered_map<std::string, AssetRecord> assets_;
};

} // namespace rkg::runtime
