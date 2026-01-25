#include "rkg/asset_cache.h"

#include "rkg/log.h"

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

#include <fstream>

namespace fs = std::filesystem;

namespace rkg::runtime {

namespace {

#if RKG_ENABLE_DATA_JSON
using json = nlohmann::json;
#endif

bool read_file_bytes(const fs::path& path, std::vector<uint8_t>& out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  in.seekg(0, std::ios::end);
  const auto size = in.tellg();
  if (size <= 0) return false;
  out.resize(static_cast<size_t>(size));
  in.seekg(0, std::ios::beg);
  in.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(out.size()));
  return true;
}

bool load_mesh_bin(const fs::path& path, MeshInfo& out, std::string& error) {
  std::vector<uint8_t> bytes;
  if (!read_file_bytes(path, bytes)) {
    error = "failed to read mesh.bin";
    return false;
  }
  if (bytes.size() < sizeof(uint32_t) * 5) {
    error = "mesh.bin too small";
    return false;
  }
  const uint32_t* header = reinterpret_cast<const uint32_t*>(bytes.data());
  const uint32_t magic = header[0];
  const uint32_t version = header[1];
  if (magic != 0x30474B52 || version != 1) {
    error = "mesh.bin header invalid";
    return false;
  }
  out.vertex_count = header[2];
  out.index_count = header[3];
  const uint32_t flags = header[4];
  out.has_normals = (flags & 1u) != 0;
  out.has_uv0 = (flags & 2u) != 0;
  out.has_tangents = (flags & 4u) != 0;
  return true;
}

#if RKG_ENABLE_DATA_JSON
bool load_json_file(const fs::path& path, json& out, std::string& error) {
  std::ifstream in(path);
  if (!in) {
    error = "failed to open json file";
    return false;
  }
  try {
    in >> out;
  } catch (const std::exception& e) {
    error = std::string("json parse failed: ") + e.what();
    return false;
  }
  return true;
}
#endif

} // namespace

const AssetRecord* AssetCache::find(const std::string& name) const {
  const auto it = assets_.find(name);
  if (it == assets_.end()) return nullptr;
  return &it->second;
}

bool AssetCache::load_asset_dir(const fs::path& asset_dir, std::string& error) {
  const fs::path asset_json = asset_dir / "asset.json";
  if (!fs::exists(asset_json)) {
    error = "asset.json missing";
    return false;
  }
#if !RKG_ENABLE_DATA_JSON
  error = "JSON support disabled";
  return false;
#else
  json asset_doc;
  if (!load_json_file(asset_json, asset_doc, error)) {
    return false;
  }
  AssetRecord record;
  record.dir = asset_dir;
  record.name = asset_doc.value("name", asset_dir.filename().string());

  const fs::path mesh_path = asset_dir / "mesh.bin";
  if (!load_mesh_bin(mesh_path, record.mesh, error)) {
    return false;
  }

  const fs::path materials_path = asset_dir / "materials.json";
  if (fs::exists(materials_path)) {
    json materials_doc;
    std::string mat_error;
    if (load_json_file(materials_path, materials_doc, mat_error)) {
      const auto materials = materials_doc.value("materials", json::array());
      for (const auto& entry : materials) {
        MaterialInfo info;
        info.name = entry.value("name", "");
        if (entry.contains("baseColorFactor") && entry["baseColorFactor"].is_array()) {
          const auto& arr = entry["baseColorFactor"];
          for (size_t i = 0; i < 4 && i < arr.size(); ++i) {
            info.base_color[i] = arr[i].get<float>();
          }
        }
        if (entry.contains("baseColorTexture") && entry["baseColorTexture"].is_string()) {
          info.base_color_texture = entry["baseColorTexture"].get<std::string>();
        }
        record.materials.push_back(std::move(info));
      }
    } else {
      rkg::log::warn(std::string("asset_cache: materials.json parse failed: ") + mat_error);
    }
  }

  const fs::path textures_dir = asset_dir / "textures";
  if (fs::exists(textures_dir) && fs::is_directory(textures_dir)) {
    for (const auto& entry : fs::directory_iterator(textures_dir)) {
      if (entry.is_regular_file()) {
        record.textures.push_back(entry.path());
      }
    }
  }

  assets_[record.name] = std::move(record);
  return true;
#endif
}

bool AssetCache::load_from_content_root(const fs::path& content_root, std::string& error) {
  const fs::path assets_dir = content_root / "assets";
  if (!fs::exists(assets_dir) || !fs::is_directory(assets_dir)) {
    error = "assets directory not found";
    return false;
  }
  for (const auto& entry : fs::directory_iterator(assets_dir)) {
    if (!entry.is_directory()) continue;
    std::string err;
    if (!load_asset_dir(entry.path(), err)) {
      rkg::log::warn(std::string("asset_cache: skip ") + entry.path().string() + ": " + err);
      continue;
    }
  }
  return true;
}

} // namespace rkg::runtime
