#include "rkg/asset_cache.h"

#include "rkg/log.h"
#include "rkg/asset_import.h"

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

#include <fstream>
#include <cstring>
#include <cstdlib>

namespace fs = std::filesystem;

namespace rkg::runtime {

namespace {

#if RKG_ENABLE_DATA_JSON
using json = nlohmann::json;

bool load_json_file(const fs::path& path, json& out, std::string& error);
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
  if (magic != 0x30474B52 || (version != 1 && version != 2)) {
    error = "mesh.bin header invalid";
    return false;
  }
  out.vertex_count = header[2];
  out.index_count = header[3];
  const uint32_t flags = header[4];
  out.has_normals = (flags & 1u) != 0;
  out.has_uv0 = (flags & 2u) != 0;
  out.has_tangents = (flags & 4u) != 0;
  out.has_joints = (flags & 8u) != 0;
  out.has_weights = (flags & 16u) != 0;
  const size_t pos_bytes = static_cast<size_t>(out.vertex_count) * 3 * sizeof(float);
  size_t offset = sizeof(uint32_t) * 5;
  if (bytes.size() >= offset + pos_bytes && out.vertex_count > 0) {
    const float* positions = reinterpret_cast<const float*>(bytes.data() + offset);
    std::array<float, 3> minv{positions[0], positions[1], positions[2]};
    std::array<float, 3> maxv{positions[0], positions[1], positions[2]};
    for (uint32_t i = 0; i < out.vertex_count; ++i) {
      const float x = positions[i * 3 + 0];
      const float y = positions[i * 3 + 1];
      const float z = positions[i * 3 + 2];
      if (x < minv[0]) minv[0] = x;
      if (y < minv[1]) minv[1] = y;
      if (z < minv[2]) minv[2] = z;
      if (x > maxv[0]) maxv[0] = x;
      if (y > maxv[1]) maxv[1] = y;
      if (z > maxv[2]) maxv[2] = z;
    }
    out.bounds_min = minv;
    out.bounds_max = maxv;
  }
  return true;
}

#if RKG_ENABLE_DATA_JSON
bool read_vec3(const json& arr, float out[3]) {
  if (!arr.is_array() || arr.size() < 3) return false;
  out[0] = arr[0].get<float>();
  out[1] = arr[1].get<float>();
  out[2] = arr[2].get<float>();
  return true;
}

bool load_skeleton_json(const fs::path& path, SkeletonInfo& out, std::string& error) {
  json doc;
  if (!load_json_file(path, doc, error)) {
    return false;
  }
  const auto bones = doc.value("bones", json::array());
  if (!bones.is_array()) {
    error = "skeleton.json bones missing";
    return false;
  }
  out.bones.clear();
  out.bones.reserve(bones.size());
  for (const auto& entry : bones) {
    if (!entry.is_object()) continue;
    rkg::ecs::Bone bone;
    bone.name = entry.value("name", "");
    bone.parent_index = entry.value("parent", -1);
    if (entry.contains("bind_local")) {
      const auto& bind = entry["bind_local"];
      if (bind.contains("position")) read_vec3(bind["position"], bone.bind_local.position);
      if (bind.contains("rotation")) read_vec3(bind["rotation"], bone.bind_local.rotation);
      if (bind.contains("scale")) read_vec3(bind["scale"], bone.bind_local.scale);
    }
    if (entry.contains("local_pose")) {
      const auto& pose = entry["local_pose"];
      if (pose.contains("position")) read_vec3(pose["position"], bone.local_pose.position);
      if (pose.contains("rotation")) read_vec3(pose["rotation"], bone.local_pose.rotation);
      if (pose.contains("scale")) read_vec3(pose["scale"], bone.local_pose.scale);
    }
    out.bones.push_back(std::move(bone));
  }
  return !out.bones.empty();
}

bool load_skin_bin(const fs::path& path, SkeletonInfo& out, std::string& error) {
  std::vector<uint8_t> bytes;
  if (!read_file_bytes(path, bytes)) {
    error = "failed to read skin.bin";
    return false;
  }
  if (bytes.size() < sizeof(uint32_t) * 4) {
    error = "skin.bin too small";
    return false;
  }
  const uint32_t* header = reinterpret_cast<const uint32_t*>(bytes.data());
  const uint32_t magic = header[0];
  const uint32_t version = header[1];
  const uint32_t joint_count = header[2];
  if (magic != 0x4E494B53 || version != 1) {
    error = "skin.bin header invalid";
    return false;
  }
  const size_t expected = sizeof(uint32_t) * 4 + static_cast<size_t>(joint_count) * sizeof(float) * 16;
  if (bytes.size() < expected) {
    error = "skin.bin truncated";
    return false;
  }
  out.inverse_bind_mats.clear();
  out.inverse_bind_mats.resize(joint_count);
  const float* mats = reinterpret_cast<const float*>(bytes.data() + sizeof(uint32_t) * 4);
  for (size_t i = 0; i < joint_count; ++i) {
    std::array<float, 16> mat{};
    std::memcpy(mat.data(), mats + (i * 16), sizeof(float) * 16);
    out.inverse_bind_mats[i] = mat;
  }
  return true;
}
#endif

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
  if (asset_dir.filename() == "manny") {
    record.name = "manny";
  }
  record.source_path = asset_doc.value("source_path", "");
  if (asset_doc.contains("mesh") && asset_doc["mesh"].is_object()) {
    const auto& mesh_node = asset_doc["mesh"];
    if (mesh_node.contains("scale") && mesh_node["scale"].is_array()) {
      const auto& arr = mesh_node["scale"];
      for (size_t i = 0; i < 3 && i < arr.size(); ++i) {
        record.mesh.mesh_scale[i] = arr[i].get<float>();
      }
    }
  }

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

  const fs::path skeleton_path = asset_dir / "skeleton.json";
  if (fs::exists(skeleton_path)) {
    std::string skel_error;
    if (!load_skeleton_json(skeleton_path, record.skeleton, skel_error)) {
      rkg::log::warn(std::string("asset_cache: skeleton.json parse failed: ") + skel_error);
    } else {
      const fs::path skin_path = asset_dir / "skin.bin";
      if (fs::exists(skin_path)) {
        std::string skin_error;
        if (!load_skin_bin(skin_path, record.skeleton, skin_error)) {
          rkg::log::warn(std::string("asset_cache: skin.bin parse failed: ") + skin_error);
        }
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
  fs::path repo_root;
  {
    fs::path p = content_root;
    for (int i = 0; i < 3 && p.has_parent_path(); ++i) {
      p = p.parent_path();
    }
    repo_root = p;
  }
  const fs::path generated_assets_dir = repo_root / "build" / "content_cache" / "generated_assets";

  fs::path manny_source;
  if (const char* home = std::getenv("HOME")) {
    manny_source = fs::path(home) / "rkg" / "NecTr" / "AI_GUARD_1" / "AI_GUARD_1_Character_output.glb";
  }
  if (manny_source.empty() || !fs::exists(manny_source)) {
    if (const char* home = std::getenv("HOME")) {
      manny_source = fs::path(home) / "rkg" / "NECESSARY TRANSFERS" / "manny mesh glb" / "testmanny.glb";
    }
  }
  if (manny_source.empty() || !fs::exists(manny_source)) {
    manny_source = content_root / "source_assets" / "manny" / "manny.glb";
  }
  const fs::path manny_asset_dir = generated_assets_dir / "manny";
  if (fs::exists(manny_source)) {
    rkg::log::info(std::string("asset_cache: manny source -> ") + manny_source.string());
    rkg::log::info("asset_cache: auto-importing manny into build/content_cache/generated_assets/manny");
    rkg::asset::ImportOptions options{};
    options.overwrite = true;
    options.write_textures = true;
    std::error_code ec;
    fs::create_directories(manny_asset_dir, ec);
    const auto result = rkg::asset::import_glb(manny_source, manny_asset_dir, options);
    if (!result.ok) {
      rkg::log::warn(std::string("asset_cache: auto-import manny failed: ") + result.error);
    } else {
      rkg::log::info("asset_cache: auto-imported manny.glb into build/content_cache/generated_assets/manny");
    }
  }
  auto load_dir_assets = [&](const fs::path& dir_root) {
    if (!fs::exists(dir_root) || !fs::is_directory(dir_root)) return;
    for (const auto& entry : fs::directory_iterator(dir_root)) {
      if (!entry.is_directory()) continue;
      std::string err;
      if (!load_asset_dir(entry.path(), err)) {
        rkg::log::warn(std::string("asset_cache: skip ") + entry.path().string() + ": " + err);
        continue;
      }
    }
  };
  load_dir_assets(assets_dir);
  load_dir_assets(generated_assets_dir);
  return true;
}

} // namespace rkg::runtime
