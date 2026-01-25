#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

#include "rkg/asset_import.h"
#include "rkg/json_write.h"
#include "rkg/paths.h"

#include <array>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "stb_image.h"

namespace fs = std::filesystem;

namespace rkg::asset {

namespace {

struct BinaryBlob {
  std::vector<uint8_t> bytes;
  std::string mime;
};

bool read_file_bytes(const fs::path& path, std::vector<uint8_t>& out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  in.seekg(0, std::ios::end);
  const auto size = in.tellg();
  if (size < 0) return false;
  out.resize(static_cast<size_t>(size));
  in.seekg(0, std::ios::beg);
  in.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(out.size()));
  return true;
}

bool write_file_bytes(const fs::path& path, const std::vector<uint8_t>& data) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path, std::ios::binary);
  if (!out) return false;
  out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
  return true;
}

bool starts_with(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

bool decode_base64(const std::string& input, std::vector<uint8_t>& out) {
  static const int kInvalid = -1;
  std::array<int, 256> table{};
  table.fill(kInvalid);
  const std::string alphabet =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  for (size_t i = 0; i < alphabet.size(); ++i) {
    table[static_cast<unsigned char>(alphabet[i])] = static_cast<int>(i);
  }
  int val = 0;
  int valb = -8;
  for (unsigned char c : input) {
    if (c == '=') break;
    int d = table[c];
    if (d == kInvalid) continue;
    val = (val << 6) + d;
    valb += 6;
    if (valb >= 0) {
      out.push_back(static_cast<uint8_t>((val >> valb) & 0xFF));
      valb -= 8;
    }
  }
  return !out.empty();
}

bool parse_data_uri(const std::string& uri, std::string& mime, std::vector<uint8_t>& data) {
  // data:[<mediatype>][;base64],<data>
  const std::string prefix = "data:";
  if (!starts_with(uri, prefix)) return false;
  const auto comma = uri.find(',');
  if (comma == std::string::npos) return false;
  std::string meta = uri.substr(prefix.size(), comma - prefix.size());
  std::string payload = uri.substr(comma + 1);
  bool base64 = false;
  if (meta.find(";base64") != std::string::npos) {
    base64 = true;
    meta.erase(meta.find(";base64"), 7);
  }
  mime = meta.empty() ? "application/octet-stream" : meta;
  if (base64) {
    return decode_base64(payload, data);
  }
  data.assign(payload.begin(), payload.end());
  return true;
}

std::string lower(const std::string& text) {
  std::string out = text;
  for (char& c : out) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return out;
}

std::string extension_from_mime(const std::string& mime) {
  if (mime == "image/png") return ".png";
  if (mime == "image/jpeg") return ".jpg";
  if (mime == "image/jpg") return ".jpg";
  if (mime == "image/webp") return ".webp";
  return ".bin";
}

std::string extension_from_uri(const std::string& uri) {
  const auto pos = uri.find_last_of('.');
  if (pos == std::string::npos) return "";
  std::string ext = uri.substr(pos);
  if (ext.size() > 8) return "";
  return lower(ext);
}

std::string make_name(const char* name, const std::string& fallback) {
  if (name && name[0] != '\0') return name;
  return fallback;
}

std::string format_indexed(const std::string& prefix, size_t index, const std::string& ext) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%s_%03zu%s", prefix.c_str(), index, ext.c_str());
  return std::string(buffer);
}

bool ensure_empty_or_overwrite(const fs::path& output_dir, bool overwrite, std::string& error) {
  if (fs::exists(output_dir)) {
    if (!overwrite) {
      error = "output directory exists (use --overwrite to replace)";
      return false;
    }
    std::error_code ec;
    fs::remove_all(output_dir, ec);
    if (ec) {
      error = "failed to remove output directory";
      return false;
    }
  }
  fs::create_directories(output_dir);
  return true;
}

void write_asset_json(const fs::path& path,
                      const std::string& name,
                      const fs::path& source,
                      const MeshPrimitiveInfo& mesh,
                      size_t material_count,
                      size_t texture_count,
                      size_t exported_primitives,
                      size_t total_primitives) {
  std::ofstream out(path);
  rkg::json::Writer writer(out);
  writer.begin_object();
  writer.key("name");
  writer.value(name);
  writer.key("type");
  writer.value("mesh");
  writer.key("source_path");
  writer.value(source.generic_string());
  writer.key("importer_version");
  writer.value("phase1");
  writer.key("deps");
  writer.begin_array();
  writer.end_array();
  writer.key("hash");
  writer.value("");
  writer.key("timestamp");
  writer.value(static_cast<int64_t>(0));

  writer.key("mesh");
  writer.begin_object();
  writer.key("file");
  writer.value("mesh.bin");
  writer.key("format");
  writer.value("rkg_mesh_bin_v1");
  writer.key("vertex_count");
  writer.value(static_cast<uint64_t>(mesh.vertex_count));
  writer.key("index_count");
  writer.value(static_cast<uint64_t>(mesh.index_count));
  writer.key("attributes");
  writer.begin_array();
  writer.value("POSITION");
  if (mesh.has_normals) writer.value("NORMAL");
  if (mesh.has_uv0) writer.value("TEXCOORD_0");
  if (mesh.has_tangents) writer.value("TANGENT");
  writer.end_array();
  writer.key("exported_primitives");
  writer.value(static_cast<uint64_t>(exported_primitives));
  writer.key("total_primitives");
  writer.value(static_cast<uint64_t>(total_primitives));
  writer.end_object();

  writer.key("materials");
  writer.begin_object();
  writer.key("file");
  writer.value("materials.json");
  writer.key("count");
  writer.value(static_cast<uint64_t>(material_count));
  writer.end_object();

  writer.key("textures");
  writer.begin_object();
  writer.key("dir");
  writer.value("textures");
  writer.key("count");
  writer.value(static_cast<uint64_t>(texture_count));
  writer.end_object();

  writer.end_object();
  writer.finish();
}

void write_materials_json(const fs::path& path,
                          const std::vector<MaterialInfo>& materials,
                          const std::vector<TextureInfo>& textures) {
  std::ofstream out(path);
  rkg::json::Writer writer(out);
  writer.begin_object();
  writer.key("materials");
  writer.begin_array();
  for (const auto& material : materials) {
    writer.begin_object();
    writer.key("name");
    writer.value(material.name);
    writer.key("baseColorFactor");
    writer.begin_array();
    writer.value(material.base_color[0]);
    writer.value(material.base_color[1]);
    writer.value(material.base_color[2]);
    writer.value(material.base_color[3]);
    writer.end_array();
    writer.key("baseColorTexture");
    if (material.base_color_texture >= 0 &&
        static_cast<size_t>(material.base_color_texture) < textures.size()) {
      writer.value(textures[static_cast<size_t>(material.base_color_texture)].file);
    } else {
      writer.null();
    }
    writer.end_object();
  }
  writer.end_array();
  writer.end_object();
  writer.finish();
}

}  // namespace

ImportResult import_glb(const fs::path& input_path,
                        const fs::path& output_dir,
                        const ImportOptions& options) {
  ImportResult result;
  if (!fs::exists(input_path)) {
    result.error = "input file not found";
    return result;
  }
  if (!ensure_empty_or_overwrite(output_dir, options.overwrite, result.error)) {
    return result;
  }

  cgltf_options cgltf_opts{};
  cgltf_data* data = nullptr;
  cgltf_result parse_result = cgltf_parse_file(&cgltf_opts, input_path.string().c_str(), &data);
  if (parse_result != cgltf_result_success) {
    result.error = "cgltf_parse_file failed";
    return result;
  }
  cgltf_result load_result = cgltf_load_buffers(&cgltf_opts, data, input_path.string().c_str());
  if (load_result != cgltf_result_success) {
    cgltf_free(data);
    result.error = "cgltf_load_buffers failed";
    return result;
  }
  if (cgltf_validate(data) != cgltf_result_success) {
    cgltf_free(data);
    result.error = "cgltf_validate failed";
    return result;
  }

  const std::string asset_name = input_path.stem().string();
  result.asset.name = asset_name;

  std::vector<TextureInfo> textures;
  textures.reserve(data->images_count);
  std::unordered_map<const cgltf_image*, int> image_to_index;

  const fs::path textures_dir = output_dir / "textures";
  if (options.write_textures) {
    fs::create_directories(textures_dir);
  }

  for (size_t i = 0; i < data->images_count; ++i) {
    const cgltf_image& image = data->images[i];
    TextureInfo info;
    info.name = make_name(image.name, format_indexed("image", i, ""));

    std::string mime = image.mime_type ? image.mime_type : "";
    std::string ext = extension_from_mime(mime);
    std::vector<uint8_t> bytes;
    bool image_ok = true;

    if (image.buffer_view) {
      const cgltf_buffer_view* view = image.buffer_view;
      const cgltf_buffer* buffer = view->buffer;
      const auto* data_ptr = reinterpret_cast<const uint8_t*>(buffer->data) + view->offset;
      bytes.assign(data_ptr, data_ptr + view->size);
    } else if (image.uri) {
      const std::string uri = image.uri;
      if (starts_with(uri, "data:")) {
        if (!parse_data_uri(uri, mime, bytes)) {
          image_ok = false;
        }
        ext = extension_from_mime(mime);
      } else {
        const fs::path src_path = input_path.parent_path() / uri;
        if (!read_file_bytes(src_path, bytes)) {
          image_ok = false;
        }
        if (ext == ".bin") {
          std::string uri_ext = extension_from_uri(uri);
          if (!uri_ext.empty()) ext = uri_ext;
        }
      }
    }

    if (!image_ok || bytes.empty()) {
      result.error = "failed to load image bytes";
      cgltf_free(data);
      return result;
    }

    const std::string filename = format_indexed("tex", i, ext);
    info.file = (fs::path("textures") / filename).generic_string();
    info.mime_type = mime.empty() ? "application/octet-stream" : mime;

    if (options.write_textures) {
      if (!write_file_bytes(textures_dir / filename, bytes)) {
        result.error = "failed to write texture bytes";
        cgltf_free(data);
        return result;
      }
    }

    if (!bytes.empty()) {
      int w = 0, h = 0, ch = 0;
      stbi_uc* decoded = stbi_load_from_memory(bytes.data(),
                                               static_cast<int>(bytes.size()),
                                               &w, &h, &ch, 0);
      if (decoded) {
        info.width = w;
        info.height = h;
        info.channels = ch;
        stbi_image_free(decoded);
      }
    }

    image_to_index[&image] = static_cast<int>(textures.size());
    textures.push_back(info);
  }

  std::vector<MaterialInfo> materials;
  materials.reserve(data->materials_count);
  for (size_t i = 0; i < data->materials_count; ++i) {
    const cgltf_material& material = data->materials[i];
    MaterialInfo info;
    info.name = make_name(material.name, format_indexed("material", i, ""));
    if (material.has_pbr_metallic_roughness) {
      const float* factor = material.pbr_metallic_roughness.base_color_factor;
      info.base_color[0] = factor[0];
      info.base_color[1] = factor[1];
      info.base_color[2] = factor[2];
      info.base_color[3] = factor[3];
      if (material.pbr_metallic_roughness.base_color_texture.texture &&
          material.pbr_metallic_roughness.base_color_texture.texture->image) {
        const cgltf_image* image = material.pbr_metallic_roughness.base_color_texture.texture->image;
        auto it = image_to_index.find(image);
        if (it != image_to_index.end()) {
          info.base_color_texture = it->second;
        }
      }
    }
    materials.push_back(info);
  }

  size_t total_primitives = 0;
  for (size_t m = 0; m < data->meshes_count; ++m) {
    total_primitives += data->meshes[m].primitives_count;
  }

  // Export the first mesh primitive deterministically.
  if (data->meshes_count == 0 || data->meshes[0].primitives_count == 0) {
    cgltf_free(data);
    result.error = "no mesh primitives found in glb";
    return result;
  }

  const cgltf_mesh& mesh = data->meshes[0];
  const cgltf_primitive& prim = mesh.primitives[0];

  cgltf_accessor* pos_accessor = nullptr;
  cgltf_accessor* norm_accessor = nullptr;
  cgltf_accessor* uv_accessor = nullptr;
  cgltf_accessor* tan_accessor = nullptr;
  for (size_t i = 0; i < prim.attributes_count; ++i) {
    const cgltf_attribute& attr = prim.attributes[i];
    if (attr.type == cgltf_attribute_type_position) pos_accessor = attr.data;
    if (attr.type == cgltf_attribute_type_normal) norm_accessor = attr.data;
    if (attr.type == cgltf_attribute_type_texcoord && attr.index == 0) uv_accessor = attr.data;
    if (attr.type == cgltf_attribute_type_tangent) tan_accessor = attr.data;
  }
  if (!pos_accessor) {
    cgltf_free(data);
    result.error = "mesh primitive missing POSITION";
    return result;
  }

  const size_t vertex_count = pos_accessor->count;
  std::vector<float> positions(vertex_count * 3, 0.0f);
  std::vector<float> normals(vertex_count * 3, 0.0f);
  std::vector<float> uvs(vertex_count * 2, 0.0f);
  std::vector<float> tangents(vertex_count * 4, 0.0f);

  for (size_t i = 0; i < vertex_count; ++i) {
    cgltf_accessor_read_float(pos_accessor, i, &positions[i * 3], 3);
  }
  if (norm_accessor) {
    for (size_t i = 0; i < vertex_count; ++i) {
      cgltf_accessor_read_float(norm_accessor, i, &normals[i * 3], 3);
    }
  }
  if (uv_accessor) {
    for (size_t i = 0; i < vertex_count; ++i) {
      cgltf_accessor_read_float(uv_accessor, i, &uvs[i * 2], 2);
    }
  }
  if (tan_accessor) {
    for (size_t i = 0; i < vertex_count; ++i) {
      cgltf_accessor_read_float(tan_accessor, i, &tangents[i * 4], 4);
    }
  }

  std::vector<uint32_t> indices;
  if (prim.indices) {
    indices.resize(prim.indices->count);
    for (size_t i = 0; i < prim.indices->count; ++i) {
      indices[i] = static_cast<uint32_t>(cgltf_accessor_read_index(prim.indices, i));
    }
  } else {
    indices.resize(vertex_count);
    for (size_t i = 0; i < vertex_count; ++i) {
      indices[i] = static_cast<uint32_t>(i);
    }
  }

  MeshPrimitiveInfo mesh_info;
  mesh_info.vertex_count = static_cast<uint32_t>(vertex_count);
  mesh_info.index_count = static_cast<uint32_t>(indices.size());
  mesh_info.has_normals = norm_accessor != nullptr;
  mesh_info.has_uv0 = uv_accessor != nullptr;
  mesh_info.has_tangents = tan_accessor != nullptr;

  if (!options.validate_only) {
    fs::create_directories(output_dir);
    std::ofstream mesh_out(output_dir / "mesh.bin", std::ios::binary);
    if (!mesh_out) {
      cgltf_free(data);
      result.error = "failed to open mesh.bin";
      return result;
    }

    const uint32_t magic = 0x30474B52; // 'RKG0'
    const uint32_t version = 1;
    uint32_t flags = 0;
    if (mesh_info.has_normals) flags |= 1;
    if (mesh_info.has_uv0) flags |= 2;
    if (mesh_info.has_tangents) flags |= 4;

    mesh_out.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    mesh_out.write(reinterpret_cast<const char*>(&version), sizeof(version));
    mesh_out.write(reinterpret_cast<const char*>(&mesh_info.vertex_count), sizeof(mesh_info.vertex_count));
    mesh_out.write(reinterpret_cast<const char*>(&mesh_info.index_count), sizeof(mesh_info.index_count));
    mesh_out.write(reinterpret_cast<const char*>(&flags), sizeof(flags));

    mesh_out.write(reinterpret_cast<const char*>(positions.data()),
                   static_cast<std::streamsize>(positions.size() * sizeof(float)));
    if (mesh_info.has_normals) {
      mesh_out.write(reinterpret_cast<const char*>(normals.data()),
                     static_cast<std::streamsize>(normals.size() * sizeof(float)));
    }
    if (mesh_info.has_uv0) {
      mesh_out.write(reinterpret_cast<const char*>(uvs.data()),
                     static_cast<std::streamsize>(uvs.size() * sizeof(float)));
    }
    if (mesh_info.has_tangents) {
      mesh_out.write(reinterpret_cast<const char*>(tangents.data()),
                     static_cast<std::streamsize>(tangents.size() * sizeof(float)));
    }
    mesh_out.write(reinterpret_cast<const char*>(indices.data()),
                   static_cast<std::streamsize>(indices.size() * sizeof(uint32_t)));

    write_asset_json(output_dir / "asset.json",
                     asset_name,
                     input_path,
                     mesh_info,
                     materials.size(),
                     textures.size(),
                     1,
                     total_primitives);

    write_materials_json(output_dir / "materials.json", materials, textures);
  }

  result.ok = true;
  result.asset.mesh_count = static_cast<uint32_t>(data->meshes_count);
  result.asset.primitive_count = static_cast<uint32_t>(mesh.primitives_count);
  result.asset.exported_primitives = 1;
  result.asset.total_primitives = static_cast<uint32_t>(total_primitives);
  result.asset.material_count = static_cast<uint32_t>(materials.size());
  result.asset.texture_count = static_cast<uint32_t>(textures.size());
  result.mesh = mesh_info;
  result.materials = materials;
  result.textures = textures;

  if (result.asset.exported_primitives < result.asset.total_primitives) {
    result.warning = "Phase 1 exports only the first mesh primitive";
  }

  cgltf_free(data);
  return result;
}

}  // namespace rkg::asset
