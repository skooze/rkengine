#include "rkg/config.h"
#include "rkg/log.h"
#include "rkg/paths.h"
#include "rkg/project.h"
#include "rkg_platform/file_watcher.h"
#include "rkg_data/database.h"
#include "rkg_data/serialization.h"
#include "rkgctl/cli_api.h"

#include <nlohmann/json.hpp>

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif

#if RKG_ENABLE_OPENAI
#include <curl/curl.h>
#endif

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fs = std::filesystem;
using json = nlohmann::json;

struct CliOptions {
  std::string plan_path;
  bool dry_run = true;
  bool require_approval = true;
  bool apply = false;
};

struct ActionResult {
  bool ok = false;
  std::string message;
  std::vector<std::string> files_touched;
};

struct LlmResult {
  std::string raw_json;
  std::string parsed_json;
  std::string error;
  int http_status = 0;
};

class ILlmProvider {
 public:
  virtual ~ILlmProvider() = default;
  virtual LlmResult GeneratePlan(const json& request) = 0;
};

struct ValidationError {
  fs::path file;
  std::string keypath;
  std::string message;
};

enum class ContentType {
  Prefab,
  Level
};

enum class PackEntryType : uint32_t {
  Prefab = 1,
  Level = 2
};

struct ContentAsset {
  ContentType type;
  std::string name;
  fs::path source_path;
  std::string logical_path;
  std::string id;
  std::string source_hash;
  uint64_t mtime_epoch = 0;
  std::vector<std::string> dependencies;
};

struct DependencyRef {
  fs::path file;
  std::string keypath;
  std::string prefab;
};

struct PackEntry {
  uint64_t id = 0;
  uint32_t type = 0;
  uint64_t offset = 0;
  uint64_t size = 0;
  std::string path;
  std::string payload;
};

void write_u16(std::ofstream& out, uint16_t value);
void write_u32(std::ofstream& out, uint32_t value);
void write_u64(std::ofstream& out, uint64_t value);
uint16_t read_u16(std::ifstream& in);
uint32_t read_u32(std::ifstream& in);
uint64_t read_u64(std::ifstream& in);


std::string now_iso() {
  const auto now = std::chrono::system_clock::now();
  const auto tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &tt);
#else
  localtime_r(&tt, &tm);
#endif
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
  return std::string(buf);
}

uint64_t fnv1a_64(const std::string& data) {
  uint64_t hash = 14695981039346656037ull;
  for (unsigned char c : data) {
    hash ^= static_cast<uint64_t>(c);
    hash *= 1099511628211ull;
  }
  return hash;
}

uint32_t crc32(const std::string& data) {
  uint32_t crc = 0xFFFFFFFFu;
  for (unsigned char c : data) {
    crc ^= static_cast<uint32_t>(c);
    for (int i = 0; i < 8; ++i) {
      const uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

std::string to_hex(uint64_t value) {
  std::ostringstream oss;
  oss << std::hex << value;
  return oss.str();
}

uint64_t file_mtime_epoch(const fs::path& path) {
  std::error_code ec;
  auto ftime = fs::last_write_time(path, ec);
  if (ec) return 0;
  auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
      ftime - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::seconds>(sctp.time_since_epoch()).count());
}

std::string read_file_string(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string get_env(const char* name) {
  const char* value = std::getenv(name);
  return value ? std::string(value) : std::string();
}

std::string normalize_base_url(std::string base) {
  if (base.empty()) {
    base = "https://api.openai.com";
  }
  while (!base.empty() && base.back() == '/') {
    base.pop_back();
  }
  return base;
}

bool load_text_file(const fs::path& path, std::string& out, std::string& error) {
  if (!fs::exists(path)) {
    error = "file missing";
    return false;
  }
  out = read_file_string(path);
  if (out.empty() && fs::file_size(path) > 0) {
    error = "file read failed";
    return false;
  }
  return true;
}

std::string render_template(const std::string& text,
                            const std::unordered_map<std::string, std::string>& vars) {
  std::string out = text;
  for (const auto& kv : vars) {
    const std::string token = "{{" + kv.first + "}}";
    size_t pos = 0;
    while ((pos = out.find(token, pos)) != std::string::npos) {
      out.replace(pos, token.size(), kv.second);
      pos += kv.second.size();
    }
  }
  return out;
}

bool load_json_file(const fs::path& path, json& out);
bool copy_tree(const fs::path& src, const fs::path& dst);

bool is_safe_rel_path(const fs::path& path) {
  if (path.is_absolute()) return false;
  for (const auto& part : path) {
    if (part == "..") return false;
  }
  if (path.empty()) return false;
  const auto top = (*path.begin()).string();
  static const std::unordered_set<std::string> kAllowed = {
      "content", "config", "docs", "examples", "build", "projects"};
  return kAllowed.count(top) > 0;
}

std::string hash_params(const json& params) {
  const auto dumped = params.dump();
  const auto hash = std::hash<std::string>{}(dumped);
  std::ostringstream oss;
  oss << std::hex << hash;
  return oss.str();
}

void append_audit_line(const fs::path& audit_path, const json& record) {
  fs::create_directories(audit_path.parent_path());
  std::ofstream out(audit_path, std::ios::app);
  out << record.dump() << "\n";
}

bool confirm_actions() {
  std::cout << "Approval required. Proceed? (y/N): ";
  std::string line;
  std::getline(std::cin, line);
  return !line.empty() && (line[0] == 'y' || line[0] == 'Y');
}

#if RKG_ENABLE_DATA_YAML
struct KeyToken {
  enum class Type { Key, Index } type;
  std::string key;
  int index = 0;
};

std::vector<KeyToken> parse_keypath(const std::string& keypath) {
  std::vector<KeyToken> tokens;
  size_t i = 0;
  while (i < keypath.size()) {
    if (keypath[i] == '.') {
      ++i;
      continue;
    }
    if (keypath[i] == '[') {
      ++i;
      std::string num;
      while (i < keypath.size() && keypath[i] != ']') {
        num.push_back(keypath[i++]);
      }
      if (i < keypath.size() && keypath[i] == ']') ++i;
      tokens.push_back({KeyToken::Type::Index, "", std::stoi(num)});
      continue;
    }
    std::string key;
    while (i < keypath.size() && keypath[i] != '.' && keypath[i] != '[') {
      key.push_back(keypath[i++]);
    }
    if (!key.empty()) {
      tokens.push_back({KeyToken::Type::Key, key, 0});
    }
  }
  return tokens;
}

YAML::Node yaml_from_json(const json& value) {
  if (value.is_object()) {
    YAML::Node node(YAML::NodeType::Map);
    for (auto it = value.begin(); it != value.end(); ++it) {
      node[it.key()] = yaml_from_json(it.value());
    }
    return node;
  }
  if (value.is_array()) {
    YAML::Node node(YAML::NodeType::Sequence);
    for (const auto& v : value) {
      node.push_back(yaml_from_json(v));
    }
    return node;
  }
  if (value.is_boolean()) return YAML::Node(value.get<bool>());
  if (value.is_number_float()) return YAML::Node(value.get<double>());
  if (value.is_number_integer()) return YAML::Node(value.get<int64_t>());
  if (value.is_string()) return YAML::Node(value.get<std::string>());
  return YAML::Node();
}

bool update_yaml_keypath(YAML::Node& root, const std::string& keypath, const json& value) {
  auto tokens = parse_keypath(keypath);
  YAML::Node current = root;
  for (size_t i = 0; i < tokens.size(); ++i) {
    const bool last = (i == tokens.size() - 1);
    const auto& token = tokens[i];
    if (token.type == KeyToken::Type::Key) {
      if (last) {
        current[token.key] = yaml_from_json(value);
      } else {
        if (!current[token.key]) {
          current[token.key] = YAML::Node();
        }
        current = current[token.key];
      }
    } else {
      if (!current.IsSequence()) {
        current = YAML::Node(YAML::NodeType::Sequence);
      }
      while (current.size() <= static_cast<size_t>(token.index)) {
        current.push_back(YAML::Node());
      }
      if (last) {
        current[token.index] = yaml_from_json(value);
      } else {
        current = current[token.index];
      }
    }
  }
  return true;
}

std::string emit_yaml(const YAML::Node& node) {
  YAML::Emitter emitter;
  emitter << node;
  return std::string(emitter.c_str());
}
#endif

bool parse_int64(const std::string& value, int64_t& out) {
  const char* begin = value.data();
  const char* end = value.data() + value.size();
  if (begin == end) return false;
  auto result = std::from_chars(begin, end, out);
  return result.ec == std::errc() && result.ptr == end;
}

bool parse_double(const std::string& value, double& out) {
  char* end = nullptr;
  out = std::strtod(value.c_str(), &end);
  return end && *end == '\0';
}

#if RKG_ENABLE_DATA_YAML
json yaml_to_json_sorted(const YAML::Node& node) {
  if (node.IsNull()) {
    return nullptr;
  }
  if (node.IsScalar()) {
    const std::string scalar = node.as<std::string>();
    if (scalar == "true") return true;
    if (scalar == "false") return false;
    int64_t as_int = 0;
    if (parse_int64(scalar, as_int)) {
      return as_int;
    }
    double as_double = 0.0;
    if (parse_double(scalar, as_double)) {
      return as_double;
    }
    return scalar;
  }
  if (node.IsSequence()) {
    json arr = json::array();
    for (const auto& item : node) {
      arr.push_back(yaml_to_json_sorted(item));
    }
    return arr;
  }
  if (node.IsMap()) {
    std::vector<std::string> keys;
    keys.reserve(node.size());
    for (const auto& pair : node) {
      keys.push_back(pair.first.as<std::string>());
    }
    std::sort(keys.begin(), keys.end());
    json obj = json::object();
    for (const auto& key : keys) {
      obj[key] = yaml_to_json_sorted(node[key]);
    }
    return obj;
  }
  return nullptr;
}
#endif

json canonicalize_json(const json& input) {
  if (input.is_object()) {
    std::vector<std::string> keys;
    keys.reserve(input.size());
    for (auto it = input.begin(); it != input.end(); ++it) {
      keys.push_back(it.key());
    }
    std::sort(keys.begin(), keys.end());
    json obj = json::object();
    for (const auto& key : keys) {
      obj[key] = canonicalize_json(input.at(key));
    }
    return obj;
  }
  if (input.is_array()) {
    json arr = json::array();
    for (const auto& item : input) {
      arr.push_back(canonicalize_json(item));
    }
    return arr;
  }
  return input;
}

bool load_canonical_json(const fs::path& path, std::string& out, std::string& error) {
  if (!fs::exists(path)) {
    error = "file missing";
    return false;
  }
  const auto ext = path.extension().string();
  if (ext == ".json") {
    std::ifstream in(path);
    if (!in) {
      error = "json open failed";
      return false;
    }
    json doc;
    in >> doc;
    out = canonicalize_json(doc).dump();
    return true;
  }
  if (ext == ".yaml" || ext == ".yml") {
#if RKG_ENABLE_DATA_YAML
    YAML::Node doc = YAML::LoadFile(path.string());
    out = yaml_to_json_sorted(doc).dump();
    return true;
#else
    error = "yaml disabled";
    return false;
#endif
  }
  error = "unsupported extension";
  return false;
}

void add_error(std::vector<ValidationError>& errors,
               const fs::path& file,
               const std::string& keypath,
               const std::string& message) {
  errors.push_back({file, keypath, message});
}

void add_warning(const fs::path& file,
                 const std::string& keypath,
                 const std::string& message) {
  std::cerr << file.string() << ":" << keypath << " -> warning: " << message << "\n";
}

bool is_known_mesh_id(const std::string& mesh) {
  return mesh == "cube" || mesh == "quad";
}

bool collect_content_files(const fs::path& root, std::vector<fs::path>& out_files) {
  if (!fs::exists(root)) {
    return false;
  }
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file()) continue;
    const auto ext = entry.path().extension().string();
    if (ext == ".yaml" || ext == ".yml" || ext == ".json") {
      out_files.push_back(entry.path());
    }
  }
  return true;
}

std::string logical_path_from(const fs::path& file, const fs::path& content_root) {
  std::error_code ec;
  auto rel = fs::relative(file, content_root, ec);
  if (ec) {
    return file.filename().string();
  }
  return rel.generic_string();
}

bool parse_prefab_json(const fs::path& path,
                       ContentAsset& out,
                       std::vector<ValidationError>& errors) {
  json doc;
  if (!rkg::data::load_json_file(path, doc)) {
    add_error(errors, path, "", "failed to load JSON");
    return false;
  }
  if (!doc.contains("name") || !doc["name"].is_string()) {
    add_error(errors, path, "name", "prefab.name must be a string");
  } else {
    out.name = doc["name"].get<std::string>();
  }
  if (!doc.contains("components")) {
    add_error(errors, path, "components", "prefab.components is required");
  } else if (!(doc["components"].is_object() || doc["components"].is_array())) {
    add_error(errors, path, "components", "prefab.components must be map or list");
  }
  if (doc.contains("tags") && !doc["tags"].is_array()) {
    add_error(errors, path, "tags", "prefab.tags must be a list");
  }
  if (doc.contains("components") && doc["components"].is_object()) {
    const auto& comps = doc["components"];
    if (comps.contains("Renderable")) {
      const auto& rend = comps["Renderable"];
      if (!rend.is_object()) {
        add_error(errors, path, "components.Renderable", "Renderable must be a map");
      } else {
        if (rend.contains("mesh")) {
          if (!rend["mesh"].is_string()) {
            add_error(errors, path, "components.Renderable.mesh", "mesh must be a string");
          } else if (!is_known_mesh_id(rend["mesh"].get<std::string>())) {
            add_warning(path, "components.Renderable.mesh", "unknown mesh id (expected cube or quad)");
          }
        }
        if (rend.contains("color")) {
          if (!rend["color"].is_array()) {
            add_error(errors, path, "components.Renderable.color", "color must be a list");
          } else if (rend["color"].size() < 3 || rend["color"].size() > 4) {
            add_error(errors, path, "components.Renderable.color", "color must have 3 or 4 numbers");
          }
        }
      }
    }
  }
  return errors.empty();
}

bool parse_level_json(const fs::path& path,
                      ContentAsset& out,
                      std::vector<DependencyRef>& deps,
                      std::vector<ValidationError>& errors) {
  json doc;
  if (!rkg::data::load_json_file(path, doc)) {
    add_error(errors, path, "", "failed to load JSON");
    return false;
  }
  if (doc.contains("name") && doc["name"].is_string()) {
    out.name = doc["name"].get<std::string>();
  }
  if (!doc.contains("entities") || !doc["entities"].is_array()) {
    add_error(errors, path, "entities", "level.entities must be a list");
    return errors.empty();
  }
  for (size_t i = 0; i < doc["entities"].size(); ++i) {
    const auto& ent = doc["entities"][i];
    const std::string base = "entities[" + std::to_string(i) + "]";
    if (!ent.contains("name") || !ent["name"].is_string()) {
      add_error(errors, path, base + ".name", "entity.name must be a string");
    }
    std::string prefab_ref;
    if (ent.contains("prefab") && ent["prefab"].is_string()) {
      prefab_ref = ent["prefab"].get<std::string>();
    } else if (ent.contains("prefab_ref") && ent["prefab_ref"].is_string()) {
      prefab_ref = ent["prefab_ref"].get<std::string>();
    } else {
      add_error(errors, path, base + ".prefab", "entity.prefab must be a string");
    }
    if (!prefab_ref.empty()) {
      deps.push_back({path, base + ".prefab", prefab_ref});
      out.dependencies.push_back(prefab_ref);
    }
    if (ent.contains("transform") && !ent["transform"].is_object()) {
      add_error(errors, path, base + ".transform", "entity.transform must be a map");
    }
    if (ent.contains("renderable")) {
      const auto& rend = ent["renderable"];
      if (!rend.is_object()) {
        add_error(errors, path, base + ".renderable", "entity.renderable must be a map");
      } else {
        if (rend.contains("mesh")) {
          if (!rend["mesh"].is_string()) {
            add_error(errors, path, base + ".renderable.mesh", "mesh must be a string");
          } else if (!is_known_mesh_id(rend["mesh"].get<std::string>())) {
            add_warning(path, base + ".renderable.mesh", "unknown mesh id (expected cube or quad)");
          }
        }
        if (rend.contains("color")) {
          if (!rend["color"].is_array()) {
            add_error(errors, path, base + ".renderable.color", "color must be a list");
          } else if (rend["color"].size() < 3 || rend["color"].size() > 4) {
            add_error(errors, path, base + ".renderable.color", "color must have 3 or 4 numbers");
          }
        }
      }
    }
  }
  return errors.empty();
}

#if RKG_ENABLE_DATA_YAML
bool parse_prefab_yaml(const fs::path& path,
                       ContentAsset& out,
                       std::vector<ValidationError>& errors) {
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(path.string());
  } catch (const std::exception& e) {
    add_error(errors, path, "", std::string("YAML parse failed: ") + e.what());
    return false;
  }
  if (!doc["name"] || !doc["name"].IsScalar()) {
    add_error(errors, path, "name", "prefab.name must be a string");
  } else {
    out.name = doc["name"].as<std::string>();
  }
  if (!doc["components"]) {
    add_error(errors, path, "components", "prefab.components is required");
  } else if (!(doc["components"].IsMap() || doc["components"].IsSequence())) {
    add_error(errors, path, "components", "prefab.components must be map or list");
  }
  if (doc["tags"] && !doc["tags"].IsSequence()) {
    add_error(errors, path, "tags", "prefab.tags must be a list");
  }
  if (doc["components"] && doc["components"].IsMap()) {
    const auto comps = doc["components"];
    if (comps["Renderable"]) {
      const auto rend = comps["Renderable"];
      if (!rend.IsMap()) {
        add_error(errors, path, "components.Renderable", "Renderable must be a map");
      } else {
        if (rend["mesh"]) {
          if (!rend["mesh"].IsScalar()) {
            add_error(errors, path, "components.Renderable.mesh", "mesh must be a string");
          } else if (!is_known_mesh_id(rend["mesh"].as<std::string>())) {
            add_warning(path, "components.Renderable.mesh", "unknown mesh id (expected cube or quad)");
          }
        }
        if (rend["color"]) {
          if (!rend["color"].IsSequence()) {
            add_error(errors, path, "components.Renderable.color", "color must be a list");
          } else if (rend["color"].size() < 3 || rend["color"].size() > 4) {
            add_error(errors, path, "components.Renderable.color", "color must have 3 or 4 numbers");
          }
        }
      }
    }
  }
  return errors.empty();
}

bool parse_level_yaml(const fs::path& path,
                      ContentAsset& out,
                      std::vector<DependencyRef>& deps,
                      std::vector<ValidationError>& errors) {
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(path.string());
  } catch (const std::exception& e) {
    add_error(errors, path, "", std::string("YAML parse failed: ") + e.what());
    return false;
  }
  if (doc["name"] && doc["name"].IsScalar()) {
    out.name = doc["name"].as<std::string>();
  }
  if (!doc["entities"] || !doc["entities"].IsSequence()) {
    add_error(errors, path, "entities", "level.entities must be a list");
    return errors.empty();
  }
  const auto entities = doc["entities"];
  for (size_t i = 0; i < entities.size(); ++i) {
    const auto ent = entities[i];
    const std::string base = "entities[" + std::to_string(i) + "]";
    if (!ent["name"] || !ent["name"].IsScalar()) {
      add_error(errors, path, base + ".name", "entity.name must be a string");
    }
    std::string prefab_ref;
    if (ent["prefab"] && ent["prefab"].IsScalar()) {
      prefab_ref = ent["prefab"].as<std::string>();
    } else if (ent["prefab_ref"] && ent["prefab_ref"].IsScalar()) {
      prefab_ref = ent["prefab_ref"].as<std::string>();
    } else {
      add_error(errors, path, base + ".prefab", "entity.prefab must be a string");
    }
    if (!prefab_ref.empty()) {
      deps.push_back({path, base + ".prefab", prefab_ref});
      out.dependencies.push_back(prefab_ref);
    }
    if (ent["transform"] && !ent["transform"].IsMap()) {
      add_error(errors, path, base + ".transform", "entity.transform must be a map");
    }
    if (ent["renderable"]) {
      const auto rend = ent["renderable"];
      if (!rend.IsMap()) {
        add_error(errors, path, base + ".renderable", "entity.renderable must be a map");
      } else {
        if (rend["mesh"]) {
          if (!rend["mesh"].IsScalar()) {
            add_error(errors, path, base + ".renderable.mesh", "mesh must be a string");
          } else if (!is_known_mesh_id(rend["mesh"].as<std::string>())) {
            add_warning(path, base + ".renderable.mesh", "unknown mesh id (expected cube or quad)");
          }
        }
        if (rend["color"]) {
          if (!rend["color"].IsSequence()) {
            add_error(errors, path, base + ".renderable.color", "color must be a list");
          } else if (rend["color"].size() < 3 || rend["color"].size() > 4) {
            add_error(errors, path, base + ".renderable.color", "color must have 3 or 4 numbers");
          }
        }
      }
    }
  }
  return errors.empty();
}
#endif

bool gather_assets(const fs::path& content_root,
                   std::vector<ContentAsset>& assets,
                   std::vector<DependencyRef>& deps,
                   std::vector<ValidationError>& errors) {
  std::vector<fs::path> prefab_files;
  std::vector<fs::path> level_files;
  collect_content_files(content_root / "prefabs", prefab_files);
  collect_content_files(content_root / "levels", level_files);

  for (const auto& file : prefab_files) {
    ContentAsset asset;
    asset.type = ContentType::Prefab;
    asset.source_path = file;
    asset.logical_path = logical_path_from(file, content_root);
    const auto ext = file.extension().string();
    if (ext == ".json") {
      parse_prefab_json(file, asset, errors);
#if RKG_ENABLE_DATA_YAML
    } else {
      parse_prefab_yaml(file, asset, errors);
#endif
    }
    const auto contents = read_file_string(file);
    asset.source_hash = to_hex(fnv1a_64(contents));
    asset.id = to_hex(fnv1a_64(asset.logical_path));
    asset.mtime_epoch = file_mtime_epoch(file);
    assets.push_back(asset);
  }

  for (const auto& file : level_files) {
    ContentAsset asset;
    asset.type = ContentType::Level;
    asset.source_path = file;
    asset.logical_path = logical_path_from(file, content_root);
    const auto ext = file.extension().string();
    if (ext == ".json") {
      parse_level_json(file, asset, deps, errors);
#if RKG_ENABLE_DATA_YAML
    } else {
      parse_level_yaml(file, asset, deps, errors);
#endif
    }
    const auto contents = read_file_string(file);
    asset.source_hash = to_hex(fnv1a_64(contents));
    asset.id = to_hex(fnv1a_64(asset.logical_path));
    asset.mtime_epoch = file_mtime_epoch(file);
    assets.push_back(asset);
  }

  return true;
}

bool validate_dependencies(const std::vector<ContentAsset>& assets,
                           const std::vector<DependencyRef>& deps,
                           std::vector<ValidationError>& errors) {
  std::unordered_set<std::string> prefabs;
  for (const auto& asset : assets) {
    if (asset.type == ContentType::Prefab && !asset.name.empty()) {
      prefabs.insert(asset.name);
    }
  }
  for (const auto& dep : deps) {
    if (prefabs.find(dep.prefab) == prefabs.end()) {
      add_error(errors, dep.file, dep.keypath, "missing prefab reference: " + dep.prefab);
    }
  }
  return errors.empty();
}

uint32_t pack_type_for(ContentType type) {
  switch (type) {
    case ContentType::Prefab:
      return static_cast<uint32_t>(PackEntryType::Prefab);
    case ContentType::Level:
      return static_cast<uint32_t>(PackEntryType::Level);
  }
  return 0;
}

bool build_pack_entries(const fs::path& content_root,
                        const std::vector<ContentAsset>& assets,
                        std::vector<PackEntry>& out_entries,
                        std::string& error) {
  std::vector<ContentAsset> sorted = assets;
  std::sort(sorted.begin(), sorted.end(),
            [](const ContentAsset& a, const ContentAsset& b) { return a.logical_path < b.logical_path; });

  out_entries.clear();
  for (const auto& asset : sorted) {
    const fs::path src = content_root / asset.logical_path;
    std::string payload;
    std::string load_error;
    if (!load_canonical_json(src, payload, load_error)) {
      error = std::string("pack build failed for ") + src.string() + ": " + load_error;
      return false;
    }
    PackEntry entry;
    entry.id = fnv1a_64(asset.logical_path);
    entry.type = pack_type_for(asset.type);
    entry.path = asset.logical_path;
    entry.payload = std::move(payload);
    entry.size = entry.payload.size();
    out_entries.push_back(std::move(entry));
  }
  return true;
}

bool write_pack_file(const fs::path& pack_path, std::vector<PackEntry>& entries, std::string& error) {
  const uint32_t version = 1;
  const uint32_t count = static_cast<uint32_t>(entries.size());
  const uint64_t header_size = 4 + 4 + 4 + 8;
  uint64_t table_size = 0;
  for (const auto& entry : entries) {
    table_size += 8 + 4 + 8 + 8 + 2 + static_cast<uint64_t>(entry.path.size());
  }
  uint64_t data_offset = header_size + table_size;
  uint64_t cursor = data_offset;
  for (auto& entry : entries) {
    entry.offset = cursor;
    cursor += entry.size;
  }

  std::ofstream out(pack_path, std::ios::binary);
  if (!out) {
    error = "pack open failed";
    return false;
  }

  out.write("RKGP", 4);
  write_u32(out, version);
  write_u32(out, count);
  write_u64(out, header_size);

  for (const auto& entry : entries) {
    write_u64(out, entry.id);
    write_u32(out, entry.type);
    write_u64(out, entry.offset);
    write_u64(out, entry.size);
    if (entry.path.size() > 0xFFFFu) {
      error = "path too long";
      return false;
    }
    write_u16(out, static_cast<uint16_t>(entry.path.size()));
    out.write(entry.path.data(), static_cast<std::streamsize>(entry.path.size()));
  }

  for (const auto& entry : entries) {
    out.write(entry.payload.data(), static_cast<std::streamsize>(entry.payload.size()));
  }

  return true;
}

struct PackFileEntry {
  uint64_t id = 0;
  uint32_t type = 0;
  uint64_t offset = 0;
  uint64_t size = 0;
  std::string path;
};

bool read_pack_entries(const fs::path& pack_path, std::vector<PackFileEntry>& entries, std::string& error) {
  std::ifstream in(pack_path, std::ios::binary);
  if (!in) {
    error = "pack open failed";
    return false;
  }
  char magic[4] = {};
  in.read(magic, 4);
  if (std::string(magic, 4) != "RKGP") {
    error = "pack magic mismatch";
    return false;
  }
  const uint32_t version = read_u32(in);
  if (version != 1) {
    error = "unsupported pack version";
    return false;
  }
  const uint32_t count = read_u32(in);
  const uint64_t index_offset = read_u64(in);
  in.seekg(static_cast<std::streamoff>(index_offset), std::ios::beg);
  entries.clear();
  for (uint32_t i = 0; i < count; ++i) {
    PackFileEntry entry;
    entry.id = read_u64(in);
    entry.type = read_u32(in);
    entry.offset = read_u64(in);
    entry.size = read_u64(in);
    const uint16_t path_len = read_u16(in);
    if (path_len > 0) {
      std::string path(path_len, '\0');
      in.read(path.data(), path_len);
      entry.path = path;
    }
    entries.push_back(std::move(entry));
  }
  return true;
}

bool read_pack_entry_data(const fs::path& pack_path,
                          const PackFileEntry& entry,
                          std::string& out) {
  std::ifstream in(pack_path, std::ios::binary);
  if (!in) return false;
  in.seekg(static_cast<std::streamoff>(entry.offset), std::ios::beg);
  out.resize(static_cast<size_t>(entry.size));
  in.read(out.data(), static_cast<std::streamsize>(entry.size));
  return true;
}

const char* pack_type_name(uint32_t type) {
  switch (type) {
    case static_cast<uint32_t>(PackEntryType::Prefab):
      return "prefab";
    case static_cast<uint32_t>(PackEntryType::Level):
      return "level";
    default:
      return "unknown";
  }
}

bool parse_u64_hex(const std::string& text, uint64_t& out) {
  if (text.empty()) return false;
  std::string s = text;
  if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) {
    s = s.substr(2);
  }
  if (s.empty()) return false;
  char* end = nullptr;
  out = std::strtoull(s.c_str(), &end, 16);
  return end && *end == '\0';
}

int content_dump_pack(const fs::path& pack_path, const std::optional<std::string>& extract) {
  std::vector<PackFileEntry> entries;
  std::string error;
  if (!read_pack_entries(pack_path, entries, error)) {
    std::cerr << "pack read failed: " << error << "\n";
    return 1;
  }

  if (!extract.has_value()) {
    std::cout << "Entries: " << entries.size() << "\n";
    for (const auto& entry : entries) {
      std::cout << "  0x" << std::hex << entry.id << std::dec
                << " [" << pack_type_name(entry.type) << "] "
                << entry.size << " bytes " << entry.path << "\n";
    }
    return 0;
  }

  uint64_t target_id = 0;
  const std::string target = extract.value();
  const bool id_ok = parse_u64_hex(target, target_id);
  for (const auto& entry : entries) {
    if ((id_ok && entry.id == target_id) || (!id_ok && entry.path == target)) {
      std::string payload;
      if (!read_pack_entry_data(pack_path, entry, payload)) {
        std::cerr << "failed to read entry data\n";
        return 1;
      }
      std::cout << payload;
      return 0;
    }
  }

  std::cerr << "entry not found: " << target << "\n";
  return 1;
}

void print_validation_errors(const std::vector<ValidationError>& errors) {
  for (const auto& err : errors) {
    std::cerr << err.file.string();
    if (!err.keypath.empty()) {
      std::cerr << ":" << err.keypath;
    }
    std::cerr << " -> " << err.message << "\n";
  }
}

std::vector<std::string> split_lines(const std::string& text) {
  std::vector<std::string> lines;
  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    lines.push_back(line);
  }
  if (!text.empty() && text.back() == '\n') {
    lines.push_back("");
  }
  return lines;
}

std::string unified_diff(const std::string& old_text,
                         const std::string& new_text,
                         const std::string& path) {
  const auto old_lines = split_lines(old_text);
  const auto new_lines = split_lines(new_text);
  const size_t n = old_lines.size();
  const size_t m = new_lines.size();

  std::vector<std::vector<int>> lcs(n + 1, std::vector<int>(m + 1, 0));
  for (size_t i = n; i-- > 0;) {
    for (size_t j = m; j-- > 0;) {
      if (old_lines[i] == new_lines[j]) {
        lcs[i][j] = lcs[i + 1][j + 1] + 1;
      } else {
        lcs[i][j] = std::max(lcs[i + 1][j], lcs[i][j + 1]);
      }
    }
  }

  std::ostringstream diff;
  diff << "--- a/" << path << "\n";
  diff << "+++ b/" << path << "\n";
  diff << "@@ -" << 1 << "," << n << " +" << 1 << "," << m << " @@\n";

  size_t i = 0;
  size_t j = 0;
  while (i < n || j < m) {
    if (i < n && j < m && old_lines[i] == new_lines[j]) {
      diff << " " << old_lines[i] << "\n";
      ++i;
      ++j;
    } else if (j < m && (i == n || lcs[i][j + 1] >= lcs[i + 1][j])) {
      diff << "+" << new_lines[j] << "\n";
      ++j;
    } else if (i < n) {
      diff << "-" << old_lines[i] << "\n";
      ++i;
    }
  }
  return diff.str();
}

std::string sanitize_filename(const fs::path& path) {
  std::string name = path.generic_string();
  for (auto& c : name) {
    if (c == '/' || c == '\\' || c == ':') c = '_';
  }
  return name;
}

void write_u16(std::ofstream& out, uint16_t value) {
  out.put(static_cast<char>(value & 0xFF));
  out.put(static_cast<char>((value >> 8) & 0xFF));
}

void write_u32(std::ofstream& out, uint32_t value) {
  out.put(static_cast<char>(value & 0xFF));
  out.put(static_cast<char>((value >> 8) & 0xFF));
  out.put(static_cast<char>((value >> 16) & 0xFF));
  out.put(static_cast<char>((value >> 24) & 0xFF));
}

void write_u64(std::ofstream& out, uint64_t value) {
  out.put(static_cast<char>(value & 0xFF));
  out.put(static_cast<char>((value >> 8) & 0xFF));
  out.put(static_cast<char>((value >> 16) & 0xFF));
  out.put(static_cast<char>((value >> 24) & 0xFF));
  out.put(static_cast<char>((value >> 32) & 0xFF));
  out.put(static_cast<char>((value >> 40) & 0xFF));
  out.put(static_cast<char>((value >> 48) & 0xFF));
  out.put(static_cast<char>((value >> 56) & 0xFF));
}

uint16_t read_u16(std::ifstream& in) {
  uint16_t v = 0;
  v |= static_cast<uint16_t>(in.get() & 0xFF);
  v |= static_cast<uint16_t>((in.get() & 0xFF) << 8);
  return v;
}

uint32_t read_u32(std::ifstream& in) {
  uint32_t v = 0;
  v |= static_cast<uint32_t>(in.get() & 0xFF);
  v |= static_cast<uint32_t>((in.get() & 0xFF) << 8);
  v |= static_cast<uint32_t>((in.get() & 0xFF) << 16);
  v |= static_cast<uint32_t>((in.get() & 0xFF) << 24);
  return v;
}

uint64_t read_u64(std::ifstream& in) {
  uint64_t v = 0;
  v |= static_cast<uint64_t>(in.get() & 0xFF);
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 8;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 16;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 24;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 32;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 40;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 48;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 56;
  return v;
}

bool load_base_if_missing(const fs::path& path,
                          std::unordered_map<std::string, std::string>& base_contents,
                          std::unordered_map<std::string, std::string>& current_contents) {
  const auto key = path.generic_string();
  if (base_contents.find(key) == base_contents.end()) {
    base_contents[key] = read_file_string(path);
  }
  if (current_contents.find(key) == current_contents.end()) {
    current_contents[key] = base_contents[key];
  }
  return true;
}

bool apply_action_to_content(const json& action,
                             std::unordered_map<std::string, std::string>& base_contents,
                             std::unordered_map<std::string, std::string>& current_contents,
                             std::vector<std::string>& errors) {
  const auto type = action.value("action", "");
  if (type == "record_audit") {
    return false;
  }

  fs::path path;
  if (type == "write_text_file") {
    path = fs::path(action.value("path", ""));
  } else if (type == "update_yaml") {
    path = fs::path(action.value("path", ""));
  } else if (type == "create_prefab") {
    path = fs::path(action.value("output_path", ""));
  } else if (type == "add_level_entity") {
    path = fs::path(action.value("level_path", ""));
  } else {
    errors.push_back("unknown action: " + type);
    return false;
  }

  if (!is_safe_rel_path(path)) {
    errors.push_back("unsafe path: " + path.string());
    return false;
  }

  load_base_if_missing(path, base_contents, current_contents);
  const auto key = path.generic_string();
  auto& current = current_contents[key];

  if (type == "write_text_file") {
    current = action.value("contents", "");
    return true;
  }

  if (type == "update_yaml") {
#if !RKG_ENABLE_DATA_YAML
    errors.push_back("yaml disabled");
    return false;
#else
    YAML::Node root;
    if (!current.empty()) {
      root = YAML::Load(current);
    } else {
      root = YAML::Node(YAML::NodeType::Map);
    }
    const auto keypath = action.value("keypath", "");
    const auto value = action.contains("value") ? action["value"] : json();
    update_yaml_keypath(root, keypath, value);
    current = emit_yaml(root);
    return true;
#endif
  }

  if (type == "create_prefab") {
#if !RKG_ENABLE_DATA_YAML
    errors.push_back("yaml disabled");
    return false;
#else
    YAML::Node prefab;
    prefab["name"] = action.value("name", "prefab");
    const auto components = action.contains("components") ? action["components"] : json::object();
    prefab["components"] = yaml_from_json(components);
    current = emit_yaml(prefab);
    return true;
#endif
  }

  if (type == "add_level_entity") {
#if !RKG_ENABLE_DATA_YAML
    errors.push_back("yaml disabled");
    return false;
#else
    YAML::Node level;
    if (!current.empty()) {
      level = YAML::Load(current);
    } else {
      level = YAML::Node(YAML::NodeType::Map);
    }
    YAML::Node entity;
    entity["name"] = action.value("name", "entity");
    entity["prefab"] = action.value("prefab_ref", "");
    const auto transform = action.contains("transform") ? action["transform"] : json::object();
    entity["transform"] = yaml_from_json(transform);
    level["entities"].push_back(entity);
    current = emit_yaml(level);
    return true;
#endif
  }

  return false;
}

std::string make_run_id() {
  std::string id = now_iso();
  for (auto& c : id) {
    if (c == ':' || c == 'T') c = '_';
  }
  return id;
}

StageResult stage_plan(const json& plan, const fs::path& plan_path) {
  StageResult result;
  const std::string run_id = make_run_id();
  const fs::path staging_root = fs::path("build") / "ai_staging";
  result.run_dir = staging_root / run_id;
  const fs::path patches_dir = result.run_dir / "patches";
  const fs::path base_dir = result.run_dir / "base_files";
  const fs::path proposed_dir = result.run_dir / "proposed_files";

  fs::create_directories(patches_dir);
  fs::create_directories(base_dir);
  fs::create_directories(proposed_dir);

  std::unordered_map<std::string, std::string> base_contents;
  std::unordered_map<std::string, std::string> current_contents;

  if (plan.contains("tasks")) {
    for (const auto& task : plan["tasks"]) {
      if (!task.contains("actions") || !task["actions"].is_array()) {
        continue;
      }
      for (const auto& action : task["actions"]) {
        if (!apply_action_to_content(action, base_contents, current_contents, result.errors)) {
          continue;
        }
      }
    }
  }

  if (!result.errors.empty()) {
    return result;
  }

  int index = 0;
  for (const auto& kv : current_contents) {
    const fs::path path = kv.first;
    const auto base = base_contents[kv.first];
    const auto proposed = kv.second;
    if (base == proposed) {
      continue;
    }

    StagedFile staged;
    staged.path = path;
    staged.base_hash = to_hex(fnv1a_64(base));
    staged.new_hash = to_hex(fnv1a_64(proposed));

    const std::string sanitized = sanitize_filename(path);
    const std::string diff_name = std::to_string(index++) + "_" + sanitized + ".diff";
    staged.diff_path = patches_dir / diff_name;

    staged.base_path = base_dir / path;
    staged.proposed_path = proposed_dir / path;
    fs::create_directories(staged.base_path.parent_path());
    fs::create_directories(staged.proposed_path.parent_path());

    rkg::data::write_text_file(staged.base_path, base);
    rkg::data::write_text_file(staged.proposed_path, proposed);

    const auto diff_text = unified_diff(base, proposed, path.generic_string());
    rkg::data::write_text_file(staged.diff_path, diff_text);

    result.files.push_back(staged);
  }

  result.summary["run_id"] = run_id;
  result.summary["plan_path"] = plan_path.generic_string();
  result.summary["created_at"] = now_iso();
  result.summary["files"] = json::array();
  for (const auto& file : result.files) {
    json entry;
    entry["path"] = file.path.generic_string();
    entry["base_hash"] = file.base_hash;
    entry["new_hash"] = file.new_hash;
    entry["diff_path"] = file.diff_path.generic_string();
    entry["base_path"] = file.base_path.generic_string();
    entry["proposed_path"] = file.proposed_path.generic_string();
    result.summary["files"].push_back(entry);
  }

  rkg::data::write_text_file(result.run_dir / "summary.json", result.summary.dump(2));
  fs::create_directories(staging_root);
  rkg::data::write_text_file(staging_root / "last_staged.json", result.summary.dump(2));
  return result;
}

#if RKG_ENABLE_DATA_YAML
bool validate_override_vec(const YAML::Node& node, size_t min_size, size_t max_size) {
  if (!node || !node.IsSequence()) return false;
  const size_t size = node.size();
  return size >= min_size && size <= max_size;
}

bool apply_override_to_entity(YAML::Node& entity,
                              const YAML::Node& override_node,
                              std::vector<std::string>& errors,
                              std::vector<std::string>& warnings) {
  if (override_node["transform"]) {
    const auto t = override_node["transform"];
    if (!t.IsMap()) {
      errors.push_back("override transform must be a map");
      return false;
    }
    YAML::Node out = entity["transform"];
    if (!out || !out.IsMap()) out = YAML::Node(YAML::NodeType::Map);
    if (t["position"]) {
      if (!validate_override_vec(t["position"], 3, 3)) {
        errors.push_back("override transform.position must have 3 values");
        return false;
      }
      out["position"] = t["position"];
    }
    if (t["rotation"]) {
      if (!validate_override_vec(t["rotation"], 3, 3)) {
        errors.push_back("override transform.rotation must have 3 values");
        return false;
      }
      out["rotation"] = t["rotation"];
    }
    if (t["scale"]) {
      if (!validate_override_vec(t["scale"], 3, 3)) {
        errors.push_back("override transform.scale must have 3 values");
        return false;
      }
      out["scale"] = t["scale"];
    }
    entity["transform"] = out;
  }

  if (override_node["renderable"]) {
    const auto r = override_node["renderable"];
    if (!r.IsMap()) {
      errors.push_back("override renderable must be a map");
      return false;
    }
    YAML::Node out = entity["renderable"];
    if (!out || !out.IsMap()) out = YAML::Node(YAML::NodeType::Map);
    if (r["mesh"]) {
      if (!r["mesh"].IsScalar()) {
        errors.push_back("override renderable.mesh must be a string");
        return false;
      }
      const std::string mesh = r["mesh"].as<std::string>();
      if (!is_known_mesh_id(mesh)) {
        warnings.push_back("unknown mesh id: " + mesh);
      }
      out["mesh"] = mesh;
    }
    if (r["color"]) {
      if (!validate_override_vec(r["color"], 3, 4)) {
        errors.push_back("override renderable.color must have 3 or 4 values");
        return false;
      }
      out["color"] = r["color"];
    }
    entity["renderable"] = out;
  }

  return true;
}
#endif

StageResult stage_commit_overrides(const fs::path& root,
                                   const fs::path& project_root,
                                   const fs::path& overrides_path,
                                   const std::optional<fs::path>& level_override,
                                   const std::optional<std::string>& entity_id,
                                   const std::optional<std::string>& entity_name,
                                   const fs::path& staging_dir,
                                   const std::string& run_id,
                                   std::string& error_code,
                                   std::string& error_message) {
  StageResult result;
  result.run_dir = staging_dir;
  error_code.clear();
  error_message.clear();

#if !RKG_ENABLE_DATA_YAML
  error_code = "yaml_disabled";
  error_message = "YAML support is disabled";
  result.errors.push_back(error_message);
  return result;
#else
  if (!fs::exists(overrides_path)) {
    error_code = "overrides_missing";
    error_message = "overrides file not found: " + overrides_path.string();
    result.errors.push_back(error_message);
    return result;
  }

  const fs::path project_config_path = project_root / "project.yaml";
  const rkg::ProjectConfig cfg = rkg::load_project_config(project_config_path);
  fs::path level_rel;
  if (level_override.has_value() && !level_override->empty()) {
    level_rel = level_override.value();
  } else {
    level_rel = fs::path(cfg.initial_level);
  }
  if (level_rel.empty()) {
    error_code = "level_missing";
    error_message = "project initial_level missing";
    result.errors.push_back(error_message);
    return result;
  }

  if (level_rel.is_absolute()) {
    error_code = "level_path_invalid";
    error_message = "initial_level must be relative";
    result.errors.push_back(error_message);
    return result;
  }
  const fs::path level_abs = project_root / level_rel;
  if (!fs::exists(level_abs)) {
    error_code = "level_missing";
    error_message = "level not found: " + level_abs.string();
    result.errors.push_back(error_message);
    return result;
  }

  std::error_code rel_ec;
  const fs::path level_rel_to_root = fs::relative(level_abs, root, rel_ec);
  if (rel_ec || !is_safe_rel_path(level_rel_to_root)) {
    error_code = "level_path_invalid";
    error_message = "level path is not under repo root";
    result.errors.push_back(error_message);
    return result;
  }

  const std::string base = read_file_string(level_abs);
  YAML::Node level;
  try {
    level = YAML::Load(base);
  } catch (const std::exception& e) {
    error_code = "level_invalid";
    error_message = std::string("level parse failed: ") + e.what();
    result.errors.push_back(error_message);
    return result;
  }
  if (!level || !level.IsMap()) {
    level = YAML::Node(YAML::NodeType::Map);
  }
  if (!level["entities"] || !level["entities"].IsSequence()) {
    error_code = "entities_missing";
    error_message = "level.entities missing or invalid";
    result.errors.push_back(error_message);
    return result;
  }

  YAML::Node overrides_doc;
  try {
    overrides_doc = YAML::LoadFile(overrides_path.string());
  } catch (const std::exception& e) {
    error_code = "overrides_invalid";
    error_message = std::string("overrides parse failed: ") + e.what();
    result.errors.push_back(error_message);
    return result;
  }
  if (!overrides_doc["overrides"] || !overrides_doc["overrides"].IsMap()) {
    error_code = "overrides_invalid";
    error_message = "overrides map missing";
    result.errors.push_back(error_message);
    return result;
  }

  std::string selector_type = "all";
  std::string selector_value;
  std::string selector_warning;
  bool use_name_selector = false;
  if (entity_id.has_value() && !entity_id->empty()) {
    selector_type = "id";
    selector_value = entity_id.value();
  } else if (entity_name.has_value() && !entity_name->empty()) {
    selector_type = "name";
    selector_value = entity_name.value();
    selector_warning = "Staging by name is less reliable; add id to level YAML for stability.";
    use_name_selector = true;
  }

  std::vector<std::pair<std::string, YAML::Node>> override_entries;
  if (selector_type != "all") {
    const auto node = overrides_doc["overrides"][selector_value];
    if (!node) {
      error_code = "override_missing";
      error_message = "override entry missing: " + selector_value;
      result.errors.push_back(error_message);
      return result;
    }
    override_entries.emplace_back(selector_value, node);
  } else {
    for (const auto& item : overrides_doc["overrides"]) {
      override_entries.emplace_back(item.first.as<std::string>(), item.second);
    }
  }

  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  YAML::Node entities = level["entities"];
  std::string matched_name;
  for (const auto& item : override_entries) {
    const std::string key = item.first;
    const YAML::Node override_node = item.second;
    bool matched = false;
    for (std::size_t i = 0; i < entities.size(); ++i) {
      YAML::Node entity = entities[i];
      if (!entity.IsMap()) {
        continue;
      }
      std::string id;
      if (entity["id"] && entity["id"].IsScalar()) {
        id = entity["id"].as<std::string>();
      }
      std::string name;
      if (entity["name"] && entity["name"].IsScalar()) {
        name = entity["name"].as<std::string>();
      }
      bool match = false;
      if (selector_type == "id") {
        match = (!id.empty() && id == key);
      } else if (selector_type == "name") {
        match = (!name.empty() && name == key);
      } else {
        match = (!id.empty() && id == key) || (id.empty() && !name.empty() && name == key);
      }
      if (match) {
        matched = true;
        if (matched_name.empty()) {
          matched_name = name;
        }
        if (!apply_override_to_entity(entity, override_node, errors, warnings)) {
          break;
        }
        entities[i] = entity;
        break;
      }
    }
    if (!matched) {
      errors.push_back("override target missing: " + key);
    }
  }

  for (const auto& warn : warnings) {
    std::cerr << "warning: " << warn << "\n";
  }
  if (!errors.empty()) {
    error_code = "override_apply_failed";
    error_message = errors.front();
    result.errors = errors;
    return result;
  }

  level["entities"] = entities;
  const std::string proposed = emit_yaml(level);
  if (base == proposed) {
    result.summary["run_id"] = run_id;
    result.summary["created_at"] = now_iso();
    result.summary["overrides_path"] = overrides_path.generic_string();
    result.summary["level_path"] = level_rel_to_root.generic_string();
    result.summary["entity_id"] = entity_id.value_or("");
    result.summary["entity_name"] = matched_name;
    result.summary["selector_type"] = selector_type;
    result.summary["selector_value"] = selector_value;
    result.summary["selector_warning"] = selector_warning;
    result.summary["files"] = json::array();
    fs::create_directories(staging_dir);
    rkg::data::write_text_file(staging_dir / "summary.json", result.summary.dump(2));
    return result;
  }

  const fs::path patches_dir = staging_dir / "patches";
  const fs::path base_dir = staging_dir / "base_files";
  const fs::path proposed_dir = staging_dir / "proposed_files";

  fs::create_directories(patches_dir);
  fs::create_directories(base_dir);
  fs::create_directories(proposed_dir);

  StagedFile staged;
  staged.path = level_rel_to_root;
  staged.base_hash = to_hex(fnv1a_64(base));
  staged.new_hash = to_hex(fnv1a_64(proposed));
  const std::string sanitized = sanitize_filename(level_rel_to_root);
  staged.diff_path = patches_dir / ("0_" + sanitized + ".diff");
  staged.base_path = base_dir / level_rel_to_root;
  staged.proposed_path = proposed_dir / level_rel_to_root;
  fs::create_directories(staged.base_path.parent_path());
  fs::create_directories(staged.proposed_path.parent_path());
  rkg::data::write_text_file(staged.base_path, base);
  rkg::data::write_text_file(staged.proposed_path, proposed);
  const auto diff_text = unified_diff(base, proposed, level_rel_to_root.generic_string());
  rkg::data::write_text_file(staged.diff_path, diff_text);
  result.files.push_back(staged);

  result.summary["run_id"] = run_id;
  result.summary["created_at"] = now_iso();
  result.summary["overrides_path"] = overrides_path.generic_string();
  result.summary["level_path"] = level_rel_to_root.generic_string();
  result.summary["entity_id"] = entity_id.value_or("");
  result.summary["entity_name"] = matched_name;
  result.summary["selector_type"] = selector_type;
  result.summary["selector_value"] = selector_value;
  result.summary["selector_warning"] = selector_warning;
  result.summary["files"] = json::array();
  for (const auto& file : result.files) {
    json entry;
    entry["path"] = file.path.generic_string();
    entry["base_hash"] = file.base_hash;
    entry["new_hash"] = file.new_hash;
    entry["diff_path"] = file.diff_path.generic_string();
    entry["base_path"] = file.base_path.generic_string();
    entry["proposed_path"] = file.proposed_path.generic_string();
    result.summary["files"].push_back(entry);
  }
  rkg::data::write_text_file(staging_dir / "summary.json", result.summary.dump(2));
  return result;
#endif
}

bool is_safe_run_dir(const fs::path& root, const fs::path& run_dir) {
  std::error_code ec;
  const auto rel = fs::relative(run_dir, root, ec);
  if (ec || rel.empty()) return false;
  auto it = rel.begin();
  if (it == rel.end() || (*it).string() != "build") return false;
  ++it;
  if (it == rel.end() || (*it).string() != "ai_runs") return false;
  return true;
}

bool load_commit_stage_summary(const fs::path& run_dir, StageResult& out, std::string& error) {
  const fs::path summary_path = run_dir / "staged_patches" / "summary.json";
  json summary;
  if (!load_json_file(summary_path, summary)) {
    error = "staged summary missing";
    return false;
  }
  out = StageResult{};
  out.run_dir = run_dir / "staged_patches";
  out.summary = summary;
  if (summary.contains("files") && summary["files"].is_array()) {
    for (const auto& file : summary["files"]) {
      StagedFile staged;
      staged.path = fs::path(file.value("path", ""));
      staged.base_hash = file.value("base_hash", "");
      staged.new_hash = file.value("new_hash", "");
      staged.diff_path = fs::path(file.value("diff_path", ""));
      staged.base_path = fs::path(file.value("base_path", ""));
      staged.proposed_path = fs::path(file.value("proposed_path", ""));
      out.files.push_back(staged);
    }
  }
  return true;
}

struct SelectorInfo {
  std::string entity_id;
  std::string selector_type;
  std::string selector_value;
  std::string selector_warning;
};

SelectorInfo selector_from_summary(const json& summary) {
  SelectorInfo info;
  info.entity_id = summary.value("entity_id", "");
  info.selector_type = summary.value("selector_type", "");
  info.selector_value = summary.value("selector_value", "");
  info.selector_warning = summary.value("selector_warning", "");
  return info;
}

SelectorInfo selector_from_opts(const CommitOverridesOptions& opts) {
  SelectorInfo info;
  if (opts.entity_id.has_value() && !opts.entity_id->empty()) {
    info.entity_id = opts.entity_id.value();
    info.selector_type = "id";
    info.selector_value = opts.entity_id.value();
  } else if (opts.entity_name.has_value() && !opts.entity_name->empty()) {
    info.selector_type = "name";
    info.selector_value = opts.entity_name.value();
    info.selector_warning = "Staging by name is less reliable; add id to level YAML for stability.";
  } else {
    info.selector_type = "all";
  }
  return info;
}

void write_commit_results(const fs::path& run_dir,
                          const std::string& run_id,
                          const ApplyOptions& opts,
                          int rc,
                          size_t applied,
                          size_t conflicts,
                          const std::vector<std::string>& conflict_files,
                          const std::string& stage_name,
                          const std::string& error_code,
                          const std::string& error_message,
                          const fs::path& staging_dir,
                          bool forced_apply,
                          bool conflict_detected,
                          const std::string& entity_id,
                          const std::string& selector_type,
                          const std::string& selector_value,
                          const std::string& selector_warning,
                          bool snapshots_taken,
                          const std::string& snapshot_manifest_path) {
  json results;
  results["run_id"] = run_id;
  results["status"] = result_status_from_rc(rc);
  results["dry_run"] = opts.dry_run;
  results["applied"] = applied;
  results["conflicts"] = conflicts;
  results["conflict_files"] = conflict_files;
  results["staging_dir"] = staging_dir.generic_string();
  results["success"] = (rc == 0);
  results["stage"] = stage_name;
  results["forced_apply"] = forced_apply;
  results["conflict_detected"] = conflict_detected;
  results["entity_id"] = entity_id;
  results["selector_type"] = selector_type;
  results["selector_value"] = selector_value;
  results["selector_warning"] = selector_warning;
  results["snapshots_taken"] = snapshots_taken;
  results["snapshot_manifest_path"] = snapshot_manifest_path;
  results["error_code"] = error_code;
  results["error_message"] = error_message;
  write_json_file(run_dir / "results.json", results);
}

int apply_commit_overrides(const StageResult& stage,
                           const ApplyOptions& opts,
                           const fs::path& run_dir,
                           const std::string& run_id,
                           std::string& error_code,
                           std::string& error_message,
                           std::string& stage_name) {
  stage_name = "apply";
  error_code.clear();
  error_message.clear();
  const SelectorInfo selector = selector_from_summary(stage.summary);
  bool snapshots_taken = false;
  fs::path snapshot_manifest;

  if (stage.files.empty()) {
    write_commit_results(run_dir, run_id, opts, 0, 0, 0, {}, stage_name, "", "", stage.run_dir,
                         opts.force, false, selector.entity_id, selector.selector_type,
                         selector.selector_value, selector.selector_warning, false, "");
    return 0;
  }

  std::cout << "Staged " << stage.files.size() << " file(s) in " << stage.run_dir.string() << "\n";
  for (const auto& file : stage.files) {
    const auto diff = read_file_string(file.diff_path);
    std::cout << diff << "\n";
  }

  if (opts.require_approval && !opts.dry_run) {
    if (!confirm_actions()) {
      error_code = "approval_denied";
      error_message = "approval denied";
      write_commit_results(run_dir, run_id, opts, 1, 0, 0, {}, stage_name, error_code, error_message,
                           stage.run_dir, opts.force, false, selector.entity_id,
                           selector.selector_type, selector.selector_value, selector.selector_warning,
                           false, "");
      return 1;
    }
  }

  size_t applied = 0;
  size_t conflicts = 0;
  std::vector<std::string> conflict_files;
  bool conflict_detected = false;

  if (opts.force) {
    const fs::path snapshots_dir = run_dir / "snapshots";
    std::error_code snap_ec;
    fs::create_directories(snapshots_dir, snap_ec);
    if (snap_ec) {
      error_code = "snapshot_failed";
      error_message = "snapshot dir create failed";
      write_commit_results(run_dir, run_id, opts, 1, 0, 0, {}, stage_name, error_code, error_message,
                           stage.run_dir, opts.force, false, selector.entity_id,
                           selector.selector_type, selector.selector_value, selector.selector_warning,
                           false, "");
      return 1;
    }
    json manifest;
    manifest["created_at"] = now_iso();
    manifest["files"] = json::array();
    for (const auto& file : stage.files) {
      const fs::path target = file.path;
      std::error_code exists_ec;
      const bool exists = fs::exists(target, exists_ec);
      if (exists_ec) {
        error_code = "snapshot_failed";
        error_message = "snapshot stat failed: " + target.generic_string();
        write_commit_results(run_dir, run_id, opts, 1, 0, 0, {}, stage_name, error_code, error_message,
                             stage.run_dir, opts.force, false, selector.entity_id,
                             selector.selector_type, selector.selector_value, selector.selector_warning,
                             false, "");
        return 1;
      }
      const std::string contents = exists ? read_file_string(target) : std::string{};
      fs::path snapshot_path = snapshots_dir / target;
      snapshot_path += ".before";
      fs::create_directories(snapshot_path.parent_path(), snap_ec);
      if (snap_ec) {
        error_code = "snapshot_failed";
        error_message = "snapshot dir create failed: " + snapshot_path.parent_path().generic_string();
        write_commit_results(run_dir, run_id, opts, 1, 0, 0, {}, stage_name, error_code, error_message,
                             stage.run_dir, opts.force, false, selector.entity_id,
                             selector.selector_type, selector.selector_value, selector.selector_warning,
                             false, "");
        return 1;
      }
      if (!rkg::data::write_text_file(snapshot_path, contents)) {
        error_code = "snapshot_failed";
        error_message = "snapshot write failed: " + snapshot_path.generic_string();
        write_commit_results(run_dir, run_id, opts, 1, 0, 0, {}, stage_name, error_code, error_message,
                             stage.run_dir, opts.force, false, selector.entity_id,
                             selector.selector_type, selector.selector_value, selector.selector_warning,
                             false, "");
        return 1;
      }
      json entry;
      entry["path"] = target.generic_string();
      entry["size"] = static_cast<uint64_t>(contents.size());
      entry["hash"] = to_hex(fnv1a_64(contents));
      entry["timestamp"] = now_iso();
      entry["missing"] = !exists;
      entry["snapshot_path"] = snapshot_path.generic_string();
      manifest["files"].push_back(entry);
    }
    snapshot_manifest = snapshots_dir / "manifest.json";
    if (!write_json_file(snapshot_manifest, manifest)) {
      error_code = "snapshot_failed";
      error_message = "snapshot manifest write failed";
      write_commit_results(run_dir, run_id, opts, 1, 0, 0, {}, stage_name, error_code, error_message,
                           stage.run_dir, opts.force, false, selector.entity_id,
                           selector.selector_type, selector.selector_value, selector.selector_warning,
                           false, "");
      return 1;
    }
    snapshots_taken = true;
  }

  for (const auto& file : stage.files) {
    const auto current_contents = read_file_string(file.path);
    const auto current_hash = to_hex(fnv1a_64(current_contents));
    const bool hash_match = (current_hash == file.base_hash);
    if (!hash_match) {
      conflict_detected = true;
      if (!opts.force) {
        ++conflicts;
        conflict_files.push_back(file.path.string());
        continue;
      }
    }

    if (!opts.dry_run) {
      fs::create_directories(file.path.parent_path());
      rkg::data::write_text_file(file.path, read_file_string(file.proposed_path));
    }
    ++applied;

    json audit;
    audit["time"] = now_iso();
    audit["action_type"] = "commit_overrides";
    audit["params_hash"] = file.new_hash;
    audit["result"] = hash_match ? "ok" : "forced";
    audit["message"] = file.path.string();
    audit["files_touched"] = {file.path.string()};
    append_audit_line("build/ai_audit.log", audit);
  }

  int rc = 0;
  if (conflicts > 0) {
    rc = 2;
    error_code = "conflict";
    error_message = "content changed since overrides were staged";
    stage_name = "apply";
  }

  write_commit_results(run_dir, run_id, opts, rc, applied, conflicts, conflict_files,
                       stage_name, error_code, error_message, stage.run_dir,
                       opts.force, conflict_detected, selector.entity_id,
                       selector.selector_type, selector.selector_value, selector.selector_warning,
                       snapshots_taken, snapshot_manifest.generic_string());

  if (conflicts > 0) {
    std::cerr << "Conflicts detected; apply aborted for those files.\n";
    return rc;
  }

  return rc;
}

int apply_staged(const StageResult& stage, const ApplyOptions& opts) {
  if (stage.files.empty()) {
    std::cout << "No file changes proposed.\n";
    return 0;
  }

  std::cout << "Staged " << stage.files.size() << " file(s) in " << stage.run_dir.string() << "\n";
  for (const auto& file : stage.files) {
    const auto diff = read_file_string(file.diff_path);
    std::cout << diff << "\n";
  }

  if (opts.require_approval && !opts.dry_run) {
    if (!confirm_actions()) {
      rkg::log::warn("approval denied");
      return 1;
    }
  }

  size_t applied = 0;
  size_t conflicts = 0;
  std::vector<std::string> conflict_files;

  for (const auto& file : stage.files) {
    const auto current_contents = read_file_string(file.path);
    const auto current_hash = to_hex(fnv1a_64(current_contents));
    const bool hash_match = (current_hash == file.base_hash);
    if (!hash_match && !opts.force) {
      ++conflicts;
      conflict_files.push_back(file.path.string());
      continue;
    }

    if (!opts.dry_run) {
      fs::create_directories(file.path.parent_path());
      rkg::data::write_text_file(file.path, read_file_string(file.proposed_path));
    }
    ++applied;

    json audit;
    audit["time"] = now_iso();
    audit["action_type"] = "apply_patch";
    audit["params_hash"] = file.new_hash;
    audit["result"] = hash_match ? "ok" : "forced";
    audit["message"] = file.path.string();
    audit["files_touched"] = {file.path.string()};
    append_audit_line("build/ai_audit.log", audit);
  }

  json results;
  results["run_id"] = stage.summary.value("run_id", "");
  results["staging_dir"] = stage.run_dir.generic_string();
  results["applied"] = applied;
  results["conflicts"] = conflicts;
  results["dry_run"] = opts.dry_run;
  results["conflict_files"] = conflict_files;
  fs::create_directories("build");
  rkg::data::write_text_file("build/ai_results.json", results.dump(2));

  if (!opts.dry_run && conflicts == 0) {
    rkg::data::write_text_file("build/ai_staging/last_applied.json", stage.summary.dump(2));
  }

  if (conflicts > 0) {
    std::cerr << "Conflicts detected; apply aborted for those files.\n";
    return 2;
  }

  return 0;
}

int apply_plan(const json& plan, const fs::path& plan_path, const ApplyOptions& opts) {
  const StageResult stage = stage_plan(plan, plan_path);
  if (!stage.errors.empty()) {
    for (const auto& err : stage.errors) {
      std::cerr << "stage error: " << err << "\n";
    }
    return 1;
  }
  return apply_staged(stage, opts);
}

bool load_json_file(const fs::path& path, json& out) {
  if (!fs::exists(path)) {
    return false;
  }
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  in >> out;
  return true;
}

bool load_run_summary(const fs::path& run_dir, json& out, std::string& error) {
  json run_info;
  if (!load_json_file(run_dir / "run_info.json", run_info)) {
    error = "run_info.json missing";
    return false;
  }
  json results;
  load_json_file(run_dir / "results.json", results);

  out = json::object();
  out["run_id"] = run_info.value("run_id", run_dir.filename().string());
  out["created_at"] = run_info.value("created_at", "");
  out["goal"] = run_info.value("goal", "");
  out["mode"] = run_info.value("mode", "");
  out["plan_path"] = run_info.value("plan_path", (run_dir / "plan.json").generic_string());
  out["provider"] = run_info.value("provider", "");
  out["model"] = run_info.value("model", "");
  out["base_url"] = run_info.value("base_url", "");
  out["templates_dir"] = run_info.value("templates_dir", "");
  out["timeout_seconds"] = run_info.value("timeout_seconds", 0);
  out["strict_enums"] = run_info.value("strict_enums", true);
  out["config_sources"] = run_info.value("config_sources", json::object());
  if (!results.is_null() && !results.empty()) {
    out["results"] = results;
    if (results.contains("context_drift")) {
      out["context_drift"] = results["context_drift"];
    }
  }
  return true;
}

int plan_validate(const fs::path& plan_path) {
  json plan;
  if (!load_json_file(plan_path, plan)) {
    std::cerr << "plan not found: " << plan_path.string() << "\n";
    return 1;
  }
  std::vector<PlanValidationError> errors;
  if (!validate_plan_detailed(plan, errors, true)) {
    print_plan_errors(errors);
    return 1;
  }
  std::cout << "plan valid\n";
  return 0;
}

int plan_summarize(const fs::path& plan_path) {
  json plan;
  if (!load_json_file(plan_path, plan)) {
    std::cerr << "plan not found: " << plan_path.string() << "\n";
    return 1;
  }
  if (!plan.contains("tasks") || !plan["tasks"].is_array()) {
    std::cerr << "plan missing tasks\n";
    return 1;
  }
  std::cout << "Tasks: " << plan["tasks"].size() << "\n";
  for (const auto& task : plan["tasks"]) {
    std::cout << "- " << task.value("id", "") << " [" << task.value("type", "") << "] "
              << task.value("title", "") << "\n";
  }
  return 0;
}

int plan_schema_dump(const fs::path& root, const fs::path& out_path) {
  const fs::path schema_path = root / "docs" / "schemas" / "rkg_plan.schema.json";
  if (!fs::exists(schema_path)) {
    std::cerr << "schema not found: " << schema_path.string() << "\n";
    return 1;
  }
  const auto contents = read_file_string(schema_path);
  if (contents.empty()) {
    std::cerr << "schema read failed: " << schema_path.string() << "\n";
    return 1;
  }
  if (out_path.empty()) {
    std::cout << contents << "\n";
  } else {
    fs::create_directories(out_path.parent_path());
    rkg::data::write_text_file(out_path, contents);
    std::cout << "schema -> " << out_path.string() << "\n";
  }
  return 0;
}

int apply_command(const fs::path& plan_path, const ApplyOptions& opts) {
  json plan;
  if (!load_json_file(plan_path, plan)) {
    std::cerr << "plan not found: " << plan_path.string() << "\n";
    return 1;
  }
  std::vector<PlanValidationError> errors;
  if (!validate_plan_detailed(plan, errors, true)) {
    print_plan_errors(errors);
    return 1;
  }
  return apply_plan(plan, plan_path, opts);
}

int status_command() {
  const fs::path staging_root = fs::path("build") / "ai_staging";
  json staged;
  json applied;
  if (load_json_file(staging_root / "last_staged.json", staged)) {
    std::cout << "Last staged: " << staged.value("run_id", "") << "\n";
  } else {
    std::cout << "Last staged: none\n";
  }
  if (load_json_file(staging_root / "last_applied.json", applied)) {
    std::cout << "Last applied: " << applied.value("run_id", "") << "\n";
  } else {
    std::cout << "Last applied: none\n";
  }
  return 0;
}

int agent_status_command(const char* argv0,
                         const std::optional<fs::path>& project_override,
                         int runs,
                         bool json_out) {
  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const auto cfg = rkg::load_project_config(paths.project / "project.yaml");
  AiConfigResolved effective_cfg = resolve_ai_config(cfg, "", "", "", "", std::nullopt);

  std::string provider_note;
  const bool have_api_key = !get_env("OPENAI_API_KEY").empty();
#if !RKG_ENABLE_OPENAI
  const bool openai_enabled = false;
#else
  const bool openai_enabled = true;
#endif
  if (effective_cfg.values.provider == "openai" && (!openai_enabled || !have_api_key)) {
    provider_note = "fallback: openai unavailable";
    effective_cfg.values.provider = "offline";
    effective_cfg.provider_source = "fallback";
  }

  if (runs <= 0) runs = 5;
  const fs::path runs_root = paths.root / "build" / "ai_runs";
  std::vector<fs::path> run_dirs;
  if (fs::exists(runs_root)) {
    for (const auto& entry : fs::directory_iterator(runs_root)) {
      if (entry.is_directory()) {
        run_dirs.push_back(entry.path());
      }
    }
  }
  std::sort(run_dirs.begin(), run_dirs.end(),
            [](const fs::path& a, const fs::path& b) { return a.filename().string() > b.filename().string(); });

  json runs_json = json::array();
  for (const auto& dir : run_dirs) {
    if (runs_json.size() >= static_cast<size_t>(runs)) break;
    json summary;
    std::string error;
    if (!load_run_summary(dir, summary, error)) {
      continue;
    }
    runs_json.push_back(summary);
  }

  json fallback_results;
  load_json_file(paths.root / "build" / "ai_results.json", fallback_results);

  if (json_out) {
    json out;
    json config;
    config["provider"] = effective_cfg.values.provider;
    config["provider_note"] = provider_note;
    config["model"] = effective_cfg.values.model;
    config["base_url"] = effective_cfg.values.base_url;
    config["templates_dir"] = effective_cfg.values.templates_dir;
    config["timeout_seconds"] = effective_cfg.values.timeout_seconds;
    config["sources"] = json::object();
    config["sources"]["provider"] = effective_cfg.provider_source;
    config["sources"]["model"] = effective_cfg.model_source;
    config["sources"]["base_url"] = effective_cfg.base_url_source;
    config["sources"]["templates_dir"] = effective_cfg.templates_dir_source;
    config["sources"]["timeout_seconds"] = effective_cfg.timeout_source;
    out["config"] = config;
    out["runs"] = runs_json;
    if (!fallback_results.empty()) {
      out["last_result"] = fallback_results;
    }
    std::cout << out.dump(2) << "\n";
    return 0;
  }

  std::cout << format_ai_config_line(effective_cfg, provider_note) << "\n";
  if (runs_json.empty()) {
    if (!fallback_results.empty()) {
      std::cout << "Last result (build/ai_results.json):\n";
      std::cout << "  run_id: " << fallback_results.value("run_id", "") << "\n";
      std::cout << "  status: " << fallback_results.value("status", "unknown") << "\n";
      std::cout << "  conflicts: " << fallback_results.value("conflicts", 0) << "\n";
      if (fallback_results.contains("context_drift")) {
        const auto drift = fallback_results["context_drift"];
        const bool drifted = drift.value("drift_detected", false);
        std::cout << "  drift: " << (drifted ? "mismatch" : "match") << "\n";
      }
    } else {
      std::cout << "No AI runs found.\n";
    }
    return 0;
  }

  for (const auto& run : runs_json) {
    const std::string run_id = run.value("run_id", "");
    const std::string created_at = run.value("created_at", "");
    const std::string goal = run.value("goal", "");
    const std::string plan_path = run.value("plan_path", "");
    const auto results = run.value("results", json::object());
    const bool has_results = !results.empty();
    const bool dry_run = results.value("dry_run", true);
    const std::string status = results.value("status", "unknown");
    const int conflicts = results.value("conflicts", 0);

    std::cout << "- run " << run_id;
    if (!created_at.empty()) std::cout << " (" << created_at << ")";
    if (!goal.empty()) std::cout << " goal: " << goal;
    std::cout << "\n";
    if (!plan_path.empty()) {
      std::cout << "  plan: " << plan_path << "\n";
    }
    if (has_results) {
      std::cout << "  apply: " << (dry_run ? "dry-run" : "applied")
                << ", status=" << status << ", conflicts=" << conflicts << "\n";
    } else {
      std::cout << "  apply: n/a\n";
    }

    if (run.contains("context_drift")) {
      const auto drift = run["context_drift"];
      const bool drifted = drift.value("drift_detected", false);
      if (drifted) {
        const auto counts = drift.value("changed_counts", json::object());
        std::cout << "  drift: mismatch";
        if (!counts.empty()) {
          std::cout << " (+" << counts.value("added", 0)
                    << " -" << counts.value("removed", 0)
                    << " ~" << counts.value("modified", 0) << ")";
        }
        std::cout << "\n";
      } else {
        std::cout << "  drift: match\n";
      }
    }
  }
  return 0;
}

int rollback_last() {
  const fs::path staging_root = fs::path("build") / "ai_staging";
  json applied;
  if (!load_json_file(staging_root / "last_applied.json", applied)) {
    std::cerr << "no last applied run\n";
    return 1;
  }
  const auto run_id = applied.value("run_id", "");
  if (run_id.empty()) {
    std::cerr << "invalid last applied record\n";
    return 1;
  }
  const fs::path run_dir = staging_root / run_id;
  if (!applied.contains("files") || !applied["files"].is_array()) {
    std::cerr << "last applied missing files list\n";
    return 1;
  }

  size_t restored = 0;
  for (const auto& file : applied["files"]) {
    const auto base_path = fs::path(file.value("base_path", ""));
    const auto target_path = fs::path(file.value("path", ""));
    if (base_path.empty() || target_path.empty()) {
      continue;
    }
    const auto base_contents = read_file_string(base_path);
    fs::create_directories(target_path.parent_path());
    rkg::data::write_text_file(target_path, base_contents);
    ++restored;
    json audit;
    audit["time"] = now_iso();
    audit["action_type"] = "rollback";
    audit["params_hash"] = file.value("base_hash", "");
    audit["result"] = "ok";
    audit["message"] = target_path.string();
    audit["files_touched"] = {target_path.string()};
    append_audit_line("build/ai_audit.log", audit);
  }

  std::cout << "rollback restored " << restored << " file(s)\n";
  return 0;
}

int context_dump(const char* argv0,
                 const std::optional<fs::path>& project_override,
                 const fs::path& out_path,
                 size_t cap) {
  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const fs::path project_root = paths.project;
  const auto cfg = rkg::load_project_config(project_root / "project.yaml");
  const std::string project_name = project_name_or_dir(cfg, project_root);

  json ctx;
  ctx["generated_at"] = now_iso();
  ctx["project_name"] = project_name;
  ctx["project_root"] = project_root.generic_string();
  ctx["engine_config"] = (paths.root / "config/engine.yaml").generic_string();
  ctx["content_root"] = (project_root / "content").generic_string();
  ctx["files"] = json::array();
  ctx["fingerprint"] = build_context_fingerprint(paths, project_root);

  const size_t effective_cap = cap == 0 ? kContextFileCap : cap;

  json files;
  FileListMeta files_meta;
  collect_file_list(project_root, project_root, effective_cap, files, files_meta);
  ctx["files"] = files;
  ctx["files_cap"] = static_cast<uint32_t>(effective_cap);
  ctx["files_truncated"] = files_meta.truncated;
  ctx["files_total_count"] = static_cast<uint32_t>(files_meta.total_count);
  ctx["files_included_count"] = static_cast<uint32_t>(files_meta.included_count);

  json core_files;
  FileListMeta core_meta;
  collect_file_list(paths.root, paths.root / "core", effective_cap, core_files, core_meta);
  ctx["core_files"] = core_files;
  ctx["core_files_cap"] = static_cast<uint32_t>(effective_cap);
  ctx["core_files_truncated"] = core_meta.truncated;
  ctx["core_files_total_count"] = static_cast<uint32_t>(core_meta.total_count);
  ctx["core_files_included_count"] = static_cast<uint32_t>(core_meta.included_count);

  json plugin_files;
  FileListMeta plugin_meta;
  collect_file_list(paths.root, paths.root / "plugins", effective_cap, plugin_files, plugin_meta);
  ctx["plugins_files"] = plugin_files;
  ctx["plugins_files_cap"] = static_cast<uint32_t>(effective_cap);
  ctx["plugins_files_truncated"] = plugin_meta.truncated;
  ctx["plugins_files_total_count"] = static_cast<uint32_t>(plugin_meta.total_count);
  ctx["plugins_files_included_count"] = static_cast<uint32_t>(plugin_meta.included_count);

  fs::create_directories(out_path.parent_path());
  rkg::data::write_text_file(out_path, ctx.dump(2));
  std::cout << "context pack -> " << out_path.string() << "\n";
  return 0;
}

int context_diff(const char* argv0,
                 const fs::path& baseline_path,
                 const std::optional<fs::path>& project_override,
                 bool include_core,
                 bool include_plugins,
                 const std::optional<size_t>& cap_override) {
  json baseline;
  if (!load_json_file(baseline_path, baseline)) {
    std::cerr << "baseline context not found: " << baseline_path.string() << "\n";
    return 1;
  }

  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const fs::path project_root = paths.project;

  json current_fp = build_context_fingerprint(paths, project_root);
  const json baseline_fp = baseline.value("fingerprint", json::object());

  const std::string base_hash = baseline_fp.value("tree_hash", "");
  const std::string cur_hash = current_fp.value("tree_hash", "");
  const std::string base_head = baseline_fp.value("git_head", "");
  const std::string cur_head = current_fp.value("git_head", "");

  if (cap_override.has_value()) {
    std::cout << "cap override: " << cap_override.value() << "\n";
  }

  if (!base_hash.empty() || !cur_hash.empty()) {
    std::cout << "tree_hash: " << base_hash << " -> " << cur_hash << "\n";
  }
  if (!base_head.empty() || !cur_head.empty()) {
    std::cout << "git_head: " << base_head << " -> " << cur_head << "\n";
  }

  const auto tree_hashes_base = baseline_fp.value("tree_hashes", json::object());
  const auto tree_hashes_cur = current_fp.value("tree_hashes", json::object());
  auto print_tree_hash = [&](const std::string& label) {
    const std::string base_val = tree_hashes_base.value(label, "");
    const std::string cur_val = tree_hashes_cur.value(label, "");
    if (!base_val.empty() || !cur_val.empty()) {
      std::cout << label << " tree_hash: " << base_val << " -> " << cur_val << "\n";
    }
  };
  if (include_core) {
    print_tree_hash("core");
  }
  if (include_plugins) {
    print_tree_hash("plugins");
  }

  const size_t file_cap = baseline.value("files_cap", kContextFileCap);
  const bool baseline_truncated = baseline.value("files_truncated", false);
  const size_t baseline_total = baseline.value("files_total_count", 0);
  const size_t baseline_included = baseline.value("files_included_count", 0);
  FileChangeSummary changes;
  FileListMeta current_meta;
  const size_t effective_cap = cap_override.has_value() ? cap_override.value() : file_cap;
  if (compute_file_changes(baseline, project_root, project_root, effective_cap, "files", changes, current_meta)) {
    std::cout << "files: +" << changes.added << " -" << changes.removed << " ~" << changes.modified << "\n";
    if (baseline_truncated || current_meta.truncated) {
      std::cout << "files: list truncated (baseline "
                << baseline_included << "/" << baseline_total
                << ", current " << current_meta.included_count << "/" << current_meta.total_count << ")\n";
    }
    const size_t max_list = 10;
    if (!changes.changed_paths.empty()) {
      std::cout << "changed (first " << std::min(max_list, changes.changed_paths.size()) << "):\n";
      for (size_t i = 0; i < changes.changed_paths.size() && i < max_list; ++i) {
        std::cout << "  " << changes.changed_paths[i] << "\n";
      }
    }
  } else {
    std::cout << "files: baseline file list missing\n";
  }

  if (include_core) {
    const size_t core_cap = baseline.value("core_files_cap", kContextFileCap);
    const bool core_base_truncated = baseline.value("core_files_truncated", false);
    const size_t core_base_total = baseline.value("core_files_total_count", 0);
    const size_t core_base_included = baseline.value("core_files_included_count", 0);
    FileChangeSummary core_changes;
    FileListMeta core_current_meta;
    const size_t core_effective = cap_override.has_value() ? cap_override.value() : core_cap;
    if (compute_file_changes(baseline, paths.root, paths.root / "core", core_effective, "core_files",
                             core_changes, core_current_meta)) {
      std::cout << "core files: +" << core_changes.added << " -" << core_changes.removed
                << " ~" << core_changes.modified << "\n";
      if (core_base_truncated || core_current_meta.truncated) {
        std::cout << "core files: list truncated (baseline "
                  << core_base_included << "/" << core_base_total
                  << ", current " << core_current_meta.included_count << "/" << core_current_meta.total_count << ")\n";
      }
    } else {
      std::cout << "core files: baseline list missing\n";
    }
  }

  if (include_plugins) {
    const size_t plugins_cap = baseline.value("plugins_files_cap", kContextFileCap);
    const bool plugins_base_truncated = baseline.value("plugins_files_truncated", false);
    const size_t plugins_base_total = baseline.value("plugins_files_total_count", 0);
    const size_t plugins_base_included = baseline.value("plugins_files_included_count", 0);
    FileChangeSummary plugin_changes;
    FileListMeta plugins_current_meta;
    const size_t plugins_effective = cap_override.has_value() ? cap_override.value() : plugins_cap;
    if (compute_file_changes(baseline, paths.root, paths.root / "plugins", plugins_effective, "plugins_files",
                             plugin_changes, plugins_current_meta)) {
      std::cout << "plugins files: +" << plugin_changes.added << " -" << plugin_changes.removed
                << " ~" << plugin_changes.modified << "\n";
      if (plugins_base_truncated || plugins_current_meta.truncated) {
        std::cout << "plugins files: list truncated (baseline "
                  << plugins_base_included << "/" << plugins_base_total
                  << ", current " << plugins_current_meta.included_count << "/" << plugins_current_meta.total_count
                  << ")\n";
      }
    } else {
      std::cout << "plugins files: baseline list missing\n";
    }
  }
  return 0;
}

bool load_plan_schema(const fs::path& root, json& schema, std::string& error) {
  const fs::path schema_path = root / "docs" / "schemas" / "rkg_plan.schema.json";
  if (!load_json_file(schema_path, schema)) {
    error = "plan schema not found: " + schema_path.string();
    return false;
  }
  return true;
}

bool write_json_file(const fs::path& path, const json& value) {
  fs::create_directories(path.parent_path());
  return rkg::data::write_text_file(path, value.dump(2));
}

std::string result_status_from_rc(int rc) {
  if (rc == 0) return "ok";
  if (rc == 2) return "conflicts";
  return "failed";
}

void write_ai_results_with_drift(const fs::path& run_dir,
                                 const std::string& run_id,
                                 const ApplyOptions& opts,
                                 int rc,
                                 const json& drift_info,
                                 const std::optional<std::string>& status_override) {
  json results;
  if (!load_json_file("build/ai_results.json", results)) {
    results = json::object();
  }
  results["run_id"] = run_id;
  results["dry_run"] = opts.dry_run;
  results["status"] = status_override.value_or(result_status_from_rc(rc));
  results["context_drift"] = drift_info;
  write_json_file("build/ai_results.json", results);
  write_json_file(run_dir / "results.json", results);
}

void collect_plan_paths(const json& plan, std::unordered_set<std::string>& out_paths) {
  if (!plan.contains("tasks") || !plan["tasks"].is_array()) return;
  for (const auto& task : plan["tasks"]) {
    if (!task.contains("actions") || !task["actions"].is_array()) continue;
    for (const auto& action : task["actions"]) {
      const std::string type = action.value("action", "");
      if (type == "write_text_file") {
        out_paths.insert(action.value("path", ""));
      } else if (type == "update_yaml") {
        out_paths.insert(action.value("path", ""));
      } else if (type == "create_prefab") {
        out_paths.insert(action.value("output_path", ""));
      } else if (type == "add_level_entity") {
        out_paths.insert(action.value("level_path", ""));
      }
    }
  }
  out_paths.erase("");
}

bool load_context_pack(const fs::path& path, json& ctx, std::string& error) {
  if (!load_json_file(path, ctx)) {
    error = "context pack missing: " + path.string();
    return false;
  }
  return true;
}

struct OpenAiPromptTemplates {
  std::string system;
  std::string user;
};

bool load_openai_templates(const fs::path& root,
                           const fs::path& templates_dir,
                           OpenAiPromptTemplates& out,
                           std::string& error) {
  fs::path base = templates_dir;
  if (base.empty()) {
    base = root / "docs" / "agent_templates" / "openai";
  } else if (!base.is_absolute()) {
    base = root / base;
  }
  if (!load_text_file(base / "planner_system.md", out.system, error)) {
    error = "failed to load planner_system.md: " + error;
    return false;
  }
  if (!load_text_file(base / "planner_user.md", out.user, error)) {
    error = "failed to load planner_user.md: " + error;
    return false;
  }
  return true;
}

std::string constraints_summary_text() {
  return "Output strict JSON only. Allowed actions: write_text_file, update_yaml, create_prefab, "
         "add_level_entity, record_audit. No shell commands or executable code. "
         "Use safe repo-relative paths under content/, projects/, config/, docs/, build/.";
}

std::string repo_paths_text(const rkg::ResolvedPaths& paths) {
  std::ostringstream oss;
  oss << "root=" << paths.root.generic_string() << "\n";
  oss << "project=" << paths.project.generic_string() << "\n";
  oss << "content_root=" << (paths.project / "content").generic_string() << "\n";
  oss << "shared_content=" << (paths.root / "content").generic_string();
  return oss.str();
}

std::string format_ai_config_line(const AiConfigResolved& cfg, const std::string& provider_note) {
  std::ostringstream oss;
  oss << "AI config: provider=" << cfg.values.provider << " (" << cfg.provider_source << ")";
  if (!provider_note.empty()) {
    oss << " [" << provider_note << "]";
  }
  oss << ", model=" << cfg.values.model << " (" << cfg.model_source << ")";
  oss << ", base_url=" << cfg.values.base_url << " (" << cfg.base_url_source << ")";
  oss << ", templates_dir=" << cfg.values.templates_dir << " (" << cfg.templates_dir_source << ")";
  oss << ", timeout=" << cfg.values.timeout_seconds << "s (" << cfg.timeout_source << ")";
  return oss.str();
}

json build_run_info(const std::string& run_id,
                    const std::string& goal,
                    const std::string& mode,
                    const AiConfigResolved& cfg,
                    const std::string& provider_note,
                    bool strict_enums) {
  json info;
  info["run_id"] = run_id;
  info["created_at"] = now_iso();
  info["goal"] = goal;
  info["mode"] = mode;
  info["provider"] = cfg.values.provider;
  info["provider_note"] = provider_note;
  info["model"] = cfg.values.model;
  info["base_url"] = cfg.values.base_url;
  info["templates_dir"] = cfg.values.templates_dir;
  info["timeout_seconds"] = cfg.values.timeout_seconds;
  info["strict_enums"] = strict_enums;
  json sources;
  sources["provider"] = cfg.provider_source;
  sources["model"] = cfg.model_source;
  sources["base_url"] = cfg.base_url_source;
  sources["templates_dir"] = cfg.templates_dir_source;
  sources["timeout_seconds"] = cfg.timeout_source;
  info["config_sources"] = sources;
  return info;
}

struct AiRuntimeConfig {
  std::string provider;
  std::string model;
  std::string base_url;
  std::string templates_dir;
  int timeout_seconds = 60;
};

struct AiConfigResolved {
  AiRuntimeConfig values;
  std::string provider_source;
  std::string model_source;
  std::string base_url_source;
  std::string templates_dir_source;
  std::string timeout_source;
};

std::string to_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

AiConfigResolved resolve_ai_config(const rkg::ProjectConfig& cfg,
                                   const std::string& provider_override,
                                   const std::string& model_override,
                                   const std::string& base_url_override,
                                   const std::string& templates_dir_override,
                                   const std::optional<int>& timeout_override) {
  AiConfigResolved out;

  if (!provider_override.empty()) {
    out.values.provider = to_lower(provider_override);
    out.provider_source = "cli";
  } else if (!cfg.ai.provider.empty()) {
    out.values.provider = to_lower(cfg.ai.provider);
    out.provider_source = "project";
  } else {
    out.values.provider = "openai";
    out.provider_source = "default";
  }

  if (!model_override.empty()) {
    out.values.model = model_override;
    out.model_source = "cli";
  } else {
    const std::string env_model = get_env("OPENAI_MODEL");
    if (!env_model.empty()) {
      out.values.model = env_model;
      out.model_source = "env";
    } else if (!cfg.ai.model.empty()) {
      out.values.model = cfg.ai.model;
      out.model_source = "project";
    } else {
      out.values.model = "gpt-5.2-codex";
      out.model_source = "default";
    }
  }

  if (!base_url_override.empty()) {
    out.values.base_url = base_url_override;
    out.base_url_source = "cli";
  } else {
    const std::string env_base = get_env("OPENAI_BASE_URL");
    if (!env_base.empty()) {
      out.values.base_url = env_base;
      out.base_url_source = "env";
    } else if (!cfg.ai.base_url.empty()) {
      out.values.base_url = cfg.ai.base_url;
      out.base_url_source = "project";
    } else {
      out.values.base_url = "https://api.openai.com";
      out.base_url_source = "default";
    }
  }

  if (!templates_dir_override.empty()) {
    out.values.templates_dir = templates_dir_override;
    out.templates_dir_source = "cli";
  } else if (!cfg.ai.templates_dir.empty()) {
    out.values.templates_dir = cfg.ai.templates_dir;
    out.templates_dir_source = "project";
  } else {
    out.values.templates_dir = "docs/agent_templates/openai";
    out.templates_dir_source = "default";
  }

  if (timeout_override.has_value()) {
    out.values.timeout_seconds = timeout_override.value();
    out.timeout_source = "cli";
  } else if (cfg.ai.timeout_seconds > 0) {
    out.values.timeout_seconds = cfg.ai.timeout_seconds;
    out.timeout_source = "project";
  } else {
    out.values.timeout_seconds = 60;
    out.timeout_source = "default";
  }

  if (out.values.timeout_seconds <= 0) out.values.timeout_seconds = 60;
  return out;
}

std::string trim_whitespace(std::string value) {
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front()))) {
    value.erase(value.begin());
  }
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
    value.pop_back();
  }
  return value;
}

std::string read_git_head_hash(const fs::path& root) {
  const fs::path head_path = root / ".git" / "HEAD";
  if (!fs::exists(head_path)) {
    return {};
  }
  const std::string head_contents = trim_whitespace(read_file_string(head_path));
  if (head_contents.rfind("ref:", 0) == 0) {
    const std::string ref = trim_whitespace(head_contents.substr(4));
    if (ref.empty()) return {};
    const fs::path ref_path = root / ".git" / ref;
    if (!fs::exists(ref_path)) {
      return {};
    }
    return trim_whitespace(read_file_string(ref_path));
  }
  return head_contents;
}

uint64_t hash_tree_for_dirs(const fs::path& root, const std::vector<fs::path>& dirs) {
  std::vector<std::string> entries;
  for (const auto& dir : dirs) {
    const fs::path abs = dir.is_absolute() ? dir : (root / dir);
    if (!fs::exists(abs)) continue;
    for (const auto& entry : fs::recursive_directory_iterator(abs)) {
      if (!entry.is_regular_file()) continue;
      const auto rel = fs::relative(entry.path(), root);
      const uint64_t mtime = file_mtime_epoch(entry.path());
      std::ostringstream line;
      line << rel.generic_string() << "|" << mtime;
      entries.push_back(line.str());
    }
  }
  std::sort(entries.begin(), entries.end());
  std::ostringstream joined;
  for (const auto& line : entries) {
    joined << line << "\n";
  }
  return fnv1a_64(joined.str());
}

json build_context_fingerprint(const rkg::ResolvedPaths& paths, const fs::path& project_root) {
  json fp;
  fp["root_path"] = paths.root.generic_string();
  fp["git_head"] = read_git_head_hash(paths.root);

  std::vector<fs::path> dirs;
  dirs.push_back("core");
  dirs.push_back("plugins");
  dirs.push_back("docs/schemas");
  dirs.push_back(fs::relative(project_root, paths.root));

  json tree_hashes = json::object();
  for (const auto& dir : dirs) {
    const uint64_t hash = hash_tree_for_dirs(paths.root, {dir});
    tree_hashes[dir.generic_string()] = to_hex(hash);
  }
  fp["tree_hashes"] = tree_hashes;
  fp["tree_hash"] = to_hex(hash_tree_for_dirs(paths.root, dirs));
  return fp;
}

constexpr size_t kContextFileCap = 500;

struct FileListMeta {
  size_t total_count = 0;
  size_t included_count = 0;
  bool truncated = false;
};

void collect_file_list(const fs::path& root,
                       const fs::path& dir,
                       size_t cap,
                       json& out_files,
                       FileListMeta& meta) {
  out_files = json::array();
  meta = {};
  const fs::path abs = dir.is_absolute() ? dir : (root / dir);
  if (!fs::exists(abs)) {
    return;
  }
  std::vector<fs::path> files;
  for (const auto& entry : fs::recursive_directory_iterator(abs)) {
    if (!entry.is_regular_file()) continue;
    const auto rel = fs::relative(entry.path(), root);
    files.push_back(rel);
  }
  std::sort(files.begin(), files.end());
  meta.total_count = files.size();
  const size_t effective_cap = cap == 0 ? files.size() : cap;
  meta.included_count = std::min(meta.total_count, effective_cap);
  meta.truncated = meta.total_count > effective_cap;
  for (size_t i = 0; i < meta.included_count; ++i) {
    const auto rel = files[i];
    const auto contents = read_file_string(root / rel);
    json item;
    item["path"] = rel.generic_string();
    item["hash"] = to_hex(fnv1a_64(contents));
    out_files.push_back(item);
  }
}

struct FileChangeSummary {
  size_t added = 0;
  size_t removed = 0;
  size_t modified = 0;
  std::vector<std::string> changed_paths;
};

bool compute_file_changes_from_list(const json& baseline_files,
                                    const json& current_files,
                                    FileChangeSummary& out) {
  if (!baseline_files.is_array() || !current_files.is_array()) {
    return false;
  }

  std::unordered_map<std::string, std::string> baseline_map;
  for (const auto& item : baseline_files) {
    const std::string path = item.value("path", "");
    const std::string hash = item.value("hash", "");
    if (!path.empty()) {
      baseline_map[path] = hash;
    }
  }

  std::unordered_map<std::string, std::string> current_map;
  for (const auto& item : current_files) {
    const std::string path = item.value("path", "");
    const std::string hash = item.value("hash", "");
    if (!path.empty()) {
      current_map[path] = hash;
    }
  }

  for (const auto& kv : baseline_map) {
    const auto it = current_map.find(kv.first);
    if (it == current_map.end()) {
      ++out.removed;
    } else if (it->second != kv.second) {
      ++out.modified;
      if (out.changed_paths.size() < 10) {
        out.changed_paths.push_back(kv.first);
      }
    }
  }

  for (const auto& kv : current_map) {
    if (baseline_map.find(kv.first) == baseline_map.end()) {
      ++out.added;
    }
  }

  return true;
}

bool compute_file_changes(const json& baseline_ctx,
                          const fs::path& root,
                          const fs::path& dir,
                          size_t cap,
                          const char* list_key,
                          FileChangeSummary& out,
                          FileListMeta& current_meta) {
  if (!baseline_ctx.contains(list_key) || !baseline_ctx[list_key].is_array()) {
    return false;
  }
  json current_files;
  collect_file_list(root, dir, cap, current_files, current_meta);
  return compute_file_changes_from_list(baseline_ctx[list_key], current_files, out);
}

bool detect_context_drift(const json& baseline_ctx,
                          const rkg::ResolvedPaths& paths,
                          const fs::path& project_root,
                          std::string& message,
                          json* out_current = nullptr) {
  if (!baseline_ctx.contains("fingerprint")) {
    message = "baseline context missing fingerprint";
    return true;
  }
  const auto current_fp = build_context_fingerprint(paths, project_root);
  if (out_current) {
    *out_current = current_fp;
  }
  const auto& baseline_fp = baseline_ctx["fingerprint"];
  const std::string base_hash = baseline_fp.value("tree_hash", "");
  const std::string cur_hash = current_fp.value("tree_hash", "");
  const std::string base_head = baseline_fp.value("git_head", "");
  const std::string cur_head = current_fp.value("git_head", "");

  if (!base_hash.empty() && !cur_hash.empty() && base_hash != cur_hash) {
    message = "tree hash mismatch";
    return true;
  }
  if (!base_head.empty() && !cur_head.empty() && base_head != cur_head) {
    message = "git HEAD mismatch";
    return true;
  }
  return false;
}

bool confirm_context_drift() {
  std::cout << "Context drift detected. Proceed anyway? (y/N): ";
  std::string line;
  std::getline(std::cin, line);
  return !line.empty() && (line[0] == 'y' || line[0] == 'Y');
}

int agent_openai_plan(const char* argv0,
                      const std::optional<fs::path>& project_override,
                      const std::string& goal,
                      const std::string& model_override,
                      const std::optional<fs::path>& out_override,
                      const std::optional<fs::path>& context_override,
                      const std::string& base_url_override,
                      const std::string& templates_dir_override,
                      const std::optional<int>& timeout_override,
                      const std::string& provider_note,
                      bool print_raw,
                      bool strict_enums) {
#if !RKG_ENABLE_OPENAI
  (void)argv0;
  (void)project_override;
  (void)goal;
  (void)model_override;
  (void)out_override;
  (void)context_override;
  (void)base_url_override;
  (void)templates_dir_override;
  (void)timeout_override;
  (void)provider_note;
  (void)print_raw;
  std::cerr << "OpenAI provider disabled (RKG_ENABLE_OPENAI=OFF). Install libcurl to enable.\n";
  return 1;
#else
  if (goal.empty()) {
    std::cerr << "goal is required\n";
    return 1;
  }
  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const auto cfg = rkg::load_project_config(paths.project / "project.yaml");
  const std::string run_id = make_run_id();
  const fs::path run_dir = paths.root / "build" / "ai_runs" / run_id;
  fs::create_directories(run_dir);

  json schema;
  std::string error;
  if (!load_plan_schema(paths.root, schema, error)) {
    std::cerr << error << "\n";
    return 1;
  }

  const AiConfigResolved ai_cfg =
      resolve_ai_config(cfg, "openai", model_override, base_url_override, templates_dir_override, timeout_override);
  std::cout << format_ai_config_line(ai_cfg, provider_note) << "\n";
  std::cout << "run dir -> " << run_dir.string() << "\n";

  json run_info = build_run_info(run_id, goal, "plan", ai_cfg, provider_note, strict_enums);
  write_json_file(run_dir / "run_info.json", run_info);

  OpenAiPromptTemplates templates;
  if (!load_openai_templates(paths.root, fs::path(ai_cfg.values.templates_dir), templates, error)) {
    std::cerr << error << "\n";
    return 1;
  }

  fs::path context_path = run_dir / "context_pack.json";
  if (context_override.has_value()) {
    context_path = context_override.value();
    if (!fs::exists(context_path)) {
      std::cerr << "context pack not found: " << context_path.string() << "\n";
      return 1;
    }
    std::error_code ec;
    fs::copy_file(context_path, run_dir / "context_pack.json", fs::copy_options::overwrite_existing, ec);
  } else {
    if (context_dump(argv0, project_override, run_dir / "context_pack.json", kContextFileCap) != 0) {
      std::cerr << "context pack generation failed\n";
      return 1;
    }
  }

  json ctx;
  if (!load_context_pack(run_dir / "context_pack.json", ctx, error)) {
    std::cerr << error << "\n";
    return 1;
  }

  const std::string api_key = get_env("OPENAI_API_KEY");
  if (api_key.empty()) {
    std::cerr << "OPENAI_API_KEY is not set. Export it before running.\n";
    return 1;
  }
  const std::string base_url = normalize_base_url(ai_cfg.values.base_url);

  std::unordered_map<std::string, std::string> vars;
  vars["GOAL"] = goal;
  vars["CONTEXT_PACK_JSON"] = ctx.dump(2);
  vars["CONSTRAINTS_SUMMARY"] = constraints_summary_text();
  vars["REPO_PATHS"] = repo_paths_text(paths);
  const std::string system_prompt = templates.system;
  const std::string user_prompt = render_template(templates.user, vars);

  json request;
  if (!build_openai_request_json(schema, ai_cfg.values.model, system_prompt, user_prompt, request, error)) {
    std::cerr << "openai request build failed: " << error << "\n";
    return 1;
  }

  write_json_file(run_dir / "openai_request.json", request);

  OpenAiProvider provider(api_key, base_url, ai_cfg.values.timeout_seconds);
  const LlmResult llm = provider.GeneratePlan(request);
  if (!llm.error.empty()) {
    if (!llm.raw_json.empty()) {
      rkg::data::write_text_file(run_dir / "openai_response.json", llm.raw_json);
    }
    std::cerr << "OpenAI request failed";
    if (llm.http_status > 0) std::cerr << " (HTTP " << llm.http_status << ")";
    std::cerr << ": " << llm.error << "\n";
    return 1;
  }

  if (!llm.raw_json.empty()) {
    rkg::data::write_text_file(run_dir / "openai_response.json", llm.raw_json);
  }

  if (print_raw) {
    std::cout << llm.parsed_json << "\n";
  }

  json plan;
  try {
    plan = json::parse(llm.parsed_json);
  } catch (const std::exception& e) {
    std::cerr << "plan parse failed: " << e.what() << "\n";
    return 1;
  }

  std::vector<PlanValidationError> errors;
  if (!validate_plan_detailed(plan, errors, strict_enums)) {
    print_plan_errors(errors);
    return 1;
  }

  const fs::path plan_path = out_override.value_or(run_dir / "plan.json");
  write_json_file(plan_path, plan);
  if (plan_path != (run_dir / "plan.json")) {
    write_json_file(run_dir / "plan.json", plan);
  }
  run_info["plan_path"] = plan_path.generic_string();
  write_json_file(run_dir / "run_info.json", run_info);

  std::unordered_set<std::string> paths_touched;
  collect_plan_paths(plan, paths_touched);
  std::cout << "plan tasks: " << plan["tasks"].size()
            << ", files referenced: " << paths_touched.size() << "\n";
  std::cout << "plan -> " << plan_path.string() << "\n";
  return 0;
#endif
}

int agent_openai_apply(const char* argv0,
                       const std::optional<fs::path>& project_override,
                       const std::string& goal,
                       const std::string& model_override,
                       const std::optional<fs::path>& context_override,
                       const std::string& base_url_override,
                       const std::string& templates_dir_override,
                       const std::optional<int>& timeout_override,
                       const std::string& provider_note,
                       bool print_raw,
                       bool allow_context_drift,
                       bool strict_enums,
                       const ApplyOptions& apply_opts) {
#if !RKG_ENABLE_OPENAI
  (void)argv0;
  (void)project_override;
  (void)goal;
  (void)model_override;
  (void)context_override;
  (void)base_url_override;
  (void)templates_dir_override;
  (void)timeout_override;
  (void)provider_note;
  (void)print_raw;
  (void)allow_context_drift;
  (void)apply_opts;
  std::cerr << "OpenAI provider disabled (RKG_ENABLE_OPENAI=OFF). Install libcurl to enable.\n";
  return 1;
#else
  if (goal.empty()) {
    std::cerr << "goal is required\n";
    return 1;
  }
  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const auto cfg = rkg::load_project_config(paths.project / "project.yaml");
  const std::string run_id = make_run_id();
  const fs::path run_dir = paths.root / "build" / "ai_runs" / run_id;
  fs::create_directories(run_dir);

  json schema;
  std::string error;
  if (!load_plan_schema(paths.root, schema, error)) {
    std::cerr << error << "\n";
    return 1;
  }

  const AiConfigResolved ai_cfg =
      resolve_ai_config(cfg, "openai", model_override, base_url_override, templates_dir_override, timeout_override);
  std::cout << format_ai_config_line(ai_cfg, provider_note) << "\n";
  std::cout << "run dir -> " << run_dir.string() << "\n";

  json run_info = build_run_info(run_id, goal, "apply", ai_cfg, provider_note, strict_enums);
  run_info["dry_run"] = apply_opts.dry_run;
  run_info["require_approval"] = apply_opts.require_approval;
  write_json_file(run_dir / "run_info.json", run_info);

  OpenAiPromptTemplates templates;
  if (!load_openai_templates(paths.root, fs::path(ai_cfg.values.templates_dir), templates, error)) {
    std::cerr << error << "\n";
    return 1;
  }

  if (context_override.has_value()) {
    if (!fs::exists(context_override.value())) {
      std::cerr << "context pack not found: " << context_override.value().string() << "\n";
      return 1;
    }
    std::error_code ec;
    fs::copy_file(context_override.value(), run_dir / "context_pack.json",
                  fs::copy_options::overwrite_existing, ec);
  } else {
    if (context_dump(argv0, project_override, run_dir / "context_pack.json", kContextFileCap) != 0) {
      std::cerr << "context pack generation failed\n";
      return 1;
    }
  }

  json ctx;
  if (!load_context_pack(run_dir / "context_pack.json", ctx, error)) {
    std::cerr << error << "\n";
    return 1;
  }

  const std::string api_key = get_env("OPENAI_API_KEY");
  if (api_key.empty()) {
    std::cerr << "OPENAI_API_KEY is not set. Export it before running.\n";
    return 1;
  }
  const std::string base_url = normalize_base_url(ai_cfg.values.base_url);

  std::unordered_map<std::string, std::string> vars;
  vars["GOAL"] = goal;
  vars["CONTEXT_PACK_JSON"] = ctx.dump(2);
  vars["CONSTRAINTS_SUMMARY"] = constraints_summary_text();
  vars["REPO_PATHS"] = repo_paths_text(paths);
  const std::string system_prompt = templates.system;
  const std::string user_prompt = render_template(templates.user, vars);

  json request;
  if (!build_openai_request_json(schema, ai_cfg.values.model, system_prompt, user_prompt, request, error)) {
    std::cerr << "openai request build failed: " << error << "\n";
    return 1;
  }

  write_json_file(run_dir / "openai_request.json", request);

  OpenAiProvider provider(api_key, base_url, ai_cfg.values.timeout_seconds);
  const LlmResult llm = provider.GeneratePlan(request);
  if (!llm.error.empty()) {
    if (!llm.raw_json.empty()) {
      rkg::data::write_text_file(run_dir / "openai_response.json", llm.raw_json);
    }
    std::cerr << "OpenAI request failed";
    if (llm.http_status > 0) std::cerr << " (HTTP " << llm.http_status << ")";
    std::cerr << ": " << llm.error << "\n";
    return 1;
  }
  if (!llm.raw_json.empty()) {
    rkg::data::write_text_file(run_dir / "openai_response.json", llm.raw_json);
  }

  if (print_raw) {
    std::cout << llm.parsed_json << "\n";
  }

  json plan;
  try {
    plan = json::parse(llm.parsed_json);
  } catch (const std::exception& e) {
    std::cerr << "plan parse failed: " << e.what() << "\n";
    return 1;
  }
  std::vector<PlanValidationError> errors;
  if (!validate_plan_detailed(plan, errors, strict_enums)) {
    print_plan_errors(errors);
    return 1;
  }

  const fs::path plan_path = run_dir / "plan.json";
  write_json_file(plan_path, plan);
  run_info["plan_path"] = plan_path.generic_string();
  write_json_file(run_dir / "run_info.json", run_info);

  std::string drift_message;
  json current_fp;
  const bool drift_detected = detect_context_drift(ctx, paths, paths.project, drift_message, &current_fp);
  FileChangeSummary changes;
  const size_t file_cap = ctx.value("files_cap", kContextFileCap);
  FileListMeta current_meta;
  const bool have_changes = compute_file_changes(ctx, paths.project, paths.project, file_cap, "files",
                                                 changes, current_meta);

  json drift_info;
  drift_info["baseline_fingerprint"] = ctx.value("fingerprint", json::object());
  drift_info["current_fingerprint"] = current_fp;
  drift_info["drift_detected"] = drift_detected;
  drift_info["allow_context_drift"] = allow_context_drift;
  if (!drift_message.empty()) {
    drift_info["message"] = drift_message;
  }
  if (have_changes) {
    json counts;
    counts["added"] = changes.added;
    counts["removed"] = changes.removed;
    counts["modified"] = changes.modified;
    drift_info["changed_counts"] = counts;
    if (!changes.changed_paths.empty()) {
      drift_info["changed_paths"] = changes.changed_paths;
    }
  }

  bool drift_confirmed = false;
  if (drift_detected) {
    std::cerr << "WARNING: context drift detected (" << drift_message << ")\n";
    if (!apply_opts.dry_run && !allow_context_drift) {
      if (apply_opts.require_approval) {
        drift_confirmed = confirm_context_drift();
        if (!drift_confirmed) {
          drift_info["override_used"] = false;
          drift_info["approved_interactive"] = false;
          write_ai_results_with_drift(run_dir, run_id, apply_opts, 2, drift_info, "blocked");
          return 2;
        }
      } else {
        std::cerr << "Use --allow-context-drift to proceed.\n";
        drift_info["override_used"] = false;
        write_ai_results_with_drift(run_dir, run_id, apply_opts, 2, drift_info, "blocked");
        return 2;
      }
    }
  }
  drift_info["override_used"] = drift_detected && (allow_context_drift || drift_confirmed);
  if (drift_confirmed) {
    drift_info["approved_interactive"] = true;
  }

  const StageResult stage = stage_plan(plan, plan_path);
  if (!stage.errors.empty()) {
    for (const auto& err : stage.errors) {
      std::cerr << "stage error: " << err << "\n";
    }
    return 1;
  }

  const fs::path staged_dir = run_dir / "staged_patches";
  fs::create_directories(staged_dir);
  copy_tree(stage.run_dir / "patches", staged_dir);

  const int rc = apply_staged(stage, apply_opts);

  write_ai_results_with_drift(run_dir, run_id, apply_opts, rc, drift_info);
  return rc;
#endif
}

int agent_offline_plan(const char* argv0,
                       const std::optional<fs::path>& project_override,
                       const fs::path& input_plan,
                       const std::optional<fs::path>& out_override,
                       const std::string& provider_note,
                       bool strict_enums) {
  if (input_plan.empty()) {
    std::cerr << "--in <plan.json> is required\n";
    return 1;
  }
  json plan;
  if (!load_json_file(input_plan, plan)) {
    std::cerr << "plan not found: " << input_plan.string() << "\n";
    return 1;
  }
  std::vector<PlanValidationError> errors;
  if (!validate_plan_detailed(plan, errors, strict_enums)) {
    print_plan_errors(errors);
    return 1;
  }

  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const auto cfg = rkg::load_project_config(paths.project / "project.yaml");
  const std::string run_id = make_run_id();
  const fs::path run_dir = paths.root / "build" / "ai_runs" / run_id;
  fs::create_directories(run_dir);

  const AiConfigResolved ai_cfg = resolve_ai_config(cfg, "offline", "", "", "", std::nullopt);
  std::cout << format_ai_config_line(ai_cfg, provider_note) << "\n";
  std::cout << "run dir -> " << run_dir.string() << "\n";

  json run_info = build_run_info(run_id, "", "plan", ai_cfg, provider_note, strict_enums);
  write_json_file(run_dir / "run_info.json", run_info);

  if (context_dump(argv0, project_override, run_dir / "context_pack.json", kContextFileCap) != 0) {
    std::cerr << "context pack generation failed\n";
    return 1;
  }

  write_json_file(run_dir / "offline_input.json", plan);
  const fs::path plan_path = out_override.value_or(run_dir / "plan.json");
  write_json_file(plan_path, plan);
  if (plan_path != (run_dir / "plan.json")) {
    write_json_file(run_dir / "plan.json", plan);
  }
  run_info["plan_path"] = plan_path.generic_string();
  write_json_file(run_dir / "run_info.json", run_info);

  std::unordered_set<std::string> paths_touched;
  collect_plan_paths(plan, paths_touched);
  std::cout << "plan tasks: " << plan["tasks"].size()
            << ", files referenced: " << paths_touched.size() << "\n";
  std::cout << "plan -> " << plan_path.string() << "\n";
  return 0;
}

int agent_offline_apply(const char* argv0,
                        const std::optional<fs::path>& project_override,
                        const fs::path& input_plan,
                        const std::optional<fs::path>& context_override,
                        const std::string& provider_note,
                        bool allow_context_drift,
                        bool strict_enums,
                        const ApplyOptions& apply_opts) {
  if (input_plan.empty()) {
    std::cerr << "--in <plan.json> is required\n";
    return 1;
  }
  json plan;
  if (!load_json_file(input_plan, plan)) {
    std::cerr << "plan not found: " << input_plan.string() << "\n";
    return 1;
  }
  std::vector<PlanValidationError> errors;
  if (!validate_plan_detailed(plan, errors, strict_enums)) {
    print_plan_errors(errors);
    return 1;
  }

  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const auto cfg = rkg::load_project_config(paths.project / "project.yaml");
  const std::string run_id = make_run_id();
  const fs::path run_dir = paths.root / "build" / "ai_runs" / run_id;
  fs::create_directories(run_dir);

  const AiConfigResolved ai_cfg = resolve_ai_config(cfg, "offline", "", "", "", std::nullopt);
  std::cout << format_ai_config_line(ai_cfg, provider_note) << "\n";
  std::cout << "run dir -> " << run_dir.string() << "\n";

  json run_info = build_run_info(run_id, "", "apply", ai_cfg, provider_note, strict_enums);
  run_info["dry_run"] = apply_opts.dry_run;
  run_info["require_approval"] = apply_opts.require_approval;
  write_json_file(run_dir / "run_info.json", run_info);

  fs::path baseline_context;
  if (context_override.has_value()) {
    baseline_context = context_override.value();
  } else {
    const fs::path sibling = input_plan.parent_path() / "context_pack.json";
    if (fs::exists(sibling)) {
      baseline_context = sibling;
    }
  }

  if (!baseline_context.empty() && fs::exists(baseline_context)) {
    std::error_code ec;
    fs::copy_file(baseline_context, run_dir / "context_pack.json", fs::copy_options::overwrite_existing, ec);
  } else {
    if (context_dump(argv0, project_override, run_dir / "context_pack.json", kContextFileCap) != 0) {
      std::cerr << "context pack generation failed\n";
      return 1;
    }
  }

  write_json_file(run_dir / "offline_input.json", plan);
  const fs::path plan_path = run_dir / "plan.json";
  write_json_file(plan_path, plan);
  run_info["plan_path"] = plan_path.generic_string();
  write_json_file(run_dir / "run_info.json", run_info);

  std::string drift_message;
  json ctx;
  json current_fp;
  const bool have_ctx = load_json_file(run_dir / "context_pack.json", ctx);
  const bool drift_detected =
      have_ctx && detect_context_drift(ctx, paths, paths.project, drift_message, &current_fp);
  FileChangeSummary changes;
  const size_t file_cap = ctx.value("files_cap", kContextFileCap);
  FileListMeta current_meta;
  const bool have_changes = have_ctx &&
      compute_file_changes(ctx, paths.project, paths.project, file_cap, "files", changes, current_meta);

  json drift_info;
  drift_info["baseline_fingerprint"] = have_ctx ? ctx.value("fingerprint", json::object()) : json::object();
  drift_info["current_fingerprint"] = current_fp;
  drift_info["drift_detected"] = drift_detected;
  drift_info["allow_context_drift"] = allow_context_drift;
  if (!drift_message.empty()) {
    drift_info["message"] = drift_message;
  }
  if (have_changes) {
    json counts;
    counts["added"] = changes.added;
    counts["removed"] = changes.removed;
    counts["modified"] = changes.modified;
    drift_info["changed_counts"] = counts;
    if (!changes.changed_paths.empty()) {
      drift_info["changed_paths"] = changes.changed_paths;
    }
  }

  bool drift_confirmed = false;
  if (drift_detected) {
    std::cerr << "WARNING: context drift detected (" << drift_message << ")\n";
    if (!apply_opts.dry_run && !allow_context_drift) {
      if (apply_opts.require_approval) {
        drift_confirmed = confirm_context_drift();
        if (!drift_confirmed) {
          drift_info["override_used"] = false;
          drift_info["approved_interactive"] = false;
          write_ai_results_with_drift(run_dir, run_id, apply_opts, 2, drift_info, "blocked");
          return 2;
        }
      } else {
        std::cerr << "Use --allow-context-drift to proceed.\n";
        drift_info["override_used"] = false;
        write_ai_results_with_drift(run_dir, run_id, apply_opts, 2, drift_info, "blocked");
        return 2;
      }
    }
  }
  drift_info["override_used"] = drift_detected && (allow_context_drift || drift_confirmed);
  if (drift_confirmed) {
    drift_info["approved_interactive"] = true;
  }

  const StageResult stage = stage_plan(plan, plan_path);
  if (!stage.errors.empty()) {
    for (const auto& err : stage.errors) {
      std::cerr << "stage error: " << err << "\n";
    }
    return 1;
  }

  const fs::path staged_dir = run_dir / "staged_patches";
  fs::create_directories(staged_dir);
  copy_tree(stage.run_dir / "patches", staged_dir);

  const int rc = apply_staged(stage, apply_opts);
  write_ai_results_with_drift(run_dir, run_id, apply_opts, rc, drift_info);
  return rc;
}

struct ZipEntry {
  std::string name;
  uint32_t crc = 0;
  uint32_t size = 0;
  uint32_t offset = 0;
};

bool write_zip_from_dir(const fs::path& root_dir, const fs::path& zip_path) {
  std::ofstream out(zip_path, std::ios::binary);
  if (!out) {
    return false;
  }
  std::vector<ZipEntry> entries;
  for (const auto& entry : fs::recursive_directory_iterator(root_dir)) {
    if (!entry.is_regular_file()) continue;
    const auto rel = fs::relative(entry.path(), root_dir).generic_string();
    const auto data = read_file_string(entry.path());
    ZipEntry ze;
    ze.name = rel;
    ze.size = static_cast<uint32_t>(data.size());
    ze.crc = crc32(data);
    ze.offset = static_cast<uint32_t>(out.tellp());

    write_u32(out, 0x04034b50);  // local file header signature
    write_u16(out, 20);          // version needed
    write_u16(out, 0);           // flags
    write_u16(out, 0);           // compression (store)
    write_u16(out, 0);           // mod time
    write_u16(out, 0);           // mod date
    write_u32(out, ze.crc);
    write_u32(out, ze.size);
    write_u32(out, ze.size);
    write_u16(out, static_cast<uint16_t>(ze.name.size()));
    write_u16(out, 0);  // extra len
    out.write(ze.name.data(), static_cast<std::streamsize>(ze.name.size()));
    out.write(data.data(), static_cast<std::streamsize>(data.size()));

    entries.push_back(ze);
  }

  const uint32_t central_dir_offset = static_cast<uint32_t>(out.tellp());
  for (const auto& ze : entries) {
    write_u32(out, 0x02014b50);  // central dir header
    write_u16(out, 20);          // version made by
    write_u16(out, 20);          // version needed
    write_u16(out, 0);           // flags
    write_u16(out, 0);           // compression
    write_u16(out, 0);           // mod time
    write_u16(out, 0);           // mod date
    write_u32(out, ze.crc);
    write_u32(out, ze.size);
    write_u32(out, ze.size);
    write_u16(out, static_cast<uint16_t>(ze.name.size()));
    write_u16(out, 0);  // extra
    write_u16(out, 0);  // comment
    write_u16(out, 0);  // disk number
    write_u16(out, 0);  // internal attrs
    write_u32(out, 0);  // external attrs
    write_u32(out, ze.offset);
    out.write(ze.name.data(), static_cast<std::streamsize>(ze.name.size()));
  }

  const uint32_t central_dir_size = static_cast<uint32_t>(out.tellp()) - central_dir_offset;
  write_u32(out, 0x06054b50);  // end of central dir
  write_u16(out, 0);           // disk number
  write_u16(out, 0);           // disk with central dir
  write_u16(out, static_cast<uint16_t>(entries.size()));
  write_u16(out, static_cast<uint16_t>(entries.size()));
  write_u32(out, central_dir_size);
  write_u32(out, central_dir_offset);
  write_u16(out, 0);  // comment length
  return true;
}

int package_command(const char* argv0,
                    const std::optional<fs::path>& project_override,
                    const std::optional<fs::path>& out_override) {
  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const fs::path project_root = paths.project;
  const auto cfg = rkg::load_project_config(project_root / "project.yaml");
  const std::string project_name = project_name_or_dir(cfg, project_root);

  std::string platform = "linux";
#if defined(_WIN32)
  platform = "windows";
#endif

  const fs::path dist_root = out_override.value_or(paths.dist_root / (project_name + "_" + platform));
  fs::create_directories(dist_root);

  // Ensure cooked content exists.
  if (content_cook(argv0, project_root, std::nullopt) != 0) {
    std::cerr << "content cook failed\n";
    return 1;
  }

  const fs::path cooked_dir = paths.content_cache_root / project_name;
  const fs::path bin_dir = dist_root / "bin";
  fs::create_directories(bin_dir);

  const std::string exe_name =
#if defined(_WIN32)
      "rkg_demo_game.exe";
#else
      "rkg_demo_game";
#endif
  const fs::path exe_path = paths.root / "build" / "bin" / exe_name;
  if (fs::exists(exe_path)) {
    fs::copy_file(exe_path, bin_dir / exe_name, fs::copy_options::overwrite_existing);
  }

  // Copy project files and configs.
  copy_tree(project_root, dist_root / "projects" / project_name);
  const fs::path engine_cfg = paths.root / "config" / "engine.yaml";
  if (fs::exists(engine_cfg)) {
    fs::create_directories(dist_root / "config");
    fs::copy_file(engine_cfg, dist_root / "config" / "engine.yaml", fs::copy_options::overwrite_existing);
  }
  copy_tree(cooked_dir, dist_root / "content_cache" / project_name);

  // Run scripts.
  const fs::path run_sh = dist_root / "run.sh";
  const fs::path run_bat = dist_root / "run.bat";
  const std::string run_sh_contents =
      "#!/usr/bin/env bash\n"
      "set -euo pipefail\n"
      "ROOT=\"$(cd \"$(dirname \"$0\")\" && pwd)\"\n"
      "export RKG_ROOT=\"$ROOT\"\n"
      "export RKG_PROJECT=\"$ROOT/projects/" + project_name + "\"\n"
      "\"$ROOT/bin/" + exe_name + "\"\n";
  rkg::data::write_text_file(run_sh, run_sh_contents);

  const std::string run_bat_contents =
      "@echo off\r\n"
      "set ROOT=%~dp0\r\n"
      "set RKG_ROOT=%ROOT%\r\n"
      "set RKG_PROJECT=%ROOT%projects\\" + project_name + "\r\n"
      "%ROOT%bin\\" + exe_name + "\r\n";
  rkg::data::write_text_file(run_bat, run_bat_contents);

  const fs::path zip_path = dist_root;
  const fs::path zip_file = dist_root.string() + ".zip";
  if (!write_zip_from_dir(zip_path, zip_file)) {
    std::cerr << "failed to write zip\n";
    return 1;
  }
  std::cout << "package -> " << dist_root.string() << "\n";
  std::cout << "zip -> " << zip_file.string() << "\n";
  return 0;
}

int replay_record_cmd(const char* argv0,
                      const std::optional<fs::path>& project_override,
                      const fs::path& out_path,
                      float seconds) {
  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const fs::path exe = paths.root / "build" / "bin"
#if defined(_WIN32)
      / "rkg_demo_game.exe";
#else
      / "rkg_demo_game";
#endif
  std::cout << "Replay record is in-engine. Run:\n";
  std::cout << exe.string() << " --record " << out_path.string() << " --fixed-dt 0.016\n";
  std::cout << "Recommended duration: " << seconds << " seconds\n";
  return 0;
}

int replay_verify_cmd(const char* argv0,
                      const std::optional<fs::path>& project_override,
                      const fs::path& replay_path) {
  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const fs::path exe = paths.root / "build" / "bin"
#if defined(_WIN32)
      / "rkg_demo_game.exe";
#else
      / "rkg_demo_game";
#endif
  std::cout << "Replay verify is in-engine. Run:\n";
  std::cout << exe.string() << " --replay " << replay_path.string() << " --fixed-dt 0.016\n";
  return 0;
}

ActionResult action_write_text_file(const json& action, bool dry_run) {
  ActionResult result;
  const auto path = fs::path(action.value("path", ""));
  const auto mode = action.value("mode", "create");
  const auto contents = action.value("contents", "");
  if (!is_safe_rel_path(path)) {
    result.message = "unsafe path";
    return result;
  }
  if (dry_run) {
    result.ok = true;
    result.message = "dry-run";
    return result;
  }
  fs::create_directories(path.parent_path());
  if (mode == "create" && fs::exists(path)) {
    result.message = "file exists";
    return result;
  }
  if (!rkg::data::write_text_file(path, contents)) {
    result.message = "write failed";
    return result;
  }
  result.ok = true;
  result.files_touched.push_back(path.string());
  return result;
}

ActionResult action_update_yaml(const json& action, bool dry_run) {
  ActionResult result;
#if !RKG_ENABLE_DATA_YAML
  result.message = "yaml disabled";
  return result;
#else
  const auto path = fs::path(action.value("path", ""));
  const auto keypath = action.value("keypath", "");
  const auto value = action.contains("value") ? action["value"] : json();
  if (!is_safe_rel_path(path)) {
    result.message = "unsafe path";
    return result;
  }
  if (dry_run) {
    result.ok = true;
    result.message = "dry-run";
    return result;
  }
  YAML::Node root;
  if (fs::exists(path)) {
    if (!rkg::data::load_yaml_file(path, root)) {
      result.message = "yaml load failed";
      return result;
    }
  } else {
    root = YAML::Node(YAML::NodeType::Map);
  }
  update_yaml_keypath(root, keypath, value);
  if (!rkg::data::save_yaml_file(path, root)) {
    result.message = "yaml save failed";
    return result;
  }
  result.ok = true;
  result.files_touched.push_back(path.string());
  return result;
#endif
}

ActionResult action_create_prefab(const json& action, bool dry_run) {
  ActionResult result;
#if !RKG_ENABLE_DATA_YAML
  result.message = "yaml disabled";
  return result;
#else
  const auto name = action.value("name", "prefab");
  const auto output_path = fs::path(action.value("output_path", ""));
  if (!is_safe_rel_path(output_path)) {
    result.message = "unsafe path";
    return result;
  }
  if (dry_run) {
    result.ok = true;
    result.message = "dry-run";
    return result;
  }
  YAML::Node prefab;
  prefab["name"] = name;
  const auto components = action.contains("components") ? action["components"] : json::object();
  prefab["components"] = yaml_from_json(components);
  if (!rkg::data::save_yaml_file(output_path, prefab)) {
    result.message = "prefab write failed";
    return result;
  }
  result.ok = true;
  result.files_touched.push_back(output_path.string());
  return result;
#endif
}

ActionResult action_add_level_entity(const json& action, bool dry_run) {
  ActionResult result;
#if !RKG_ENABLE_DATA_YAML
  result.message = "yaml disabled";
  return result;
#else
  const auto level_path = fs::path(action.value("level_path", ""));
  if (!is_safe_rel_path(level_path)) {
    result.message = "unsafe path";
    return result;
  }
  if (dry_run) {
    result.ok = true;
    result.message = "dry-run";
    return result;
  }
  YAML::Node level;
  if (fs::exists(level_path)) {
    if (!rkg::data::load_yaml_file(level_path, level)) {
      result.message = "level load failed";
      return result;
    }
  } else {
    level = YAML::Node(YAML::NodeType::Map);
  }

  YAML::Node entity;
  entity["name"] = action.value("name", "entity");
  entity["prefab"] = action.value("prefab_ref", "");
  const auto transform = action.contains("transform") ? action["transform"] : json::object();
  entity["transform"] = yaml_from_json(transform);

  level["entities"].push_back(entity);
  if (!rkg::data::save_yaml_file(level_path, level)) {
    result.message = "level save failed";
    return result;
  }
  result.ok = true;
  result.files_touched.push_back(level_path.string());
  return result;
#endif
}

ActionResult action_record_audit(const json& action, bool dry_run, const fs::path& audit_path) {
  ActionResult result;
  const std::string event = action.value("event", "event");
  if (dry_run) {
    result.ok = true;
    result.message = "dry-run";
    return result;
  }
  json rec;
  rec["time"] = now_iso();
  rec["event"] = event;
  append_audit_line(audit_path, rec);
  result.ok = true;
  result.files_touched.push_back(audit_path.string());
  return result;
}

ActionResult execute_action(const json& action, bool dry_run, const fs::path& audit_path) {
  const auto type = action.value("action", "");
  if (type == "write_text_file") return action_write_text_file(action, dry_run);
  if (type == "update_yaml") return action_update_yaml(action, dry_run);
  if (type == "create_prefab") return action_create_prefab(action, dry_run);
  if (type == "add_level_entity") return action_add_level_entity(action, dry_run);
  if (type == "record_audit") return action_record_audit(action, dry_run, audit_path);
  ActionResult result;
  result.message = "unknown action";
  return result;
}

void add_plan_error(std::vector<PlanValidationError>& errors,
                    const std::string& keypath,
                    const std::string& message) {
  errors.push_back({keypath, message});
}

void print_plan_errors(const std::vector<PlanValidationError>& errors) {
  const size_t limit = 10;
  const size_t count = errors.size();
  for (size_t i = 0; i < count && i < limit; ++i) {
    std::cerr << errors[i].keypath << " -> " << errors[i].message << "\n";
  }
  if (count > limit) {
    std::cerr << "… and " << (count - limit) << " more error(s)\n";
  }
}

bool validate_plan_detailed(const json& plan, std::vector<PlanValidationError>& errors, bool strict_enums) {
  if (!plan.is_object()) {
    add_plan_error(errors, "", "plan must be an object");
    return false;
  }
  if (!plan.contains("tasks") || !plan["tasks"].is_array()) {
    add_plan_error(errors, "tasks", "expected array");
    return false;
  }
  const auto& tasks = plan["tasks"];
  for (size_t i = 0; i < tasks.size(); ++i) {
    const auto& task = tasks[i];
    const std::string base = "tasks[" + std::to_string(i) + "]";
    if (!task.is_object()) {
      add_plan_error(errors, base, "expected object");
      continue;
    }
    if (!task.contains("id") || !task["id"].is_string()) {
      add_plan_error(errors, base + ".id", "expected string");
    }
    if (!task.contains("type") || !task["type"].is_string()) {
      add_plan_error(errors, base + ".type", "expected string");
    } else if (strict_enums) {
      const std::string type = task["type"].get<std::string>();
      static const std::unordered_set<std::string> kTypes = {
          "content_change", "asset_build", "build", "test", "run_demo", "doc_update"};
      if (kTypes.find(type) == kTypes.end()) {
        add_plan_error(errors,
                       base + ".type",
                       "invalid value; allowed: content_change, asset_build, build, test, run_demo, doc_update");
      }
    }
    if (task.contains("status") && task["status"].is_string() && strict_enums) {
      const std::string status = task["status"].get<std::string>();
      static const std::unordered_set<std::string> kStatus = {
          "planned", "in_progress", "done", "blocked"};
      if (kStatus.find(status) == kStatus.end()) {
        add_plan_error(errors,
                       base + ".status",
                       "invalid value; allowed: planned, in_progress, done, blocked");
      }
    }
    if (task.contains("actions")) {
      if (!task["actions"].is_array()) {
        add_plan_error(errors, base + ".actions", "expected array");
        continue;
      }
      const auto& actions = task["actions"];
      for (size_t j = 0; j < actions.size(); ++j) {
        const auto& action = actions[j];
        const std::string abase = base + ".actions[" + std::to_string(j) + "]";
        if (!action.is_object()) {
          add_plan_error(errors, abase, "expected object");
          continue;
        }
        if (!action.contains("action") || !action["action"].is_string()) {
          add_plan_error(errors, abase + ".action", "expected string action type");
          continue;
        }
        const std::string type = action.value("action", "");
        if (type == "write_text_file") {
          if (!action.contains("path") || !action["path"].is_string()) {
            add_plan_error(errors, abase + ".path", "expected string");
          }
          if (!action.contains("contents") || !action["contents"].is_string()) {
            add_plan_error(errors, abase + ".contents", "expected string");
          }
          if (!action.contains("mode") || !action["mode"].is_string()) {
            add_plan_error(errors, abase + ".mode", "expected string ('create' or 'overwrite')");
          }
        } else if (type == "update_yaml") {
          if (!action.contains("path") || !action["path"].is_string()) {
            add_plan_error(errors, abase + ".path", "expected string");
          }
          if (!action.contains("keypath") || !action["keypath"].is_string()) {
            add_plan_error(errors, abase + ".keypath", "expected string");
          }
          if (!action.contains("value")) {
            add_plan_error(errors, abase + ".value", "expected value");
          }
        } else if (type == "create_prefab") {
          if (!action.contains("name") || !action["name"].is_string()) {
            add_plan_error(errors, abase + ".name", "expected string");
          }
          if (!action.contains("components")) {
            add_plan_error(errors, abase + ".components", "expected object or array");
          }
        } else if (type == "add_level_entity") {
          if (!action.contains("level_path") || !action["level_path"].is_string()) {
            add_plan_error(errors, abase + ".level_path", "expected string");
          }
          if (!action.contains("prefab_ref") || !action["prefab_ref"].is_string()) {
            add_plan_error(errors, abase + ".prefab_ref", "expected string");
          }
        } else if (type == "record_audit") {
          if (!action.contains("event") || !action["event"].is_string()) {
            add_plan_error(errors, abase + ".event", "expected string");
          }
        } else {
          add_plan_error(errors, abase + ".action",
                         "unknown action type (expected write_text_file, update_yaml, create_prefab, add_level_entity, record_audit)");
        }
      }
    }
  }
  return errors.empty();
}

bool validate_plan(const json& plan, std::string& error) {
  std::vector<PlanValidationError> errors;
  if (!validate_plan_detailed(plan, errors, true)) {
    if (!errors.empty()) {
      error = errors.front().keypath + ": " + errors.front().message;
    } else {
      error = "plan invalid";
    }
    return false;
  }
  return true;
}

bool build_openai_request_json(const json& schema,
                               const std::string& model,
                               const std::string& system_prompt,
                               const std::string& user_prompt,
                               json& out,
                               std::string& error) {
  if (model.empty()) {
    error = "model missing";
    return false;
  }
  if (!schema.is_object()) {
    error = "schema invalid";
    return false;
  }
  out = json::object();
  out["model"] = model;
  out["input"] = json::array({
      {{"role", "system"},
       {"content", json::array({{{"type", "text"}, {"text", system_prompt}}})}},
      {{"role", "user"},
       {"content", json::array({{{"type", "text"}, {"text", user_prompt}}})}},
  });
  out["text"] = json::object();
  out["text"]["format"] = json::object({
      {"type", "json_schema"},
      {"json_schema",
       json::object({
           {"name", "rkg_plan"},
           {"schema", schema},
           {"strict", true},
       })},
  });
  return true;
}

bool extract_plan_json_from_openai_response(const json& response,
                                            std::string& out_plan_json,
                                            std::string& error) {
  auto try_parse_candidate = [&](const std::string& candidate) -> bool {
    json parsed = json::parse(candidate, nullptr, false);
    if (parsed.is_discarded()) {
      return false;
    }
    if (parsed.is_object() && parsed.contains("tasks") && parsed["tasks"].is_array()) {
      out_plan_json = parsed.dump();
      return true;
    }
    return false;
  };

  if (response.is_object() && response.contains("tasks")) {
    out_plan_json = response.dump();
    return true;
  }

  if (response.contains("output_text") && response["output_text"].is_string()) {
    if (try_parse_candidate(response["output_text"].get<std::string>())) {
      return true;
    }
  }

  auto try_extract_from_content = [&](const json& content) -> bool {
    if (content.contains("text") && content["text"].is_string()) {
      return try_parse_candidate(content["text"].get<std::string>());
    }
    if (content.contains("arguments") && content["arguments"].is_string()) {
      return try_parse_candidate(content["arguments"].get<std::string>());
    }
    if (content.contains("json") && content["json"].is_object()) {
      const auto& obj = content["json"];
      if (obj.contains("tasks")) {
        out_plan_json = obj.dump();
        return true;
      }
    }
    if (content.contains("data") && content["data"].is_object()) {
      const auto& obj = content["data"];
      if (obj.contains("tasks")) {
        out_plan_json = obj.dump();
        return true;
      }
    }
    if (content.contains("content") && content["content"].is_array()) {
      for (const auto& nested : content["content"]) {
        if (try_extract_from_content(nested)) {
          return true;
        }
      }
    }
    return false;
  };

  if (response.contains("output") && response["output"].is_array()) {
    for (const auto& output : response["output"]) {
      if (output.contains("content") && output["content"].is_array()) {
        for (const auto& content : output["content"]) {
          if (try_extract_from_content(content)) {
            return true;
          }
        }
      }
      if (output.contains("tool_calls") && output["tool_calls"].is_array()) {
        for (const auto& call : output["tool_calls"]) {
          if (call.contains("arguments") && call["arguments"].is_string()) {
            if (try_parse_candidate(call["arguments"].get<std::string>())) {
              return true;
            }
          }
        }
      }
      if (output.contains("text") && output["text"].is_string()) {
        if (try_parse_candidate(output["text"].get<std::string>())) {
          return true;
        }
      }
    }
  }

  if (response.contains("choices") && response["choices"].is_array()) {
    for (const auto& choice : response["choices"]) {
      if (choice.contains("message") && choice["message"].contains("content")) {
        const auto& content = choice["message"]["content"];
        if (content.is_string()) {
          if (try_parse_candidate(content.get<std::string>())) {
            return true;
          }
        }
      }
    }
  }

  error = "no structured output found in response";
  return false;
}

class OfflineProvider : public ILlmProvider {
 public:
  explicit OfflineProvider(fs::path plan_path) : plan_path_(std::move(plan_path)) {}

  LlmResult GeneratePlan(const json&) override {
    LlmResult result;
    if (plan_path_.empty()) {
      result.error = "offline plan path missing";
      return result;
    }
    result.raw_json = read_file_string(plan_path_);
    if (result.raw_json.empty()) {
      result.error = "offline plan read failed";
      return result;
    }
    result.parsed_json = result.raw_json;
    return result;
  }

 private:
  fs::path plan_path_;
};

class OpenAiProvider : public ILlmProvider {
 public:
  OpenAiProvider(std::string api_key, std::string base_url, int timeout_seconds)
      : api_key_(std::move(api_key)),
        base_url_(std::move(base_url)),
        timeout_seconds_(timeout_seconds) {}

  LlmResult GeneratePlan(const json& request) override {
    LlmResult result;
#if !RKG_ENABLE_OPENAI
    (void)request;
    result.error = "OpenAI provider disabled (RKG_ENABLE_OPENAI=OFF)";
    return result;
#else
    if (api_key_.empty()) {
      result.error = "OPENAI_API_KEY missing";
      return result;
    }
    const std::string url = base_url_ + "/v1/responses";
    const std::string body = request.dump();
    std::string response_body;

    curl_global_init(CURL_GLOBAL_DEFAULT);
    CURL* curl = curl_easy_init();
    if (!curl) {
      curl_global_cleanup();
      result.error = "curl init failed";
      return result;
    }

    struct curl_slist* headers = nullptr;
    const std::string auth = "Authorization: Bearer " + api_key_;
    headers = curl_slist_append(headers, auth.c_str());
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(timeout_seconds_));
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
                     +[](char* ptr, size_t size, size_t nmemb, void* userdata) -> size_t {
                       auto* out = static_cast<std::string*>(userdata);
                       out->append(ptr, size * nmemb);
                       return size * nmemb;
                     });
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);

    CURLcode res = curl_easy_perform(curl);
    long status = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    curl_global_cleanup();

    result.http_status = static_cast<int>(status);
    result.raw_json = response_body;

    if (res != CURLE_OK) {
      result.error = std::string("curl error: ") + curl_easy_strerror(res);
      return result;
    }
    if (response_body.empty()) {
      result.error = "empty response body";
      return result;
    }

    json response_json;
    try {
      response_json = json::parse(response_body);
    } catch (const std::exception& e) {
      result.error = std::string("response parse failed: ") + e.what();
      return result;
    }

    if (status >= 400) {
      std::ostringstream oss;
      if (status == 401 || status == 403) {
        oss << "authentication failed (HTTP " << status << ")";
      } else if (status == 429) {
        oss << "rate limit or quota exceeded (HTTP 429)";
      } else if (status == 400) {
        oss << "invalid request (HTTP 400)";
      } else {
        oss << "http error (" << status << ")";
      }
      if (response_json.contains("error")) {
        const auto& err = response_json["error"];
        const std::string message = err.value("message", "");
        if (!message.empty()) {
          oss << ": " << message;
        }
      } else {
        const std::string excerpt = response_body.substr(0, 200);
        if (!excerpt.empty()) {
          oss << ": " << excerpt;
        }
      }
      result.error = oss.str();
      return result;
    }

    if (response_json.contains("error")) {
      const auto& err = response_json["error"];
      std::string message = err.value("message", "openai error");
      std::string type = err.value("type", "");
      std::string code = err.value("code", "");
      std::ostringstream oss;
      oss << "OpenAI error";
      if (!type.empty()) oss << " (" << type << ")";
      if (!code.empty()) oss << " [" << code << "]";
      oss << ": " << message;
      result.error = oss.str();
      return result;
    }

    std::string plan_json;
    std::string extract_error;
    if (!extract_plan_json_from_openai_response(response_json, plan_json, extract_error)) {
      result.error = extract_error;
      return result;
    }
    result.parsed_json = plan_json;
    return result;
#endif
  }

 private:
  std::string api_key_;
  std::string base_url_;
  int timeout_seconds_ = 60;
};

int orchestrate(const CliOptions& opts) {
  rkg::log::init();
  const auto plan_path = fs::path(opts.plan_path);
  if (!fs::exists(plan_path)) {
    rkg::log::error("plan not found");
    return 1;
  }

  json plan;
  {
    std::ifstream in(plan_path);
    in >> plan;
  }

  std::string error;
  if (!validate_plan(plan, error)) {
    rkg::log::error(std::string("plan invalid: ") + error);
    return 1;
  }

  ApplyOptions apply_opts;
  apply_opts.dry_run = opts.dry_run;
  apply_opts.require_approval = opts.require_approval;

  const int rc = apply_plan(plan, plan_path, apply_opts);

  if (!opts.dry_run) {
    rkg::data::SqliteDatabase db;
    const fs::path sqlite_path = "build/ai_tasks.sqlite";
    if (db.open(sqlite_path.string())) {
      db.exec("CREATE TABLE IF NOT EXISTS tasks (id TEXT, type TEXT, status TEXT, updated TEXT);");
      for (const auto& task : plan["tasks"]) {
        const auto id = task.value("id", "");
        const auto type = task.value("type", "");
        const auto status = task.value("status", "planned");
        std::ostringstream sql;
        sql << "INSERT INTO tasks (id, type, status, updated) VALUES ('"
            << id << "', '" << type << "', '" << status << "', '" << now_iso() << "');";
        db.exec(sql.str());
      }
    }
  }

  rkg::log::info("orchestration complete");
  rkg::log::shutdown();
  return rc;
}

bool is_simple_name(const std::string& name) {
  if (name.empty()) return false;
  if (name.find("..") != std::string::npos) return false;
  if (name.find('/') != std::string::npos || name.find('\\') != std::string::npos) return false;
  for (char c : name) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-')) {
      return false;
    }
  }
  return true;
}

bool copy_tree(const fs::path& src, const fs::path& dst) {
  if (!fs::exists(src) || !fs::is_directory(src)) {
    return false;
  }
  fs::create_directories(dst);
  for (const auto& entry : fs::recursive_directory_iterator(src)) {
    const auto rel = fs::relative(entry.path(), src);
    const auto out = dst / rel;
    if (entry.is_directory()) {
      fs::create_directories(out);
    } else if (entry.is_regular_file()) {
      fs::create_directories(out.parent_path());
      std::error_code ec;
      fs::copy_file(entry.path(), out, fs::copy_options::overwrite_existing, ec);
      if (ec) {
        return false;
      }
    }
  }
  return true;
}

bool replace_token_in_file(const fs::path& path, const std::string& token, const std::string& value) {
  if (!fs::exists(path)) {
    return false;
  }
  auto contents = rkg::data::read_text_file(path);
  if (contents.empty() && fs::file_size(path) > 0) {
    return false;
  }
  size_t pos = 0;
  bool replaced = false;
  while ((pos = contents.find(token, pos)) != std::string::npos) {
    contents.replace(pos, token.size(), value);
    pos += value.size();
    replaced = true;
  }
  if (!replaced) {
    return true;
  }
  return rkg::data::write_text_file(path, contents);
}

bool add_project_to_cmake(const fs::path& cmake_path, const std::string& name) {
  std::string contents;
  if (fs::exists(cmake_path)) {
    contents = rkg::data::read_text_file(cmake_path);
  }
  const std::string line = "add_subdirectory(" + name + ")";
  if (contents.find(line) != std::string::npos) {
    return true;
  }
  if (!contents.empty() && contents.back() != '\n') {
    contents.push_back('\n');
  }
  contents.append(line);
  contents.push_back('\n');
  return rkg::data::write_text_file(cmake_path, contents);
}

int new_project(const std::string& name, const std::string& template_name) {
  rkg::log::init();
  auto finish = [](int code) {
    rkg::log::shutdown();
    return code;
  };
  if (!is_simple_name(name) || !is_simple_name(template_name)) {
    rkg::log::error("invalid project or template name");
    return finish(1);
  }

  const fs::path projects_root = "projects";
  const fs::path template_path = projects_root / template_name;
  const fs::path dest_path = projects_root / name;

  if (!fs::exists(template_path)) {
    rkg::log::error("template not found");
    return finish(1);
  }
  if (fs::exists(dest_path)) {
    rkg::log::error("project already exists");
    return finish(1);
  }

  if (!copy_tree(template_path, dest_path)) {
    rkg::log::error("failed to copy template");
    return finish(1);
  }

  const std::string token = "@PROJECT_NAME@";
  replace_token_in_file(dest_path / "project.yaml", token, name);
  replace_token_in_file(dest_path / "CMakeLists.txt", token, name);

  if (!add_project_to_cmake(projects_root / "CMakeLists.txt", name)) {
    rkg::log::warn("failed to update projects/CMakeLists.txt");
  }

  rkg::log::info("project created");
  return finish(0);
}

std::string project_name_or_dir(const rkg::ProjectConfig& cfg, const fs::path& project_root) {
  if (!cfg.name.empty()) return cfg.name;
  return project_root.filename().string();
}

int content_validate(const char* argv0, const std::optional<fs::path>& project_override) {
  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const fs::path project_root = paths.project;
  const fs::path content_root = project_root / "content";

  std::vector<ContentAsset> assets;
  std::vector<DependencyRef> deps;
  std::vector<ValidationError> errors;

  gather_assets(content_root, assets, deps, errors);
  validate_dependencies(assets, deps, errors);

  if (!errors.empty()) {
    print_validation_errors(errors);
    return 1;
  }
  return 0;
}

int content_cook(const char* argv0,
                 const std::optional<fs::path>& project_override,
                 const std::optional<fs::path>& out_override) {
  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const fs::path project_root = paths.project;
  const auto cfg = rkg::load_project_config(project_root / "project.yaml");
  const fs::path content_root = project_root / "content";
  const std::string project_name = project_name_or_dir(cfg, project_root);

  std::vector<ContentAsset> assets;
  std::vector<DependencyRef> deps;
  std::vector<ValidationError> errors;
  gather_assets(content_root, assets, deps, errors);
  validate_dependencies(assets, deps, errors);

  if (!errors.empty()) {
    print_validation_errors(errors);
    return 1;
  }

  fs::path out_dir = out_override.value_or(paths.content_cache_root / project_name);
  fs::create_directories(out_dir);

  std::vector<ContentAsset> sorted_assets = assets;
  std::sort(sorted_assets.begin(), sorted_assets.end(),
            [](const ContentAsset& a, const ContentAsset& b) { return a.logical_path < b.logical_path; });

  json index;
  index["project"] = project_name;
  index["generated_at"] = now_iso();
  index["content_root"] = out_dir.generic_string();
  index["source_root"] = content_root.generic_string();
  index["prefabs"] = json::array();
  index["levels"] = json::array();

  for (const auto& asset : sorted_assets) {
    json entry;
    entry["id"] = asset.id;
    entry["name"] = asset.name;
    entry["path"] = asset.logical_path;
    entry["source_hash"] = asset.source_hash;
    entry["mtime_epoch"] = asset.mtime_epoch;
    if (asset.type == ContentType::Level) {
      entry["dependencies"] = asset.dependencies;
      index["levels"].push_back(entry);
    } else {
      index["prefabs"].push_back(entry);
    }
  }

  for (const auto& asset : sorted_assets) {
    const fs::path src = content_root / asset.logical_path;
    const fs::path dst = out_dir / asset.logical_path;
    fs::create_directories(dst.parent_path());
    std::error_code ec;
    fs::copy_file(src, dst, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      std::cerr << "content cook copy failed: " << src.string() << " -> " << dst.string() << "\n";
      return 1;
    }
  }

  std::string pack_error;
  std::vector<PackEntry> pack_entries;
  if (build_pack_entries(content_root, sorted_assets, pack_entries, pack_error)) {
    const fs::path pack_path = out_dir / "content.pack";
    if (!write_pack_file(pack_path, pack_entries, pack_error)) {
      std::cerr << "content pack failed: " << pack_error << "\n";
      return 1;
    }
    index["pack_path"] = pack_path.generic_string();
  } else {
    std::cerr << "content pack failed: " << pack_error << "\n";
    return 1;
  }

  const fs::path index_path = out_dir / "content.index.json";
  rkg::data::write_text_file(index_path, index.dump(2));
  std::cout << "Cooked content -> " << index_path.string() << "\n";
  return 0;
}

int content_commit_overrides(const char* argv0,
                             const std::optional<fs::path>& project_override,
                             const fs::path& overrides_path,
                             const CommitOverridesOptions& opts) {
  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const fs::path project_root = paths.project;
  const fs::path run_root = paths.root / "build" / "ai_runs";

  auto write_marker = [&](const std::string& run_id, const fs::path& run_dir) {
    json marker;
    marker["run_id"] = run_id;
    marker["run_dir"] = run_dir.generic_string();
    marker["staging_dir"] = (run_dir / "staged_patches").generic_string();
    marker["results_path"] = (run_dir / "results.json").generic_string();
    write_json_file(run_root / "last_commit_overrides.json", marker);
  };

  if (opts.apply_staged_dir.has_value()) {
    const fs::path run_dir = opts.apply_staged_dir.value();
    if (!is_safe_run_dir(paths.root, run_dir)) {
      const std::string run_id = run_dir.filename().string();
      write_commit_results(run_dir, run_id, opts.apply, 1, 0, 0, {}, "apply",
                           "run_dir_invalid", "apply-staged dir must be under build/ai_runs",
                           run_dir / "staged_patches", opts.apply.force, false, "",
                           "", "", "", false, "");
      return 1;
    }

    StageResult stage;
    std::string error;
    if (!load_commit_stage_summary(run_dir, stage, error)) {
      const std::string run_id = run_dir.filename().string();
      write_commit_results(run_dir, run_id, opts.apply, 1, 0, 0, {}, "apply",
                           "staged_missing", error, run_dir / "staged_patches",
                           opts.apply.force, false, "", "", "", "", false, "");
      return 1;
    }

    std::string error_code;
    std::string error_message;
    std::string stage_name;
    const std::string run_id = stage.summary.value("run_id", run_dir.filename().string());
    write_marker(run_id, run_dir);
    std::cout << "run_dir: " << run_dir.generic_string() << "\n";
    std::cout << "staging_dir: " << stage.run_dir.generic_string() << "\n";
    const int rc = apply_commit_overrides(stage, opts.apply, run_dir, run_id, error_code, error_message, stage_name);
    return rc;
  }

  fs::path run_dir = opts.run_dir.value_or(run_root / make_run_id());
  if (!is_safe_run_dir(paths.root, run_dir)) {
    std::cerr << "run_dir must be under build/ai_runs\n";
    return 1;
  }

  const std::string run_id = run_dir.filename().string();
  const fs::path staging_dir = run_dir / "staged_patches";

  json run_info;
  run_info["run_id"] = run_id;
  run_info["created_at"] = now_iso();
  run_info["mode"] = "commit_overrides";
  run_info["overrides_path"] = overrides_path.generic_string();
  std::string level_path;
  if (opts.level_override.has_value()) {
    level_path = opts.level_override->generic_string();
  } else {
    const auto cfg = rkg::load_project_config(project_root / "project.yaml");
    level_path = cfg.initial_level;
  }
  run_info["level_path"] = level_path;
  run_info["entity_id"] = opts.entity_id.value_or("");
  run_info["entity_name"] = opts.entity_name.value_or("");
  run_info["dry_run"] = opts.apply.dry_run;
  run_info["require_approval"] = opts.apply.require_approval;
  write_json_file(run_dir / "run_info.json", run_info);

  std::string error_code;
  std::string error_message;
  const StageResult stage =
      stage_commit_overrides(paths.root, project_root, overrides_path, opts.level_override,
                             opts.entity_id, opts.entity_name, staging_dir, run_id, error_code, error_message);
  const SelectorInfo summary_selector = selector_from_summary(stage.summary);
  const SelectorInfo selector =
      summary_selector.selector_type.empty() ? selector_from_opts(opts) : summary_selector;

  write_marker(run_id, run_dir);

  std::cout << "run_dir: " << run_dir.generic_string() << "\n";
  std::cout << "staging_dir: " << staging_dir.generic_string() << "\n";

  if (!error_message.empty()) {
    write_commit_results(run_dir, run_id, opts.apply, 1, 0, 0, {}, "stage",
                         error_code, error_message, staging_dir, false, false, selector.entity_id,
                         selector.selector_type, selector.selector_value, selector.selector_warning,
                         false, "");
    return 1;
  }

  write_commit_results(run_dir, run_id, opts.apply, 0, 0, 0, {}, "stage", "", "", staging_dir,
                       false, false, selector.entity_id, selector.selector_type, selector.selector_value,
                       selector.selector_warning, false, "");
  if (opts.stage_only) {
    return 0;
  }

  std::string stage_name;
  const int rc = apply_commit_overrides(stage, opts.apply, run_dir, run_id, error_code, error_message, stage_name);
  return rc;
}

int content_list(const fs::path& cooked_path) {
  fs::path index_path = cooked_path;
  if (fs::is_directory(cooked_path)) {
    index_path = cooked_path / "content.index.json";
  }
  if (!fs::exists(index_path)) {
    std::cerr << "content index not found: " << index_path.string() << "\n";
    return 1;
  }
  json index;
  {
    std::ifstream in(index_path);
    in >> index;
  }
  const auto prefabs = index.contains("prefabs") ? index["prefabs"] : json::array();
  const auto levels = index.contains("levels") ? index["levels"] : json::array();
  std::cout << "Prefabs: " << prefabs.size() << "\n";
  for (const auto& item : prefabs) {
    std::cout << "  [prefab] " << item.value("name", "") << " (" << item.value("path", "") << ")\n";
  }
  std::cout << "Levels: " << levels.size() << "\n";
  for (const auto& item : levels) {
    std::cout << "  [level] " << item.value("name", "") << " (" << item.value("path", "") << ")\n";
  }
  return 0;
}

bool is_watch_relevant(const fs::path& project_root, const fs::path& path) {
  std::error_code ec;
  auto rel = fs::relative(path, project_root, ec);
  if (ec || rel.empty()) return false;
  if (rel == "project.yaml" || rel == "project.json") return true;
  const auto top = (*rel.begin()).string();
  return top == "content" || top == "config";
}

int content_watch(const char* argv0,
                  const std::optional<fs::path>& project_override,
                  const std::optional<fs::path>& out_override,
                  int debounce_ms) {
  const auto paths = rkg::resolve_paths(argv0, project_override, "demo_game");
  const fs::path project_root = paths.project;
  const auto cfg = rkg::load_project_config(project_root / "project.yaml");
  const std::string project_name = project_name_or_dir(cfg, project_root);
  const fs::path out_dir = out_override.value_or(paths.content_cache_root / project_name);
  fs::create_directories(out_dir);

  rkg::platform::FileWatcher watcher;
  if (!watcher.start(project_root)) {
    std::cerr << "failed to start watcher at " << project_root.string() << "\n";
    return 1;
  }
  std::cout << "watching content at " << project_root.string()
            << " (backend: " << watcher.backend_name() << ")\n";

  std::unordered_set<std::string> pending;
  auto last_change = std::chrono::steady_clock::time_point{};

  while (true) {
    std::vector<rkg::platform::FileChange> changes;
    watcher.poll(changes);
    const auto now = std::chrono::steady_clock::now();
    for (const auto& change : changes) {
      if (!is_watch_relevant(project_root, change.path)) continue;
      pending.insert(change.path.generic_string());
      last_change = now;
    }

    if (!pending.empty() && last_change != std::chrono::steady_clock::time_point()) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_change).count();
      if (elapsed >= debounce_ms) {
        std::cout << "changes detected (" << pending.size() << "):\n";
        for (const auto& path : pending) {
          std::cout << "  " << path << "\n";
        }

        const int validate_rc = content_validate(argv0, project_root);
        const bool validation_ok = (validate_rc == 0);
        int cook_rc = 1;
        std::string last_error;
        if (validation_ok) {
          cook_rc = content_cook(argv0, project_root, out_override);
          if (cook_rc != 0) {
            last_error = "cook failed";
          }
        } else {
          last_error = "validation failed";
          std::cerr << "validation failed; skipping cook\n";
        }

        const std::string timestamp = now_iso();
        json status;
        status["project"] = project_name;
        status["timestamp"] = timestamp;
        status["validation_ok"] = validation_ok;
        status["cook_ok"] = (cook_rc == 0);
        status["output_dir"] = out_dir.generic_string();
        status["content_index_path"] = (out_dir / "content.index.json").generic_string();
        status["content_pack_path"] = (out_dir / "content.pack").generic_string();
        status["last_success_timestamp"] = (validation_ok && cook_rc == 0) ? timestamp : "";
        status["last_error"] = last_error;
        status["changed_files"] = json::array();
        for (const auto& path : pending) {
          status["changed_files"].push_back(path);
        }
        rkg::data::write_text_file(out_dir / "cook_status.json", status.dump(2));

        pending.clear();
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return 0;
}

void print_usage() {
  std::cout << "Usage:\n"
            << "  rkgctl orchestrate --plan <path> [--apply] [--dry-run] [--no-approval]\n"
            << "  rkgctl new project <name> [--template template_game]\n"
            << "  rkgctl content validate --project <path>\n"
            << "  rkgctl content cook --project <path> [--out <dir>]\n"
            << "  rkgctl content list --cooked <dir>\n"
            << "  rkgctl content watch --project <path> [--out <dir>] [--debounce-ms <n>]\n"
            << "  rkgctl content dump-pack --pack <path> [--list|--extract <id_or_path>]\n"
            << "  rkgctl content commit-overrides --project <path> [--overrides <file>] [--level <path>] [--entity-id <id>] [--entity-name <name>] [--stage] [--run-dir <dir>]\n"
            << "  rkgctl content commit-overrides --project <path> [--overrides <file>] [--level <path>] [--entity-id <id>] [--entity-name <name>] --apply [--yes] [--dry-run] [--force] [--run-dir <dir>]\n"
            << "  rkgctl content commit-overrides --apply-staged <run_dir> [--yes] [--dry-run] [--force]\n"
            << "  rkgctl plan validate --plan <path>\n"
            << "  rkgctl plan summarize --plan <path>\n"
            << "  rkgctl plan schema --out <file>\n"
            << "  rkgctl apply --plan <path> [--yes] [--dry-run] [--force]\n"
            << "  rkgctl status\n"
            << "  rkgctl rollback --last\n"
            << "  rkgctl context dump --project <path> --out <file> [--cap <n>]\n"
            << "  rkgctl context diff --baseline <context_pack.json> --project <path> [--include-core] [--include-plugins] [--cap <n>]\n"
            << "  rkgctl package --project <path> [--out <dir>]\n"
            << "  rkgctl replay record --project <path> --out <file> [--seconds <n>]\n"
            << "  rkgctl replay verify --project <path> --replay <file>\n"
            << "  rkgctl agent status [--project <path>] [--runs <n>] [--json]\n"
            << "  rkgctl agent plan --goal \"<text>\" --project <path> [--provider <openai|offline>] [--model <name>] [--out <plan.json>] [--context <context.json>] [--base-url <url>] [--templates-dir <dir>] [--timeout-seconds <n>] [--no-strict-enums]\n"
            << "  rkgctl agent apply --goal \"<text>\" --project <path> [--provider <openai|offline>] [--yes] [--dry-run] [--no-approval] [--force] [--context <context.json>] [--allow-context-drift] [--no-strict-enums]\n"
            << "  rkgctl agent openai plan --project <path> --goal \"<text>\" [--model <name>] [--out <plan.json>] [--context <context.json>] [--base-url <url>] [--templates-dir <dir>] [--timeout-seconds <n>] [--print-raw] [--no-strict-enums]\n"
            << "  rkgctl agent openai apply --project <path> --goal \"<text>\" [--model <name>] [--yes] [--dry-run] [--no-approval] [--force] [--context <context.json>] [--base-url <url>] [--templates-dir <dir>] [--timeout-seconds <n>] [--print-raw] [--allow-context-drift] [--no-strict-enums]\n"
            << "  rkgctl agent offline plan --in <plan.json> --project <path> [--out <plan.json>] [--no-strict-enums]\n"
            << "  rkgctl agent offline apply --in <plan.json> --project <path> [--yes] [--dry-run] [--no-approval] [--force] [--context <context.json>] [--allow-context-drift] [--no-strict-enums]\n";
}

#ifndef RKGCTL_LIB
int main(int argc, char** argv) {
  const auto paths = rkg::resolve_paths(argc > 0 ? argv[0] : nullptr, std::nullopt, "demo_game");
  std::error_code ec;
  std::filesystem::current_path(paths.root, ec);
  if (argc < 2) {
    print_usage();
    return 1;
  }

  std::string command = argv[1];

  if (command == "orchestrate") {
    CliOptions opts;
    for (int i = 2; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--plan" && i + 1 < argc) {
        opts.plan_path = argv[++i];
      } else if (arg == "--apply") {
        opts.apply = true;
        opts.dry_run = false;
      } else if (arg == "--dry-run") {
        opts.dry_run = true;
      } else if (arg == "--no-approval") {
        opts.require_approval = false;
      } else if (arg == "--require-approval") {
        opts.require_approval = true;
      }
    }
    if (opts.plan_path.empty()) {
      print_usage();
      return 1;
    }
    return orchestrate(opts);
  }

  if (command == "new" && argc >= 4) {
    std::string sub = argv[2];
    if (sub != "project") {
      print_usage();
      return 1;
    }
    std::string name = argv[3];
    std::string tmpl = "template_game";
    for (int i = 4; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--template" && i + 1 < argc) {
        tmpl = argv[++i];
      }
    }
    return new_project(name, tmpl);
  }

  if (command == "content" && argc >= 3) {
    std::string sub = argv[2];
    std::optional<fs::path> project_override;
    std::optional<fs::path> out_override;
    fs::path overrides_path;
    fs::path cooked_path;
    fs::path pack_path;
    std::optional<std::string> extract_target;
    int debounce_ms = 300;
    CommitOverridesOptions commit_opts;
    for (int i = 3; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--project" && i + 1 < argc) {
        project_override = fs::path(argv[++i]);
      } else if (arg == "--out" && i + 1 < argc) {
        out_override = fs::path(argv[++i]);
      } else if (arg == "--overrides" && i + 1 < argc) {
        overrides_path = fs::path(argv[++i]);
      } else if (arg == "--level" && i + 1 < argc) {
        commit_opts.level_override = fs::path(argv[++i]);
      } else if (arg == "--entity-id" && i + 1 < argc) {
        commit_opts.entity_id = std::string(argv[++i]);
      } else if (arg == "--entity-name" && i + 1 < argc) {
        commit_opts.entity_name = std::string(argv[++i]);
      } else if (arg == "--run-dir" && i + 1 < argc) {
        commit_opts.run_dir = fs::path(argv[++i]);
      } else if (arg == "--apply-staged" && i + 1 < argc) {
        commit_opts.apply_staged_dir = fs::path(argv[++i]);
        commit_opts.apply.dry_run = false;
      } else if (arg == "--cooked" && i + 1 < argc) {
        cooked_path = fs::path(argv[++i]);
      } else if (arg == "--pack" && i + 1 < argc) {
        pack_path = fs::path(argv[++i]);
      } else if (arg == "--extract" && i + 1 < argc) {
        extract_target = std::string(argv[++i]);
      } else if (arg == "--debounce-ms" && i + 1 < argc) {
        debounce_ms = std::max(0, std::stoi(argv[++i]));
      } else if (arg == "--dry-run") {
        commit_opts.apply.dry_run = true;
      } else if (arg == "--yes") {
        commit_opts.apply.dry_run = false;
        commit_opts.apply.require_approval = false;
      } else if (arg == "--no-approval") {
        commit_opts.apply.require_approval = false;
      } else if (arg == "--force") {
        commit_opts.apply.force = true;
      } else if (arg == "--stage") {
        commit_opts.stage_only = true;
        commit_opts.apply.dry_run = true;
      } else if (arg == "--apply") {
        commit_opts.stage_only = false;
        commit_opts.apply.dry_run = false;
      }
    }
    if (sub == "validate") {
      return content_validate(argc > 0 ? argv[0] : nullptr, project_override);
    }
    if (sub == "cook") {
      return content_cook(argc > 0 ? argv[0] : nullptr, project_override, out_override);
    }
    if (sub == "list") {
      if (cooked_path.empty()) {
        print_usage();
        return 1;
      }
      return content_list(cooked_path);
    }
    if (sub == "watch") {
      return content_watch(argc > 0 ? argv[0] : nullptr, project_override, out_override, debounce_ms);
    }
    if (sub == "dump-pack") {
      if (pack_path.empty()) {
        print_usage();
        return 1;
      }
      return content_dump_pack(pack_path, extract_target);
    }
    if (sub == "commit-overrides") {
      if (overrides_path.empty()) {
        const auto paths = rkg::resolve_paths(argc > 0 ? argv[0] : nullptr, project_override, "demo_game");
        overrides_path = paths.project / "editor_overrides.yaml";
      }
      if (commit_opts.apply_staged_dir.has_value()) {
        commit_opts.stage_only = false;
      }
      return content_commit_overrides(argc > 0 ? argv[0] : nullptr, project_override, overrides_path, commit_opts);
    }
    print_usage();
    return 1;
  }

  if (command == "plan" && argc >= 3) {
    std::string sub = argv[2];
    fs::path plan_path;
    fs::path out_path;
    for (int i = 3; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--plan" && i + 1 < argc) {
        plan_path = argv[++i];
      } else if (arg == "--out" && i + 1 < argc) {
        out_path = argv[++i];
      }
    }
    if (sub == "validate") {
      if (plan_path.empty()) {
        print_usage();
        return 1;
      }
      return plan_validate(plan_path);
    }
    if (sub == "summarize") {
      if (plan_path.empty()) {
        print_usage();
        return 1;
      }
      return plan_summarize(plan_path);
    }
    if (sub == "schema") {
      return plan_schema_dump(paths.root, out_path);
    }
    print_usage();
    return 1;
  }

  if (command == "apply") {
    fs::path plan_path;
    ApplyOptions opts;
    for (int i = 2; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--plan" && i + 1 < argc) {
        plan_path = argv[++i];
      } else if (arg == "--dry-run") {
        opts.dry_run = true;
      } else if (arg == "--yes") {
        opts.dry_run = false;
        opts.require_approval = false;
      } else if (arg == "--no-approval") {
        opts.require_approval = false;
      } else if (arg == "--force") {
        opts.force = true;
      }
    }
    if (plan_path.empty()) {
      print_usage();
      return 1;
    }
    return apply_command(plan_path, opts);
  }

  if (command == "agent" && argc >= 3) {
    std::string provider;
    std::string sub = argv[2];
    int arg_start = 3;
    if (sub == "status") {
      std::optional<fs::path> project_override;
      int runs = 5;
      bool json_out = false;
      for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--project" && i + 1 < argc) {
          project_override = fs::path(argv[++i]);
        } else if (arg == "--runs" && i + 1 < argc) {
          runs = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--json") {
          json_out = true;
        }
      }
      return agent_status_command(argc > 0 ? argv[0] : nullptr, project_override, runs, json_out);
    }
    if (sub == "openai" || sub == "offline") {
      provider = sub;
      if (argc < 4) {
        print_usage();
        return 1;
      }
      sub = argv[3];
      arg_start = 4;
    }

    std::optional<fs::path> project_override;
    std::optional<fs::path> out_override;
    std::optional<fs::path> context_override;
    std::optional<fs::path> input_plan;
    std::string goal;
    std::string model;
    std::string base_url;
    std::string templates_dir;
    std::string provider_override;
    std::string provider_note;
    std::optional<int> timeout_override;
    bool print_raw = false;
    bool allow_context_drift = false;
    bool strict_enums = true;
    ApplyOptions opts;

    for (int i = arg_start; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--project" && i + 1 < argc) {
        project_override = fs::path(argv[++i]);
      } else if (arg == "--goal" && i + 1 < argc) {
        goal = argv[++i];
      } else if (arg == "--model" && i + 1 < argc) {
        model = argv[++i];
      } else if (arg == "--out" && i + 1 < argc) {
        out_override = fs::path(argv[++i]);
      } else if (arg == "--in" && i + 1 < argc) {
        input_plan = fs::path(argv[++i]);
      } else if (arg == "--context" && i + 1 < argc) {
        context_override = fs::path(argv[++i]);
      } else if (arg == "--base-url" && i + 1 < argc) {
        base_url = argv[++i];
      } else if (arg == "--templates-dir" && i + 1 < argc) {
        templates_dir = argv[++i];
      } else if (arg == "--provider" && i + 1 < argc) {
        provider_override = argv[++i];
      } else if (arg == "--timeout-seconds" && i + 1 < argc) {
        timeout_override = std::stoi(argv[++i]);
      } else if (arg == "--print-raw") {
        print_raw = true;
      } else if (arg == "--allow-context-drift") {
        allow_context_drift = true;
      } else if (arg == "--no-strict-enums") {
        strict_enums = false;
      } else if (arg == "--dry-run") {
        opts.dry_run = true;
      } else if (arg == "--yes") {
        opts.dry_run = false;
        opts.require_approval = false;
      } else if (arg == "--no-approval") {
        opts.require_approval = false;
      } else if (arg == "--force") {
        opts.force = true;
      }
    }

    if (provider.empty()) {
      const auto paths = rkg::resolve_paths(argc > 0 ? argv[0] : nullptr, project_override, "demo_game");
      const auto cfg = rkg::load_project_config(paths.project / "project.yaml");
      provider = to_lower(provider_override.empty() ? cfg.ai.provider : provider_override);
#if !RKG_ENABLE_OPENAI
      if (provider == "openai") {
        provider_note = "fallback: openai disabled";
        provider = "offline";
      }
#endif
      if (provider.empty()) {
        provider = "openai";
      }
    }

    const bool have_api_key = !get_env("OPENAI_API_KEY").empty();
#if !RKG_ENABLE_OPENAI
    const bool openai_enabled = false;
#else
    const bool openai_enabled = true;
#endif

    if (provider == "openai" && !openai_enabled) {
      if (input_plan.has_value()) {
        std::cerr << "OpenAI unavailable; falling back to offline provider.\n";
        provider_note = "fallback: openai disabled";
        provider = "offline";
      } else {
        std::cerr << "OpenAI unavailable (libcurl not enabled). "
                     "Provide --in <plan.json> to use offline provider.\n";
        return 1;
      }
    }

    if (provider == "openai" && !have_api_key) {
      if (input_plan.has_value()) {
        std::cerr << "OpenAI unavailable; falling back to offline provider.\n";
        provider_note = "fallback: missing OPENAI_API_KEY";
        provider = "offline";
      } else {
        std::cerr << "OpenAI unavailable (missing libcurl or OPENAI_API_KEY). "
                     "Provide --in <plan.json> to use offline provider.\n";
        return 1;
      }
    }

    if (provider == "openai") {
      if (sub == "plan") {
        return agent_openai_plan(argc > 0 ? argv[0] : nullptr,
                                 project_override,
                                 goal,
                                 model,
                                 out_override,
                                 context_override,
                                 base_url,
                                 templates_dir,
                                 timeout_override,
                                 provider_note,
                                 print_raw,
                                 strict_enums);
      }
      if (sub == "apply") {
        return agent_openai_apply(argc > 0 ? argv[0] : nullptr,
                                  project_override,
                                  goal,
                                  model,
                                  context_override,
                                  base_url,
                                  templates_dir,
                                  timeout_override,
                                  provider_note,
                                  print_raw,
                                  allow_context_drift,
                                  strict_enums,
                                  opts);
      }
    }

    if (provider == "offline") {
      if (!input_plan.has_value()) {
        std::cerr << "--in <plan.json> is required for offline provider\n";
        return 1;
      }
      if (sub == "plan") {
        return agent_offline_plan(argc > 0 ? argv[0] : nullptr,
                                  project_override,
                                  input_plan.value(),
                                  out_override,
                                  provider_note,
                                  strict_enums);
      }
      if (sub == "apply") {
        return agent_offline_apply(argc > 0 ? argv[0] : nullptr,
                                   project_override,
                                   input_plan.value(),
                                   context_override,
                                   provider_note,
                                   allow_context_drift,
                                   strict_enums,
                                   opts);
      }
    }

    print_usage();
    return 1;
  }

  if (command == "status") {
    return status_command();
  }

  if (command == "rollback" && argc >= 3) {
    std::string sub = argv[2];
    if (sub == "--last") {
      return rollback_last();
    }
    print_usage();
    return 1;
  }

  if (command == "context" && argc >= 3) {
    std::string sub = argv[2];
    std::optional<fs::path> project_override;
    fs::path out_path;
    fs::path baseline_path;
    bool include_core = false;
    bool include_plugins = false;
    std::optional<size_t> cap_override;
    for (int i = 3; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--project" && i + 1 < argc) {
        project_override = fs::path(argv[++i]);
      } else if (arg == "--out" && i + 1 < argc) {
        out_path = fs::path(argv[++i]);
      } else if (arg == "--baseline" && i + 1 < argc) {
        baseline_path = fs::path(argv[++i]);
      } else if (arg == "--include-core") {
        include_core = true;
      } else if (arg == "--include-plugins") {
        include_plugins = true;
      } else if (arg == "--cap" && i + 1 < argc) {
        cap_override = static_cast<size_t>(std::stoul(argv[++i]));
      }
    }
    if (sub == "dump") {
      if (out_path.empty()) {
        print_usage();
        return 1;
      }
      const size_t cap = cap_override.has_value() ? cap_override.value() : kContextFileCap;
      return context_dump(argc > 0 ? argv[0] : nullptr, project_override, out_path, cap);
    }
    if (sub == "diff") {
      if (baseline_path.empty()) {
        print_usage();
        return 1;
      }
      return context_diff(argc > 0 ? argv[0] : nullptr,
                          baseline_path,
                          project_override,
                          include_core,
                          include_plugins,
                          cap_override);
    }
    print_usage();
    return 1;
  }

  if (command == "package") {
    std::optional<fs::path> project_override;
    std::optional<fs::path> out_override;
    for (int i = 2; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--project" && i + 1 < argc) {
        project_override = fs::path(argv[++i]);
      } else if (arg == "--out" && i + 1 < argc) {
        out_override = fs::path(argv[++i]);
      }
    }
    return package_command(argc > 0 ? argv[0] : nullptr, project_override, out_override);
  }

  if (command == "replay" && argc >= 3) {
    std::string sub = argv[2];
    std::optional<fs::path> project_override;
    fs::path out_path;
    fs::path replay_path;
    float seconds = 10.0f;
    for (int i = 3; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--project" && i + 1 < argc) {
        project_override = fs::path(argv[++i]);
      } else if (arg == "--out" && i + 1 < argc) {
        out_path = fs::path(argv[++i]);
      } else if (arg == "--replay" && i + 1 < argc) {
        replay_path = fs::path(argv[++i]);
      } else if (arg == "--seconds" && i + 1 < argc) {
        seconds = std::stof(argv[++i]);
      }
    }
    if (sub == "record") {
      if (out_path.empty()) {
        print_usage();
        return 1;
      }
      return replay_record_cmd(argc > 0 ? argv[0] : nullptr, project_override, out_path, seconds);
    }
    if (sub == "verify") {
      if (replay_path.empty()) {
        print_usage();
        return 1;
      }
      return replay_verify_cmd(argc > 0 ? argv[0] : nullptr, project_override, replay_path);
    }
    print_usage();
    return 1;
  }

  print_usage();
  return 1;
}
#endif
