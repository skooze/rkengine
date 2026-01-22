#include "rkg/config.h"

#include "rkg/log.h"

#include <fstream>
#include <optional>

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace rkg {

namespace {
bool file_exists(const std::filesystem::path& path) {
  std::error_code ec;
  return std::filesystem::exists(path, ec);
}

void apply_common_fields(EngineConfig& cfg, const std::string& renderer,
                         const std::vector<std::string>& scripting,
                         const std::optional<bool>& enable_ai,
                         const std::optional<bool>& data_sqlite,
                         const std::optional<bool>& data_yaml,
                         const std::optional<bool>& data_json) {
  if (!renderer.empty()) {
    cfg.renderer = renderer;
  }
  if (!scripting.empty()) {
    cfg.scripting = scripting;
  }
  if (enable_ai.has_value()) {
    cfg.enable_ai = *enable_ai;
  }
  if (data_sqlite.has_value()) {
    cfg.enable_data_sqlite = *data_sqlite;
  }
  if (data_yaml.has_value()) {
    cfg.enable_data_yaml = *data_yaml;
  }
  if (data_json.has_value()) {
    cfg.enable_data_json = *data_json;
  }
}
} // namespace

EngineConfig load_engine_config(const std::filesystem::path& path) {
  EngineConfig cfg;

  if (!file_exists(path)) {
    log::warn(std::string("config not found: ") + path.string());
    return cfg;
  }

  const auto ext = path.extension().string();
  if (ext == ".json") {
#if RKG_ENABLE_DATA_JSON
    std::ifstream in(path);
    nlohmann::json j;
    in >> j;
    const auto& root = j.contains("engine") ? j["engine"] : j;

    std::string renderer;
    std::vector<std::string> scripting;
    std::optional<bool> enable_ai;
    std::optional<bool> data_sqlite;
    std::optional<bool> data_yaml;
    std::optional<bool> data_json;

    if (root.contains("renderer")) {
      renderer = root["renderer"].get<std::string>();
    }
    if (root.contains("scripting") && root["scripting"].is_array()) {
      for (const auto& v : root["scripting"]) {
        scripting.push_back(v.get<std::string>());
      }
    }
    if (root.contains("ai")) {
      enable_ai = root["ai"].get<bool>();
    }
    if (root.contains("data")) {
      const auto& data = root["data"];
      if (data.contains("sqlite")) data_sqlite = data["sqlite"].get<bool>();
      if (data.contains("yaml")) data_yaml = data["yaml"].get<bool>();
      if (data.contains("json")) data_json = data["json"].get<bool>();
    }

    apply_common_fields(cfg, renderer, scripting, enable_ai, data_sqlite, data_yaml, data_json);
#else
    log::warn("JSON config requested but JSON support is disabled.");
#endif
    return cfg;
  }

  if (ext == ".yaml" || ext == ".yml") {
#if RKG_ENABLE_DATA_YAML
    YAML::Node doc = YAML::LoadFile(path.string());
    YAML::Node root = doc["engine"] ? doc["engine"] : doc;

    std::string renderer;
    std::vector<std::string> scripting;
    std::optional<bool> enable_ai;
    std::optional<bool> data_sqlite;
    std::optional<bool> data_yaml;
    std::optional<bool> data_json;

    if (root["renderer"]) {
      renderer = root["renderer"].as<std::string>();
    }
    if (root["scripting"]) {
      for (const auto& v : root["scripting"]) {
        scripting.push_back(v.as<std::string>());
      }
    }
    if (root["ai"]) {
      enable_ai = root["ai"].as<bool>();
    }
    if (root["data"]) {
      auto data = root["data"];
      if (data["sqlite"]) data_sqlite = data["sqlite"].as<bool>();
      if (data["yaml"]) data_yaml = data["yaml"].as<bool>();
      if (data["json"]) data_json = data["json"].as<bool>();
    }

    apply_common_fields(cfg, renderer, scripting, enable_ai, data_sqlite, data_yaml, data_json);
#else
    log::warn("YAML config requested but YAML support is disabled.");
#endif
    return cfg;
  }

  log::warn("Unknown config extension; using defaults.");
  return cfg;
}

} // namespace rkg
