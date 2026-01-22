#include "rkg/project.h"

#include "rkg/log.h"

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif

#include <fstream>
#include <optional>

namespace rkg {

namespace {
bool file_exists(const std::filesystem::path& path) {
  std::error_code ec;
  return std::filesystem::exists(path, ec);
}

void apply_fields(ProjectConfig& cfg,
                  const std::string& name,
                  const std::string& renderer,
                  const std::vector<std::string>& plugins,
                  const std::string& initial_level,
                  const std::string& input_map,
                  const std::optional<bool>& dev_mode) {
  if (!name.empty()) cfg.name = name;
  if (!renderer.empty()) cfg.renderer = renderer;
  if (!plugins.empty()) cfg.plugins = plugins;
  if (!initial_level.empty()) cfg.initial_level = initial_level;
  if (!input_map.empty()) cfg.input_map = input_map;
  if (dev_mode.has_value()) cfg.dev_mode = dev_mode.value();
}

void apply_ai_fields(ProjectConfig::AiConfig& ai,
                     const std::string& provider,
                     const std::string& model,
                     const std::string& base_url,
                     const std::string& templates_dir,
                     const std::optional<int>& timeout_seconds) {
  if (!provider.empty()) ai.provider = provider;
  if (!model.empty()) ai.model = model;
  if (!base_url.empty()) ai.base_url = base_url;
  if (!templates_dir.empty()) ai.templates_dir = templates_dir;
  if (timeout_seconds.has_value() && timeout_seconds.value() > 0) {
    ai.timeout_seconds = timeout_seconds.value();
  }
}
} // namespace

ProjectConfig load_project_config(const std::filesystem::path& path) {
  ProjectConfig cfg;

  if (!file_exists(path)) {
    log::warn(std::string("project config not found: ") + path.string());
    return cfg;
  }

  const auto ext = path.extension().string();
  if (ext == ".json") {
#if RKG_ENABLE_DATA_JSON
    std::ifstream in(path);
    nlohmann::json j;
    in >> j;
    const auto& root = j.contains("project") ? j["project"] : j;

    std::string name;
    std::string renderer;
    std::vector<std::string> plugins;
    std::string initial_level;
    std::string input_map;
    std::optional<bool> dev_mode;
    std::string ai_provider;
    std::string ai_model;
    std::string ai_base_url;
    std::string ai_templates_dir;
    std::optional<int> ai_timeout;

    if (root.contains("name")) name = root["name"].get<std::string>();
    if (root.contains("renderer")) renderer = root["renderer"].get<std::string>();
    if (root.contains("plugins") && root["plugins"].is_array()) {
      for (const auto& v : root["plugins"]) {
        plugins.push_back(v.get<std::string>());
      }
    }
    if (root.contains("initial_level")) initial_level = root["initial_level"].get<std::string>();
    if (root.contains("input_map")) input_map = root["input_map"].get<std::string>();
    if (root.contains("dev_mode")) dev_mode = root["dev_mode"].get<bool>();

    if (root.contains("ai") && root["ai"].is_object()) {
      const auto& ai = root["ai"];
      if (ai.contains("provider")) ai_provider = ai["provider"].get<std::string>();
      if (ai.contains("model")) ai_model = ai["model"].get<std::string>();
      if (ai.contains("base_url")) ai_base_url = ai["base_url"].get<std::string>();
      if (ai.contains("templates_dir")) ai_templates_dir = ai["templates_dir"].get<std::string>();
      if (ai.contains("timeout_seconds")) ai_timeout = ai["timeout_seconds"].get<int>();
    }

    apply_fields(cfg, name, renderer, plugins, initial_level, input_map, dev_mode);
    apply_ai_fields(cfg.ai, ai_provider, ai_model, ai_base_url, ai_templates_dir, ai_timeout);
#else
    log::warn("JSON project config requested but JSON support is disabled.");
#endif
    return cfg;
  }

  if (ext == ".yaml" || ext == ".yml") {
#if RKG_ENABLE_DATA_YAML
    YAML::Node doc = YAML::LoadFile(path.string());
    YAML::Node root = doc["project"] ? doc["project"] : doc;

    std::string name;
    std::string renderer;
    std::vector<std::string> plugins;
    std::string initial_level;
    std::string input_map;
    std::optional<bool> dev_mode;
    std::string ai_provider;
    std::string ai_model;
    std::string ai_base_url;
    std::string ai_templates_dir;
    std::optional<int> ai_timeout;

    if (root["name"]) name = root["name"].as<std::string>();
    if (root["renderer"]) renderer = root["renderer"].as<std::string>();
    if (root["plugins"]) {
      for (const auto& v : root["plugins"]) {
        plugins.push_back(v.as<std::string>());
      }
    }
    if (root["initial_level"]) initial_level = root["initial_level"].as<std::string>();
    if (root["input_map"]) input_map = root["input_map"].as<std::string>();
    if (root["dev_mode"]) dev_mode = root["dev_mode"].as<bool>();

    if (root["ai"]) {
      const auto ai = root["ai"];
      if (ai["provider"]) ai_provider = ai["provider"].as<std::string>();
      if (ai["model"]) ai_model = ai["model"].as<std::string>();
      if (ai["base_url"]) ai_base_url = ai["base_url"].as<std::string>();
      if (ai["templates_dir"]) ai_templates_dir = ai["templates_dir"].as<std::string>();
      if (ai["timeout_seconds"]) ai_timeout = ai["timeout_seconds"].as<int>();
    }

    apply_fields(cfg, name, renderer, plugins, initial_level, input_map, dev_mode);
    apply_ai_fields(cfg.ai, ai_provider, ai_model, ai_base_url, ai_templates_dir, ai_timeout);
#else
    log::warn("YAML project config requested but YAML support is disabled.");
#endif
    return cfg;
  }

  log::warn("Unknown project config extension; using defaults.");
  return cfg;
}

} // namespace rkg
