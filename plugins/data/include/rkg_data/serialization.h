#pragma once

#include <filesystem>
#include <string>

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace rkg::data {

#if RKG_ENABLE_DATA_YAML
bool load_yaml_file(const std::filesystem::path& path, YAML::Node& out);
bool save_yaml_file(const std::filesystem::path& path, const YAML::Node& node);
#endif

#if RKG_ENABLE_DATA_JSON
bool load_json_file(const std::filesystem::path& path, nlohmann::json& out);
bool save_json_file(const std::filesystem::path& path, const nlohmann::json& node);
#endif

std::string read_text_file(const std::filesystem::path& path);
bool write_text_file(const std::filesystem::path& path, const std::string& contents);

} // namespace rkg::data
