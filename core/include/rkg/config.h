#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace rkg {

struct EngineConfig {
  std::string renderer = "null";
  std::vector<std::string> scripting;
  bool enable_ai = true;
  bool enable_data_sqlite = true;
  bool enable_data_yaml = true;
  bool enable_data_json = true;
};

EngineConfig load_engine_config(const std::filesystem::path& path);

} // namespace rkg
