#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace rkg {

struct ProjectConfig {
  struct AiConfig {
    std::string provider = "openai";
    std::string model = "gpt-5.2-codex";
    std::string base_url = "https://api.openai.com";
    std::string templates_dir = "docs/agent_templates/openai";
    int timeout_seconds = 60;
  };

  std::string name;
  std::string renderer = "null";
  std::vector<std::string> plugins;
  std::string initial_level;
  std::string input_map;
  bool dev_mode = false;
  AiConfig ai;
};

ProjectConfig load_project_config(const std::filesystem::path& path);

} // namespace rkg
