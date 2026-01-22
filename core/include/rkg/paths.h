#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace rkg {

struct ResolvedPaths {
  std::filesystem::path root;
  std::filesystem::path project;
  std::filesystem::path content_root;
  std::filesystem::path shared_content;
  std::filesystem::path logs_dir;
  std::filesystem::path content_cache_root;
  std::filesystem::path dist_root;
};

ResolvedPaths resolve_paths(const char* argv0,
                            const std::optional<std::filesystem::path>& project_override,
                            const std::string& default_project_name);

} // namespace rkg
