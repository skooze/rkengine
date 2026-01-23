#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace rkg {

struct RunCleanupOptions {
  size_t keep_last = 50;
  int older_than_days = 14;
};

struct RunCleanupResult {
  std::vector<std::filesystem::path> candidates;
  std::string error;
};

RunCleanupResult collect_run_cleanup_candidates(const std::filesystem::path& root,
                                                const RunCleanupOptions& options,
                                                const std::filesystem::path& selected_run_dir);

} // namespace rkg
