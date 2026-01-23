#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace rkg {

struct SnapshotRestoreStageResult {
  bool success = false;
  std::string run_id;
  std::filesystem::path run_dir;
  std::filesystem::path staging_dir;
  std::string error_code;
  std::string error_message;
};

struct SnapshotRestoreApplyResult {
  bool success = false;
  std::string error_code;
  std::string error_message;
  size_t applied = 0;
  size_t conflicts = 0;
  std::vector<std::string> conflict_files;
};

SnapshotRestoreStageResult stage_snapshot_restore(const std::filesystem::path& root,
                                                  const std::filesystem::path& target_path,
                                                  const std::filesystem::path& snapshot_path,
                                                  const std::string& snapshot_run_id);

SnapshotRestoreApplyResult apply_snapshot_restore(const std::filesystem::path& run_dir, bool dry_run);

} // namespace rkg
