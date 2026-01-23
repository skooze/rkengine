#include "rkg/snapshot_restore.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

namespace rkg {
namespace fs = std::filesystem;

static std::string read_text_file(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

static bool write_text_file(const fs::path& path, const std::string& contents) {
  std::ofstream out(path, std::ios::binary);
  if (!out) return false;
  out << contents;
  return static_cast<bool>(out);
}

static uint64_t fnv1a_64(const std::string& text) {
  uint64_t hash = 1469598103934665603ull;
  for (unsigned char c : text) {
    hash ^= static_cast<uint64_t>(c);
    hash *= 1099511628211ull;
  }
  return hash;
}

static std::string to_hex(uint64_t value) {
  std::ostringstream out;
  out << std::hex << std::setw(16) << std::setfill('0') << value;
  return out.str();
}

static std::string now_iso() {
  using clock = std::chrono::system_clock;
  const auto now = clock::now();
  const auto t = clock::to_time_t(now);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
  return out.str();
}

static std::vector<std::string> split_lines(const std::string& text) {
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

static std::string unified_diff(const std::string& old_text,
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

static std::string sanitize_filename(const fs::path& path) {
  std::string name = path.generic_string();
  for (char& c : name) {
    if (c == '/' || c == '\\' || c == ':' || c == ' ') {
      c = '_';
    }
  }
  return name;
}

static std::string make_run_id() {
  using clock = std::chrono::system_clock;
  const auto now = clock::now();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  return "restore_" + std::to_string(ms);
}

static bool is_under_root(const fs::path& root, const fs::path& path) {
  std::error_code ec;
  auto rel = fs::relative(path, root, ec);
  if (ec) return false;
  return !rel.empty() && rel.native().find("..") != 0;
}

SnapshotRestoreStageResult stage_snapshot_restore(const fs::path& root,
                                                  const fs::path& target_path,
                                                  const fs::path& snapshot_path,
                                                  const std::string& snapshot_run_id) {
  SnapshotRestoreStageResult result;
#if !RKG_ENABLE_DATA_JSON
  result.error_code = "json_disabled";
  result.error_message = "JSON disabled; snapshot restore unavailable";
  return result;
#else
  if (target_path.empty() || snapshot_path.empty()) {
    result.error_code = "invalid_args";
    result.error_message = "target or snapshot missing";
    return result;
  }
  fs::path target_abs = target_path;
  if (!target_abs.is_absolute()) {
    target_abs = root / target_abs;
  }
  if (!is_under_root(root, target_abs)) {
    result.error_code = "target_outside_root";
    result.error_message = "target path is outside repo";
    return result;
  }

  std::error_code ec;
  if (!fs::exists(snapshot_path, ec) || ec) {
    result.error_code = "snapshot_missing";
    result.error_message = "snapshot file missing";
    return result;
  }

  const std::string snapshot_text = read_text_file(snapshot_path);
  const std::string current_text = read_text_file(target_abs);

  const fs::path run_root = root / "build" / "ai_runs";
  fs::create_directories(run_root, ec);
  std::string run_id = make_run_id();
  fs::path run_dir = run_root / run_id;
  int attempt = 0;
  while (fs::exists(run_dir, ec) && !ec && attempt < 10) {
    ++attempt;
    run_id = make_run_id() + "_" + std::to_string(attempt);
    run_dir = run_root / run_id;
  }
  fs::create_directories(run_dir, ec);
  if (ec) {
    result.error_code = "run_dir_failed";
    result.error_message = "run dir create failed";
    return result;
  }

  const fs::path staging_dir = run_dir / "staged_patches";
  const fs::path patches_dir = staging_dir / "patches";
  const fs::path base_dir = staging_dir / "base_files";
  const fs::path proposed_dir = staging_dir / "proposed_files";
  fs::create_directories(patches_dir, ec);
  fs::create_directories(base_dir, ec);
  fs::create_directories(proposed_dir, ec);
  if (ec) {
    result.error_code = "staging_dir_failed";
    result.error_message = "staging dir create failed";
    return result;
  }

  const fs::path rel_target = fs::relative(target_abs, root, ec);
  const std::string rel_path = ec ? target_abs.generic_string() : rel_target.generic_string();
  const std::string base_hash = to_hex(fnv1a_64(current_text));
  const std::string new_hash = to_hex(fnv1a_64(snapshot_text));
  const std::string sanitized = sanitize_filename(rel_path);

  const fs::path diff_path = patches_dir / ("0_" + sanitized + ".diff");
  const fs::path base_path = base_dir / rel_path;
  const fs::path proposed_path = proposed_dir / rel_path;
  fs::create_directories(base_path.parent_path(), ec);
  fs::create_directories(proposed_path.parent_path(), ec);
  write_text_file(base_path, current_text);
  write_text_file(proposed_path, snapshot_text);
  write_text_file(diff_path, unified_diff(current_text, snapshot_text, rel_path));

  nlohmann::json summary;
  summary["run_id"] = run_id;
  summary["created_at"] = now_iso();
  summary["snapshot_run_id"] = snapshot_run_id;
  summary["snapshot_path"] = snapshot_path.generic_string();
  summary["target_path"] = rel_path;
  summary["files"] = nlohmann::json::array();
  nlohmann::json entry;
  entry["path"] = rel_path;
  entry["base_hash"] = base_hash;
  entry["new_hash"] = new_hash;
  entry["diff_path"] = diff_path.generic_string();
  entry["base_path"] = base_path.generic_string();
  entry["proposed_path"] = proposed_path.generic_string();
  summary["files"].push_back(entry);
  write_text_file(staging_dir / "summary.json", summary.dump(2));

  nlohmann::json run_info;
  run_info["run_id"] = run_id;
  run_info["created_at"] = now_iso();
  run_info["mode"] = "snapshot_restore";
  run_info["snapshot_run_id"] = snapshot_run_id;
  run_info["snapshot_path"] = snapshot_path.generic_string();
  run_info["target_path"] = rel_path;
  write_text_file(run_dir / "run_info.json", run_info.dump(2));

  nlohmann::json results;
  results["run_id"] = run_id;
  results["status"] = "ok";
  results["dry_run"] = true;
  results["applied"] = 0;
  results["conflicts"] = 0;
  results["conflict_files"] = nlohmann::json::array();
  results["staging_dir"] = staging_dir.generic_string();
  results["success"] = true;
  results["stage"] = "stage";
  results["error_code"] = "";
  results["error_message"] = "";
  results["conflict_detected"] = false;
  write_text_file(run_dir / "results.json", results.dump(2));

  result.success = true;
  result.run_id = run_id;
  result.run_dir = run_dir;
  result.staging_dir = staging_dir;
  return result;
#endif
}

SnapshotRestoreApplyResult apply_snapshot_restore(const fs::path& run_dir, bool dry_run) {
  SnapshotRestoreApplyResult result;
#if !RKG_ENABLE_DATA_JSON
  result.error_code = "json_disabled";
  result.error_message = "JSON disabled; snapshot restore unavailable";
  return result;
#else
  const fs::path summary_path = run_dir / "staged_patches" / "summary.json";
  nlohmann::json summary;
  std::ifstream in(summary_path);
  if (!in) {
    result.error_code = "summary_missing";
    result.error_message = "staged summary missing";
    return result;
  }
  in >> summary;
  if (!summary.contains("files") || !summary["files"].is_array() || summary["files"].empty()) {
    result.error_code = "summary_invalid";
    result.error_message = "staged summary invalid";
    return result;
  }
  const auto& file = summary["files"][0];
  const std::string rel_path = file.value("path", "");
  const std::string base_hash = file.value("base_hash", "");
  const std::string proposed_path = file.value("proposed_path", "");

  fs::path target = rel_path;
  if (!target.is_absolute()) {
    fs::path root = run_dir;
    root = root.parent_path();
    root = root.parent_path();
    root = root.parent_path();
    target = root / rel_path;
  }
  const std::string current_text = read_text_file(target);
  const std::string current_hash = to_hex(fnv1a_64(current_text));
  bool conflict_detected = false;
  std::vector<std::string> conflict_files;

  if (current_hash != base_hash) {
    conflict_detected = true;
    conflict_files.push_back(rel_path);
  }

  size_t applied = 0;
  if (!conflict_detected) {
    if (!dry_run) {
      std::error_code ec;
      fs::create_directories(target.parent_path(), ec);
      write_text_file(target, read_text_file(proposed_path));
    }
    applied = 1;
  }

  nlohmann::json results;
  results["run_id"] = summary.value("run_id", run_dir.filename().string());
  results["status"] = conflict_detected ? "conflicts" : "ok";
  results["dry_run"] = dry_run;
  results["applied"] = static_cast<int>(applied);
  results["conflicts"] = conflict_detected ? 1 : 0;
  results["conflict_files"] = conflict_files;
  results["staging_dir"] = (run_dir / "staged_patches").generic_string();
  results["success"] = !conflict_detected;
  results["stage"] = "apply";
  results["error_code"] = conflict_detected ? "conflict" : "";
  results["error_message"] = conflict_detected ? "content changed since snapshot staging" : "";
  results["conflict_detected"] = conflict_detected;
  write_text_file(run_dir / "results.json", results.dump(2));

  result.success = !conflict_detected;
  result.applied = applied;
  result.conflicts = conflict_detected ? 1 : 0;
  result.conflict_files = conflict_files;
  if (conflict_detected) {
    result.error_code = "conflict";
    result.error_message = "content changed since snapshot staging";
  }
  return result;
#endif
}

} // namespace rkg
