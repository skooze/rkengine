#include "rkg/run_cleanup.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <system_error>

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

namespace rkg {
namespace fs = std::filesystem;

struct RunEntry {
  fs::path run_dir;
  fs::file_time_type sort_time;
  bool has_snapshots = false;
};

static bool read_snapshots_taken(const fs::path& results_path) {
#if RKG_ENABLE_DATA_JSON
  std::ifstream in(results_path);
  if (!in) return false;
  nlohmann::json j;
  in >> j;
  return j.value("snapshots_taken", false);
#else
  return false;
#endif
}

static bool run_has_snapshots(const fs::path& run_dir) {
  std::error_code ec;
  const fs::path manifest = run_dir / "snapshots" / "manifest.json";
  if (fs::exists(manifest, ec) && !ec) {
    return true;
  }
  const fs::path results_path = run_dir / "results.json";
  if (fs::exists(results_path, ec) && !ec) {
    return read_snapshots_taken(results_path);
  }
  return false;
}

static fs::file_time_type run_timestamp(const fs::path& run_dir) {
  std::error_code ec;
  const fs::path results_path = run_dir / "results.json";
  if (fs::exists(results_path, ec) && !ec) {
    auto ts = fs::last_write_time(results_path, ec);
    if (!ec) return ts;
  }
  const fs::path info_path = run_dir / "run_info.json";
  if (fs::exists(info_path, ec) && !ec) {
    auto ts = fs::last_write_time(info_path, ec);
    if (!ec) return ts;
  }
  auto ts = fs::last_write_time(run_dir, ec);
  if (ec) {
    return fs::file_time_type::clock::now();
  }
  return ts;
}

RunCleanupResult collect_run_cleanup_candidates(const fs::path& root,
                                                const RunCleanupOptions& options,
                                                const fs::path& selected_run_dir) {
  RunCleanupResult result;
  const fs::path runs_root = root / "build" / "ai_runs";
  std::error_code ec;
  if (!fs::exists(runs_root, ec) || ec) {
    return result;
  }

  std::vector<RunEntry> entries;
  for (auto it = fs::directory_iterator(runs_root, ec); it != fs::directory_iterator(); ++it) {
    if (ec) break;
    if (!it->is_directory()) continue;
    RunEntry entry;
    entry.run_dir = it->path();
    entry.sort_time = run_timestamp(entry.run_dir);
    entry.has_snapshots = run_has_snapshots(entry.run_dir);
    entries.push_back(entry);
  }

  std::sort(entries.begin(), entries.end(), [](const RunEntry& a, const RunEntry& b) {
    return a.sort_time > b.sort_time;
  });

  const size_t keep_last = options.keep_last == 0 ? 0 : options.keep_last;
  const int older_days = std::max(0, options.older_than_days);
  const auto now = std::chrono::system_clock::now();
  const auto cutoff = now - std::chrono::hours(24 * older_days);

  for (size_t i = 0; i < entries.size(); ++i) {
    const auto& entry = entries[i];
    if (!selected_run_dir.empty() && entry.run_dir == selected_run_dir) {
      continue;
    }
    if (entry.has_snapshots) {
      continue;
    }
    const bool beyond_keep = (keep_last > 0) && (i >= keep_last);
    bool too_old = false;
    if (older_days > 0) {
      const auto sys_time = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
          entry.sort_time - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
      too_old = sys_time < cutoff;
    }
    if (beyond_keep || too_old) {
      result.candidates.push_back(entry.run_dir);
    }
  }

  return result;
}

} // namespace rkg
