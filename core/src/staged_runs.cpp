#include "rkg/staged_runs.h"

#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <system_error>

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

namespace rkg {
namespace fs = std::filesystem;

#if !RKG_ENABLE_DATA_JSON
static std::string read_text_file(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
#endif

static bool results_is_stage(const fs::path& results_path) {
#if RKG_ENABLE_DATA_JSON
  std::ifstream in(results_path);
  if (!in) return false;
  nlohmann::json results;
  in >> results;
  return results.value("stage", "") == "stage";
#else
  const std::string text = read_text_file(results_path);
  return text.find("\"stage\":\"stage\"") != std::string::npos;
#endif
}

std::optional<fs::path> find_latest_staged_run_dir(const fs::path& root, std::string* error) {
  if (error) error->clear();
  const fs::path run_root = root / "build" / "ai_runs";
  std::error_code ec;
  if (!fs::exists(run_root, ec) || ec) {
    return std::nullopt;
  }

  std::optional<fs::path> best;
  std::optional<fs::file_time_type> best_time;

  for (const auto& entry : fs::directory_iterator(run_root, ec)) {
    if (ec) break;
    if (!entry.is_directory()) continue;
    const fs::path run_dir = entry.path();
    const fs::path staging_summary = run_dir / "staged_patches" / "summary.json";
    const fs::path results_path = run_dir / "results.json";
    if (!fs::exists(staging_summary, ec) || ec) continue;
    if (!fs::exists(results_path, ec) || ec) continue;
    if (!results_is_stage(results_path)) continue;

    std::error_code time_ec;
    const auto ts = fs::last_write_time(results_path, time_ec);
    if (time_ec) continue;
    if (!best_time.has_value() || ts > best_time.value()) {
      best_time = ts;
      best = run_dir;
    }
  }

  return best;
}

} // namespace rkg
