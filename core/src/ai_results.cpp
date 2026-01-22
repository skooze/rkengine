#include "rkg/ai_results.h"

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

namespace rkg {

bool parse_ai_results_summary(const std::string& json_text, AiResultsSummary& out, std::string& error) {
#if !RKG_ENABLE_DATA_JSON
  (void)json_text;
  (void)out;
  error = "json disabled";
  return false;
#else
  if (json_text.empty()) {
    error = "empty json";
    return false;
  }
  const auto j = nlohmann::json::parse(json_text, nullptr, false);
  if (j.is_discarded() || !j.is_object()) {
    error = "invalid json";
    return false;
  }

  out = {};
  out.run_id = j.value("run_id", "");
  out.status = j.value("status", "unknown");
  out.dry_run = j.value("dry_run", true);
  out.conflicts = j.value("conflicts", 0);

  if (j.contains("context_drift") && j["context_drift"].is_object()) {
    const auto& drift = j["context_drift"];
    out.has_context_drift = true;
    out.drift_detected = drift.value("drift_detected", false);
    out.drift_message = drift.value("message", "");
    if (drift.contains("changed_counts") && drift["changed_counts"].is_object()) {
      const auto& counts = drift["changed_counts"];
      out.drift_added = counts.value("added", 0);
      out.drift_removed = counts.value("removed", 0);
      out.drift_modified = counts.value("modified", 0);
    }
  }
  return true;
#endif
}

} // namespace rkg
