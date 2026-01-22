#pragma once

#include <string>

namespace rkg {

struct AiResultsSummary {
  std::string run_id;
  std::string status;
  bool dry_run = true;
  int conflicts = 0;
  bool has_context_drift = false;
  bool drift_detected = false;
  int drift_added = 0;
  int drift_removed = 0;
  int drift_modified = 0;
  std::string drift_message;
};

bool parse_ai_results_summary(const std::string& json_text, AiResultsSummary& out, std::string& error);

} // namespace rkg
