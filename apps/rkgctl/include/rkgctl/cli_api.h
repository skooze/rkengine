#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif

struct StagedFile {
  std::filesystem::path path;
  std::string base_hash;
  std::string new_hash;
  std::filesystem::path diff_path;
  std::filesystem::path base_path;
  std::filesystem::path proposed_path;
};

struct StageResult {
  std::filesystem::path run_dir;
  std::vector<StagedFile> files;
  nlohmann::json summary;
  std::vector<std::string> errors;
};

struct ApplyOptions {
  bool dry_run = true;
  bool require_approval = true;
  bool force = false;
};

struct PlanValidationError {
  std::string keypath;
  std::string message;
};

bool validate_plan(const nlohmann::json& plan, std::string& error);
bool validate_plan_detailed(const nlohmann::json& plan,
                            std::vector<PlanValidationError>& errors,
                            bool strict_enums);
StageResult stage_plan(const nlohmann::json& plan, const std::filesystem::path& plan_path);
int apply_staged(const StageResult& stage, const ApplyOptions& opts);
int apply_plan(const nlohmann::json& plan, const std::filesystem::path& plan_path, const ApplyOptions& opts);
void write_ai_results_with_drift(const std::filesystem::path& run_dir,
                                 const std::string& run_id,
                                 const ApplyOptions& opts,
                                 int rc,
                                 const nlohmann::json& drift_info,
                                 const std::optional<std::string>& status_override = std::nullopt);

StageResult stage_commit_overrides(const std::filesystem::path& root,
                                   const std::filesystem::path& project_root,
                                   const std::filesystem::path& overrides_path,
                                   const std::optional<std::filesystem::path>& level_override,
                                   const std::optional<std::string>& entity_id,
                                   const std::optional<std::string>& entity_name,
                                   const std::filesystem::path& staging_dir,
                                   const std::string& run_id,
                                   std::string& error_code,
                                   std::string& error_message);
int apply_commit_overrides(const StageResult& stage,
                           const ApplyOptions& opts,
                           const std::filesystem::path& run_dir,
                           const std::string& run_id,
                           std::string& error_code,
                           std::string& error_message,
                           std::string& stage_name);

struct CommitOverridesOptions {
  ApplyOptions apply;
  bool stage_only = true;
  std::optional<std::filesystem::path> run_dir;
  std::optional<std::filesystem::path> apply_staged_dir;
  std::optional<std::filesystem::path> level_override;
  std::optional<std::string> entity_id;
  std::optional<std::string> entity_name;
};

int content_commit_overrides(const char* argv0,
                             const std::optional<std::filesystem::path>& project_override,
                             const std::filesystem::path& overrides_path,
                             const CommitOverridesOptions& opts);

int content_validate(const char* argv0, const std::optional<std::filesystem::path>& project_override);
int content_cook(const char* argv0,
                 const std::optional<std::filesystem::path>& project_override,
                 const std::optional<std::filesystem::path>& out_override);

int context_dump(const char* argv0,
                 const std::optional<std::filesystem::path>& project_override,
                 const std::filesystem::path& out_path,
                 size_t cap);
int context_diff(const char* argv0,
                 const std::filesystem::path& baseline_path,
                 const std::optional<std::filesystem::path>& project_override,
                 bool include_core,
                 bool include_plugins,
                 const std::optional<size_t>& cap_override);

bool build_openai_request_json(const nlohmann::json& schema,
                               const std::string& model,
                               const std::string& system_prompt,
                               const std::string& user_prompt,
                               nlohmann::json& out,
                               std::string& error);
bool extract_plan_json_from_openai_response(const nlohmann::json& response,
                                            std::string& out_plan_json,
                                            std::string& error);

bool load_run_summary(const std::filesystem::path& run_dir,
                      nlohmann::json& out,
                      std::string& error);

#if RKG_ENABLE_DATA_YAML
bool update_yaml_keypath(YAML::Node& root, const std::string& keypath, const nlohmann::json& value);
#endif
