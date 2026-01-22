#include "rkg/ecs.h"
#include "rkg/ai_results.h"
#include "rkg/log.h"
#include "rkg/paths.h"
#include "rkg/renderer_select.h"
#include "rkg/renderer_util.h"
#include "rkgctl/cli_api.h"

#include <nlohmann/json.hpp>

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;
using json = nlohmann::json;

bool write_text(const fs::path& path, const std::string& contents) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  if (!out) return false;
  out << contents;
  return true;
}

std::string read_text(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

int main(int argc, char** argv) {
  const auto paths = rkg::resolve_paths(argc > 0 ? argv[0] : nullptr, std::nullopt, "demo_game");
  rkg::log::init("rkg_tests", paths.root);

  int failures = 0;

  // Test: plan validation.
  {
    json plan;
    plan["tasks"] = json::array();
    plan["tasks"].push_back({{"id", "t1"}, {"type", "content_change"}});
    std::string error;
    if (!validate_plan(plan, error)) {
      std::cerr << "plan validation failed unexpectedly\n";
      ++failures;
    }
    json bad;
    if (validate_plan(bad, error)) {
      std::cerr << "plan validation should have failed\n";
      ++failures;
    }
  }

#if RKG_ENABLE_DATA_JSON
  // Test: cook_status parsing (schema presence).
  {
    const std::string status_text =
        R"({"cook_ok":true,"last_success_timestamp":"2024-01-01T00:00:00","content_index_path":"out/content.index.json","content_pack_path":"out/content.pack","last_error":""})";
    json status = json::parse(status_text);
    if (!status.value("cook_ok", false)) {
      std::cerr << "cook_status cook_ok missing\n";
      ++failures;
    }
    if (status.value("last_success_timestamp", "").empty()) {
      std::cerr << "cook_status last_success_timestamp missing\n";
      ++failures;
    }
    if (status.value("content_index_path", "").empty() || status.value("content_pack_path", "").empty()) {
      std::cerr << "cook_status paths missing\n";
      ++failures;
    }
  }
#endif

  // Test: renderer fallback order.
  {
    const auto order = rkg::build_renderer_fallback_order("d3d12");
    if (order.empty() || order.front() != "renderer_d3d12") {
      std::cerr << "renderer fallback order invalid\n";
      ++failures;
    }
    bool has_null = false;
    for (const auto& item : order) {
      if (item == "renderer_null") {
        has_null = true;
      }
    }
    if (!has_null) {
      std::cerr << "renderer fallback missing null\n";
      ++failures;
    }
  }

  // Test: renderer display name mapping.
  {
    if (rkg::renderer_display_name("renderer_vulkan") != "Vulkan") {
      std::cerr << "renderer display name (vulkan) invalid\n";
      ++failures;
    }
    if (rkg::renderer_display_name("renderer_d3d12") != "D3D12") {
      std::cerr << "renderer display name (d3d12) invalid\n";
      ++failures;
    }
    if (rkg::renderer_display_name("renderer_null") != "Null") {
      std::cerr << "renderer display name (null) invalid\n";
      ++failures;
    }
  }

#if RKG_ENABLE_DATA_JSON
  // Test: plan schema file exists.
  {
    const fs::path schema_path = paths.root / "docs" / "schemas" / "rkg_plan.schema.json";
    const auto schema_text = read_text(schema_path);
    if (schema_text.empty()) {
      std::cerr << "plan schema missing\n";
      ++failures;
    } else {
      const auto schema = json::parse(schema_text, nullptr, false);
      if (schema.is_discarded() || !schema.contains("properties")) {
        std::cerr << "plan schema invalid\n";
        ++failures;
      }
    }
  }

  // Test: run_info/results schema docs exist and include expected fields.
  {
    const fs::path run_info_path = paths.root / "docs" / "schemas" / "rkg_run_info.schema.json";
    const fs::path run_results_path = paths.root / "docs" / "schemas" / "rkg_run_results.schema.json";
    const auto run_info_text = read_text(run_info_path);
    const auto run_results_text = read_text(run_results_path);
    if (run_info_text.empty() || run_results_text.empty()) {
      std::cerr << "run_info/results schema missing\n";
      ++failures;
    } else {
      const auto run_info = json::parse(run_info_text, nullptr, false);
      const auto run_results = json::parse(run_results_text, nullptr, false);
      if (run_info.is_discarded() || run_results.is_discarded()) {
        std::cerr << "run_info/results schema invalid\n";
        ++failures;
      } else {
        const auto info_props = run_info.value("properties", json::object());
        const auto res_props = run_results.value("properties", json::object());
        if (!info_props.contains("run_id") || !info_props.contains("mode") ||
            !info_props.contains("provider") || !info_props.contains("model")) {
          std::cerr << "run_info schema missing fields\n";
          ++failures;
        }
        if (!res_props.contains("run_id") || !res_props.contains("status") ||
            !res_props.contains("context_drift")) {
          std::cerr << "run_results schema missing fields\n";
          ++failures;
        }
      }
    }
  }

  // Test: OpenAI request builder + response extraction.
  {
    json schema;
    schema["type"] = "object";
    schema["properties"]["tasks"] = json::object();
    schema["required"] = json::array({"tasks"});
    json req;
    std::string error;
    if (!build_openai_request_json(schema, "gpt-5.2-codex", "sys", "user", req, error)) {
      std::cerr << "openai request build failed\n";
      ++failures;
    } else {
      if (!req.contains("text")) {
        std::cerr << "openai request missing text.format\n";
        ++failures;
      }
    }

    json response;
    response["output_text"] = "{\"tasks\":[]}";
    std::string plan_json;
    if (!extract_plan_json_from_openai_response(response, plan_json, error)) {
      std::cerr << "openai response extract failed\n";
      ++failures;
    }
    json response2;
    response2["output"] = json::array(
        {{{"content", json::array({{{"type", "output_text"}, {"text", "{\"tasks\":[]}"}}})}}});
    if (!extract_plan_json_from_openai_response(response2, plan_json, error)) {
      std::cerr << "openai response extract (output content) failed\n";
      ++failures;
    }
    json response3;
    response3["choices"] = json::array({{{"message", {{"content", "{\"tasks\":[]}"}}}}});
    if (!extract_plan_json_from_openai_response(response3, plan_json, error)) {
      std::cerr << "openai response extract (choices) failed\n";
      ++failures;
    }
    json response4;
    response4["output"] = json::array(
        {{{"tool_calls", json::array({{{"arguments", "{\"tasks\":[]}"}}})}}});
    if (!extract_plan_json_from_openai_response(response4, plan_json, error)) {
      std::cerr << "openai response extract (tool_calls) failed\n";
      ++failures;
    }
  }

  // Test: plan validation error count.
  {
    json bad;
    bad["tasks"] = json::array({json::object()});
    std::vector<PlanValidationError> errors;
    if (validate_plan_detailed(bad, errors, true)) {
      std::cerr << "bad plan unexpectedly valid\n";
      ++failures;
    }
    if (errors.size() < 1) {
      std::cerr << "plan errors missing\n";
      ++failures;
    }
  }

  // Test: strict enum validation catches invalid task type.
  {
    json bad;
    bad["tasks"] = json::array({{{"id", "t1"}, {"type", "bad_type"}}});
    std::vector<PlanValidationError> errors;
    if (validate_plan_detailed(bad, errors, true)) {
      std::cerr << "enum validation should have failed\n";
      ++failures;
    } else {
      bool found = false;
      for (const auto& err : errors) {
        if (err.keypath.find(".type") != std::string::npos &&
            err.message.find("allowed") != std::string::npos) {
          found = true;
          break;
        }
      }
      if (!found) {
        std::cerr << "enum validation error missing allowed values\n";
        ++failures;
      }
    }
  }

  // Test: context dump includes fingerprint and diff runs.
  {
    const fs::path tmp_project = paths.root / "build" / "test_ctx_project";
    fs::remove_all(tmp_project);
    fs::create_directories(tmp_project / "content");
    fs::create_directories(tmp_project / "config");
    write_text(tmp_project / "project.yaml", "project:\n  name: ctx_test\n");
    write_text(tmp_project / "content/a.txt", "a");
    write_text(tmp_project / "content/b.txt", "b");
    write_text(tmp_project / "config/input.yaml", "input:\n  actions:\n    Quit: Escape\n");
    const fs::path baseline = tmp_project / "baseline.json";
    if (context_dump(argv[0], tmp_project, baseline, 1) != 0) {
      std::cerr << "context dump failed\n";
      ++failures;
    } else {
      const auto text = read_text(baseline);
      const auto ctx = json::parse(text, nullptr, false);
      if (ctx.is_discarded() || !ctx.contains("fingerprint")) {
        std::cerr << "context fingerprint missing\n";
        ++failures;
      }
      if (!ctx.contains("files") || !ctx.contains("files_cap")) {
        std::cerr << "context files list missing\n";
        ++failures;
      } else {
        const size_t cap = ctx.value("files_cap", 0);
        if (cap == 0 || !ctx["files"].is_array() || ctx["files"].size() > cap) {
          std::cerr << "context files cap invalid\n";
          ++failures;
        }
        if (!ctx.value("files_truncated", false)) {
          std::cerr << "context files should be truncated\n";
          ++failures;
        }
        const size_t total = ctx.value("files_total_count", 0);
        const size_t included = ctx.value("files_included_count", 0);
        if (total == 0 || included == 0 || included > total) {
          std::cerr << "context files counts invalid\n";
          ++failures;
        }
      }
      if (!ctx.contains("core_files") || !ctx.contains("plugins_files")) {
        std::cerr << "context core/plugins file lists missing\n";
        ++failures;
      } else {
        const size_t core_cap = ctx.value("core_files_cap", 0);
        const size_t plugins_cap = ctx.value("plugins_files_cap", 0);
        if (core_cap == 0 || plugins_cap == 0 ||
            !ctx["core_files"].is_array() || !ctx["plugins_files"].is_array() ||
            ctx["core_files"].size() > core_cap || ctx["plugins_files"].size() > plugins_cap) {
          std::cerr << "context core/plugins caps invalid\n";
          ++failures;
        }
        const size_t core_total = ctx.value("core_files_total_count", 0);
        const size_t core_included = ctx.value("core_files_included_count", 0);
        const size_t plugins_total = ctx.value("plugins_files_total_count", 0);
        const size_t plugins_included = ctx.value("plugins_files_included_count", 0);
        if ((core_total > 0 && core_included == 0) || core_included > core_total ||
            (plugins_total > 0 && plugins_included == 0) || plugins_included > plugins_total) {
          std::cerr << "context core/plugins counts invalid\n";
          ++failures;
        }
      }
      if (context_diff(argv[0], baseline, tmp_project, false, false, std::optional<size_t>{1}) != 0) {
        std::cerr << "context diff failed\n";
        ++failures;
      }
    }
  }

  // Test: agent run summary parsing.
  {
    const fs::path run_dir = paths.root / "build" / "test_ai_run_status" / "run_1";
    fs::remove_all(run_dir);
    fs::create_directories(run_dir);
    json run_info;
    run_info["run_id"] = "run_1";
    run_info["created_at"] = "2024-01-01T00:00:00";
    run_info["goal"] = "test goal";
    run_info["mode"] = "apply";
    run_info["plan_path"] = (run_dir / "plan.json").generic_string();
    run_info["provider"] = "openai";
    run_info["model"] = "gpt-5.2-codex";
    run_info["base_url"] = "https://api.openai.com";
    run_info["templates_dir"] = "docs/agent_templates/openai";
    run_info["timeout_seconds"] = 60;
    run_info["strict_enums"] = true;
    write_text(run_dir / "run_info.json", run_info.dump(2));

    json results;
    results["run_id"] = "run_1";
    results["status"] = "ok";
    results["dry_run"] = false;
    results["conflicts"] = 0;
    results["context_drift"]["drift_detected"] = false;
    write_text(run_dir / "results.json", results.dump(2));

    json summary;
    std::string error;
    if (!load_run_summary(run_dir, summary, error)) {
      std::cerr << "run summary failed: " << error << "\n";
      ++failures;
    } else {
      if (summary.value("run_id", "") != "run_1" || summary.value("goal", "").empty()) {
        std::cerr << "run summary missing fields\n";
        ++failures;
      }
      if (!summary.contains("context_drift")) {
        std::cerr << "run summary missing drift info\n";
        ++failures;
      }
    }
  }

  // Test: drift info is written to results.json.
  {
    const fs::path run_dir = paths.root / "build" / "test_ai_results" / "run_2";
    fs::remove_all(run_dir);
    fs::create_directories(run_dir);
    ApplyOptions opts;
    opts.dry_run = false;
    opts.require_approval = false;
    json drift;
    drift["baseline_fingerprint"] = json::object();
    drift["current_fingerprint"] = json::object();
    drift["drift_detected"] = true;
    drift["changed_counts"]["added"] = 1;
    drift["changed_counts"]["removed"] = 0;
    drift["changed_counts"]["modified"] = 2;
    write_ai_results_with_drift(run_dir, "run_2", opts, 0, drift);
    const auto results_text = read_text(run_dir / "results.json");
    const auto parsed = json::parse(results_text, nullptr, false);
    if (parsed.is_discarded() || !parsed.contains("context_drift")) {
      std::cerr << "results missing drift info\n";
      ++failures;
    }
  }

  // Test: ai_results summary parsing.
  {
    const std::string text =
        R"({"run_id":"r1","status":"ok","dry_run":false,"conflicts":0,"context_drift":{"drift_detected":true,"changed_counts":{"added":1,"removed":2,"modified":3},"message":"tree hash mismatch"}})";
    rkg::AiResultsSummary summary{};
    std::string error;
    if (!rkg::parse_ai_results_summary(text, summary, error)) {
      std::cerr << "ai_results parse failed: " << error << "\n";
      ++failures;
    } else {
      if (!summary.drift_detected || summary.drift_added != 1 || summary.drift_removed != 2 ||
          summary.drift_modified != 3) {
        std::cerr << "ai_results drift parse mismatch\n";
        ++failures;
      }
    }
  }
#endif

#if RKG_ENABLE_DATA_YAML
  // Test: YAML update helper.
  {
    YAML::Node root;
    root["project"]["initial_level"] = "old";
    json value = "new_level.yaml";
    update_yaml_keypath(root, "project.initial_level", value);
    if (root["project"]["initial_level"].as<std::string>() != "new_level.yaml") {
      std::cerr << "yaml update failed\n";
      ++failures;
    }
  }
#endif

#if RKG_ENABLE_DATA_YAML
  // Test: pack determinism (same input -> same pack bytes).
  {
    const fs::path tmp_project = paths.root / "build" / "test_pack_project";
    fs::remove_all(tmp_project);
    fs::create_directories(tmp_project / "content" / "prefabs");
    fs::create_directories(tmp_project / "content" / "levels");
    write_text(tmp_project / "project.yaml", "project:\n  name: pack_test\n");
    write_text(tmp_project / "content/prefabs/prefab.yaml",
               "name: prefab\ncomponents: {}\n");
    write_text(tmp_project / "content/levels/level.yaml",
               "name: level\nentities:\n  - name: e1\n    prefab: prefab\n");

    const fs::path out_a = tmp_project / "out_a";
    const fs::path out_b = tmp_project / "out_b";
    if (content_cook(argv[0], tmp_project, out_a) != 0 ||
        content_cook(argv[0], tmp_project, out_b) != 0) {
      std::cerr << "content cook failed\n";
      ++failures;
    } else {
      const auto pack_a = read_text(out_a / "content.pack");
      const auto pack_b = read_text(out_b / "content.pack");
      if (pack_a.empty() || pack_b.empty() || pack_a != pack_b) {
        std::cerr << "pack determinism failed\n";
        ++failures;
      }
    }
  }
#endif

  // Test: content validation (missing prefab ref).
  {
    const fs::path tmp_project = paths.root / "build" / "test_tmp_project";
    fs::remove_all(tmp_project);
    fs::create_directories(tmp_project / "content" / "prefabs");
    fs::create_directories(tmp_project / "content" / "levels");
    write_text(tmp_project / "project.yaml", "project:\n  name: test_project\n");
    write_text(tmp_project / "content/prefabs/prefab.yaml",
               "name: prefab\ncomponents: {}\n");
    write_text(tmp_project / "content/levels/level.yaml",
               "name: level\nentities:\n  - name: e1\n    prefab: missing_prefab\n");
    if (content_validate(argv[0], tmp_project) == 0) {
      std::cerr << "content validation should have failed\n";
      ++failures;
    }
  }

  // Test: patch apply + conflict detection.
  {
    const fs::path tmp_dir = paths.root / "build" / "test_tmp_patches";
    fs::remove_all(tmp_dir);
    fs::create_directories(tmp_dir);
    const fs::path plan_path = tmp_dir / "plan.json";
    json plan;
    plan["tasks"] = json::array();
    plan["tasks"].push_back({
        {"id", "t1"},
        {"type", "content_change"},
        {"actions",
         json::array({{{"action", "write_text_file"},
                       {"path", "build/test_tmp_patches/file.txt"},
                       {"mode", "overwrite"},
                       {"contents", "hello\n"}}})}});
    write_text(plan_path, plan.dump(2));

    const StageResult stage = stage_plan(plan, plan_path);
    if (!stage.errors.empty() || stage.files.empty()) {
      std::cerr << "stage_plan failed\n";
      ++failures;
    } else {
      // Apply should succeed.
      ApplyOptions opts;
      opts.dry_run = false;
      opts.require_approval = false;
      const int rc_ok = apply_staged(stage, opts);
      if (rc_ok != 0) {
        std::cerr << "apply_staged failed unexpectedly\n";
        ++failures;
      }

      // Conflict check: modify file after staging and re-apply.
      write_text(paths.root / "build/test_tmp_patches/file.txt", "modified\n");
      const int rc_conflict = apply_staged(stage, opts);
      if (rc_conflict == 0) {
        std::cerr << "conflict was not detected\n";
        ++failures;
      }
    }
  }

  rkg::log::shutdown();
  return failures == 0 ? 0 : 1;
}
