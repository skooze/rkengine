# API Reference

This reference documents the public engine headers, plugin API, CLI surface, and key file formats.

## Core Public Headers
These are the headers intended for engine or plugin users:
- `core/include/rkg/log.h` — logging (`rkg::log::info/warn/error`)
- `core/include/rkg/ecs.h` — entity registry + `Transform`
- `core/include/rkg/plugin_api.h` — plugin API struct + `RendererCaps`
- `core/include/rkg/plugin_host.h` — plugin registration + lookup
- `core/include/rkg/renderer_select.h` — renderer fallback order
- `core/include/rkg/renderer_util.h` — renderer display names
- `core/include/rkg/content_pack.h` — Pack v0 reader
- `core/include/rkg/instructions.h` — runtime instruction parser/executor
- `core/include/rkg/replay.h` — record/replay hooks
- `core/include/rkg/paths.h` — repo/project path resolution
- `core/include/rkg/project.h` — project.yaml config
- `core/include/rkg/ai_results.h` — parsing `ai_results.json`

## Plugin API
All plugins expose a `RkgPluginApi` table:
```
struct RkgPluginApi {
  uint32_t api_version;
  const char* name;
  PluginType type;
  uint32_t caps;
  bool (*init)(void* host_context);
  void (*shutdown)();
  void (*update)(float dt_seconds);
  void (*on_window_resized)(int width, int height);
};
```

Dynamic loading (future) expects:
```
extern "C" RkgPluginApi* rkg_plugin_get_api(uint32_t host_api_version);
```

### RendererCaps
Defined in `core/include/rkg/plugin_api.h`:
- `DrawMesh`
- `DrawPrimitive`
- `DebugUiSupported`

## rkgctl CLI Surface
Core commands:
- `rkgctl content validate|cook|watch|list|dump-pack|commit-overrides`
- `rkgctl plan validate|summarize|schema`
- `rkgctl apply --plan <file>`
- `rkgctl agent status|plan|apply`
- `rkgctl agent openai plan|apply`
- `rkgctl agent offline plan|apply`
- `rkgctl context dump|diff`
- `rkgctl rollback --last`
- `rkgctl package`
- `rkgctl replay record|verify`

Artifacts:
- `build/ai_runs/<run_id>/` — run artifacts (context, plan, results)
- `build/ai_results.json` — latest summary
- `build/ai_audit.log` — JSON-lines audit

### content commit-overrides
```
rkgctl content commit-overrides --project <path> [--overrides <file>] [--level <path>] [--entity-id <id>] [--entity-name <name>] [--stage] [--run-dir <dir>]
rkgctl content commit-overrides --apply-staged <run_dir> [--yes] [--dry-run] [--force]
```
Stages patch files under `build/ai_runs/<run_id>/staged_patches/` (default is stage-only).
Review the diff, then apply explicitly with `--apply-staged <run_dir>` (re-checks base hashes and reports conflicts).
`--run-dir` allows deterministic staging directories for later apply. `--level` targets a specific level file
(relative to the project root). `--entity-id` stages only a single override entry (by id/key);
`--entity-name` is a fallback selector (less reliable; prefer ids).
`--force` applies staged patches even if base hashes changed, and records `forced_apply` + `conflict_detected`
in `results.json`. Forced apply writes snapshots under `build/ai_runs/<run_id>/snapshots/` and records
`snapshots_taken` + `snapshot_manifest_path` in results.
Uses per-level entity `id` values to update instance `transform` and `renderable` fields.
If `--overrides` is omitted, defaults to `<project>/editor_overrides.yaml`.

Results fields include:
- `conflict_files` (relative paths)
- `selector_type`/`selector_value`/`selector_warning`
- `snapshots_taken` + `snapshot_manifest_path` (forced apply only)

## File Formats
### project.yaml
Located at `projects/<name>/project.yaml`:
```
project:
  name: demo_game
  renderer: vulkan
  plugins:
    - debug_ui_imgui
  initial_level: content/levels/demo_level.yaml
  input_map: config/input.yaml
  dev_mode: true
  ai:
    provider: openai|offline
    model: gpt-5.2-codex
    base_url: https://api.openai.com
    templates_dir: docs/agent_templates/openai
    timeout_seconds: 60
```

### input.yaml
Located at `projects/<name>/config/input.yaml`:
```
input:
  actions:
    MoveForward: W
    MoveBack: S
    MoveLeft: A
    MoveRight: D
    Quit: Escape
    Reload: F5
    ToggleDebugUI: F1
```

### Prefab YAML
```
name: demo_cube
components:
  Transform:
    position: [0, 0, 0]
    rotation: [0, 0, 0]
    scale: [1, 1, 1]
```

### Level YAML
```
name: demo_level
entities:
  - name: demo_cube_1
    id: demo_cube_1
    prefab: demo_cube
    transform:
      position: [0, 0, 0]
      rotation: [0, 0, 0]
      scale: [1, 1, 1]
```
`id` is optional but recommended for editor overrides; if omitted, the entity name is used as the override key.

### Plan Schema
Canonical JSON schema: `docs/schemas/rkg_plan.schema.json`

### Run Info / Results Schemas
- `docs/schemas/rkg_run_info.schema.json`
- `docs/schemas/rkg_run_results.schema.json`

### content.index.json
Produced by `rkgctl content cook`:
```
{
  "project": "demo_game",
  "generated_at": "2024-01-01T00:00:00",
  "content_root": "build/content_cache/demo_game",
  "source_root": "projects/demo_game/content",
  "prefabs": [
    { "id": "...", "name": "...", "path": "...", "source_hash": "...", "mtime_epoch": 0 }
  ],
  "levels": [
    { "id": "...", "name": "...", "path": "...", "source_hash": "...", "mtime_epoch": 0,
      "dependencies": ["prefab_name"] }
  ],
  "pack_path": "build/content_cache/demo_game/content.pack"
}
```

### Pack v0 (content.pack)
Binary format:
- Header: magic `RKGP`, version `1`, `entry_count`, `index_offset`
- Entry table: `id`, `type`, `offset`, `size`, `path_len`, `path`
- Data blobs: canonical JSON bytes per asset

Type values:
- `1` = prefab
- `2` = level

### cook_status.json
Written by `rkgctl content watch`:
```
{
  "project": "demo_game",
  "timestamp": "2024-01-01T00:00:00",
  "validation_ok": true,
  "cook_ok": true,
  "output_dir": "build/content_cache/demo_game",
  "content_index_path": ".../content.index.json",
  "content_pack_path": ".../content.pack",
  "last_success_timestamp": "2024-01-01T00:00:00",
  "last_error": "",
  "changed_files": ["content/prefabs/demo_cube.yaml"]
}
```

### context_pack.json
Written by `rkgctl context dump` (used for drift detection and agent planning):
```
{
  "generated_at": "2024-01-01T00:00:00",
  "project_name": "demo_game",
  "project_root": "...",
  "engine_config": ".../config/engine.yaml",
  "content_root": ".../content",
  "fingerprint": {
    "root_path": "...",
    "git_head": "...",
    "tree_hash": "...",
    "tree_hashes": { "core": "...", "plugins": "...", "...": "..." }
  },
  "files": [ { "path": "content/prefabs/demo_cube.yaml", "hash": "..." } ],
  "files_cap": 500,
  "files_total_count": 1200,
  "files_included_count": 500,
  "files_truncated": true,
  "core_files": [ ... ],
  "core_files_cap": 500,
  "core_files_total_count": 800,
  "core_files_included_count": 500,
  "core_files_truncated": true,
  "plugins_files": [ ... ],
  "plugins_files_cap": 500,
  "plugins_files_total_count": 300,
  "plugins_files_included_count": 300,
  "plugins_files_truncated": false
}
```
