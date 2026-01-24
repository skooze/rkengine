# Technical Overview

This document summarizes the **current, working** state of RKG. It is intended as a factual map of
what exists today (Alpha 0.4+), not a design wishlist.

## What RKG Is
RKG is an AI-orchestrated game engine where **agents propose changes** and humans retain **full manual control**.
All game code and content are plain files. The CLI (`rkgctl`) validates and stages changes as patches, prints diffs,
and applies only after approval.

## Coordinate System + Units
- **Axes**: Y-up, Z-forward, X-right
- **Ground plane**: XZ at y=0
- **Units**: 1.0 = 1 meter
- **Angles**: radians (Transform.rotation is in radians)

## Repo Layout (Key Paths)
- `core/` — engine core (logging, ECS, plugin host, config, replay, content pack reader)
- `platform/` — SDL3 window/input, file watcher (inotify/ReadDirectoryChangesW/poll)
- `plugins/` — renderer, scripting, data, debug UI, AI/cloud stubs
- `projects/` — project roots (demo + template)
- `apps/rkgctl/` — compiled dev tooling (AI orchestration, content pipeline, packaging)
- `projects/demo_game/` — main demo executable + content
- `docs/` — architecture, orchestration, schemas, and references

## Runtime Architecture
### Core (small + stable)
- **Logging**: `rkg::log` with file + ring buffer
- **ECS**: entity registry + `Transform`
- **Plugin host**: static plugins today, dynamic-ready API
- **Config**: YAML/JSON project config
- **Replay hooks**: record/replay input streams
- **Content pack reader**: runtime reads `content.pack` (Pack v0)

### Platform Layer (SDL3)
- Window creation and input handling on Linux/Windows
- File watcher backend (Linux inotify, Windows ReadDirectoryChangesW, polling fallback)

### Plugins (most features live here)
**Renderer**
- **Vulkan**: Linux target, renders visible geometry in `demo_game`
- **D3D12**: Windows-only, renders a basic triangle/quad; resize supported
- **Null**: no-op fallback

**Debug UI (ImGui)**
- Vulkan + SDL3 only (D3D12 backend not implemented)
- Panels: Stats, World, Hot Reload, AI Orchestration, Console

**Scripting**
- Lua minimal
- JS/Python are stubs when runtimes are missing

**Data**
- YAML/JSON loaders
- SQLite for local task/audit storage (if present)

**AI / Cloud**
- Cloud stubs only; no runtime LLM calls

## Renderer Capabilities + Fallback
Renderer plugins expose caps (`DrawMesh`, `DrawPrimitive`, `DebugUiSupported`). The demo selects a renderer using
a deterministic fallback order:
1) Requested renderer (project.yaml)
2) Vulkan (if available)
3) D3D12 (Windows-only)
4) Null

If a renderer is not supported, the demo logs the fallback reason and continues.

## Project Structure
Each project lives under `projects/<name>/`:
- `project.yaml` — renderer selection, plugins, input map, initial level, dev_mode, ai config
- `src/` — project-specific C++ entry point
- `content/` — prefabs and levels
- `config/input.yaml` — action bindings (WASD, Escape, F5, F1)

`projects/demo_game` is the main executable (`rkg_demo_game`).

## Content Pipeline (Compiled)
`rkgctl` owns content validation/cooking:
- `content validate` checks prefabs/levels schema
- `content cook` creates `content.index.json` + `content.pack`
- `content watch` auto-recooks on changes

Cook outputs live under `build/content_cache/<project>/`. In dev_mode, the demo watches `cook_status.json` and
auto-reloads after a successful cook.

**Pack v0**
- `content.pack` is a deterministic, uncompressed pack
- Runtime loads from pack if present, otherwise uses cooked YAML/JSON mirror

## AI Orchestration Workflow
End-to-end dev loop:
1) **Context pack** (`rkgctl context dump`) captures file hashes + fingerprints
2) **Agent plan** (`rkgctl agent plan/apply`) requests a plan using OpenAI (or offline)
3) **Validation**: plan JSON validated against schema
4) **Patch-first staging**: diffs in `build/ai_staging/`
5) **Apply**: human approval required by default; conflicts detected
6) **Audit**: `build/ai_audit.log`, `build/ai_results.json`, `build/ai_runs/<run_id>/`
7) **Rollback**: `rkgctl rollback --last`

Run artifacts live under `build/ai_runs/<run_id>/` (run_info/results, staged_patches diffs, and optional snapshots),
and power the Editor Runs Browser for diff review, conflict visibility, cleanup, and snapshot restore workflows.

## Safety Model
- **Allowlist-only actions** (no arbitrary shell execution)
- **Conflict detection** via base-file hash checks
- **Context drift gate**: warns and blocks unless explicitly overridden

## Record/Replay Hooks
The engine supports recording input streams and replaying them deterministically. Use `rkgctl replay record/verify`
to generate or validate replays.

## Known Limitations / Stubs
- Debug UI only supports Vulkan (no D3D12 backend yet)
- JS/Python scripting runtimes are stubbed without a backing VM
- Cloud/Bedrock integrations are placeholders only
- Dynamic plugin loading is planned but not implemented

## Quickstarts (Commands Only)
### Linux build + run demo_game
```
cmake --preset linux-debug
cmake --build --preset linux-debug
./build/bin/rkg_demo_game
```

### Windows build + run demo_game
```
cmake --preset windows-debug
cmake --build --preset windows-debug
./build/bin/rkg_demo_game.exe
```

### Content dev loop (watch + auto-reload)
Terminal 1:
```
./build/bin/rkgctl content watch --project projects/demo_game --debounce-ms 300
```
Terminal 2:
```
./build/bin/rkg_demo_game
```

### AI-driven dev loop (OpenAI)
```
./build/bin/rkgctl agent plan --project projects/demo_game --goal "Add a demo NPC prefab"
./build/bin/rkgctl agent apply --project projects/demo_game --goal "Add a demo NPC prefab" --yes
./build/bin/rkgctl agent status --project projects/demo_game
```

## Further Reading
- `docs/ARCHITECTURE.md`
- `docs/AI_ORCHESTRATION.md`
- `docs/DEVELOPMENT.md`
- `docs/PLUGINS.md`
