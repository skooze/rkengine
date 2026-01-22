# Development

## Dependencies
- CMake 3.20+
- C++20 compiler
- SDL3 development package
- Vulkan SDK (for Vulkan renderer)
- yaml-cpp
- nlohmann/json (header-only)
- sqlite3 dev package (optional)
- Lua / QuickJS / Duktape (optional)
- ImGui (auto-fetched when RKG_ENABLE_IMGUI=ON)
- libcurl (optional; required for OpenAI provider in rkgctl)

Note: CMake will try to FetchContent `yaml-cpp` and `nlohmann/json` if system packages are not found.

## Configure + Build (Presets)
Linux:
```
cmake --preset linux-debug
cmake --build --preset linux-debug
```

Windows (example using Ninja):
```
cmake --preset windows-debug
cmake --build --preset windows-debug
```
Windows notes:
- Install SDL3 and make sure CMake can find it (e.g., set `SDL3_DIR` or use vcpkg).
- Install the Vulkan SDK if you want the Vulkan renderer.
- D3D12 requires the Windows SDK and `d3dcompiler_47` (usually present with the SDK).
- If D3D12 init fails, the demo will fall back to the null renderer and log why.
- D3D12 resize is supported; resizing the demo window should not crash.
- If you want `rkgctl agent openai ...`, ensure libcurl is available (e.g., via vcpkg).

Compile commands are generated at `build/compile_commands.json`.

## Projects
Demo project lives at `projects/demo_game`. To create a new project:
```
./build/bin/rkgctl new project mygame --template template_game
```

## Run Demo Game
```
./build/bin/rkg_demo_game
```
Runtime loads cooked content from `build/content_cache/<project>` when present. If missing and `project.dev_mode: true`, it falls back to raw content and logs a warning.
Hot reload watches `projects/demo_game/content`; F5 reloads the current content root and warns if cooked content is stale.
Input map lives at `projects/demo_game/config/input.yaml` (WASD move, ESC quit, F1 toggle debug UI, F5 reload).
Logs are written to `build/logs/`.
To test D3D12 on Windows, set `renderer: d3d12` in `projects/demo_game/project.yaml`.
Renderer fallback (runtime only): requested → Vulkan → D3D12 → Null, with logs explaining the choice.

## Run Minimal App (Sanity)
```
./build/bin/rkg_minimal_app
```

## Run Orchestration Demo
```
./build/bin/rkgctl apply --plan docs/examples/plan_spawn_cube.json --yes
./build/bin/rkg_minimal_app
```

## OpenAI Setup (Dev-Time)
Set your key before running OpenAI agent commands:
```
export OPENAI_API_KEY="..."
export OPENAI_MODEL="gpt-5.2-codex"   # optional
export OPENAI_BASE_URL="https://api.openai.com"  # optional
```
Example:
```
./build/bin/rkgctl agent openai plan --project projects/demo_game --goal "Add a demo NPC prefab" --out build/ai_runs/demo_plan.json
./build/bin/rkgctl agent openai apply --project projects/demo_game --goal "Add a demo NPC prefab" --yes
```

Quickstart (config‑driven):
```
./build/bin/rkgctl agent plan --project projects/demo_game --goal "Add a demo NPC prefab"
./build/bin/rkgctl agent apply --project projects/demo_game --goal "Add a demo NPC prefab" --yes
```

Force provider via CLI:
```
./build/bin/rkgctl agent plan --provider offline --in docs/examples/plan_spawn_cube.json --project projects/demo_game
```

Offline provider:
```
./build/bin/rkgctl agent offline plan --in docs/examples/plan_spawn_cube.json --project projects/demo_game
./build/bin/rkgctl agent offline apply --in docs/examples/plan_spawn_cube.json --project projects/demo_game --yes
```

Inspect last run / status:
```
./build/bin/rkgctl agent status --project projects/demo_game
./build/bin/rkgctl agent status --project projects/demo_game --runs 5 --json
```
Artifacts are stored under `build/ai_runs/<run_id>/` with a summary in `build/ai_results.json`.

Context diff with core/plugins:
```
./build/bin/rkgctl context diff --baseline build/ai_runs/<run_id>/context_pack.json --project projects/demo_game --include-core --include-plugins
```
Control context file list cap (default 500):
```
./build/bin/rkgctl context dump --project projects/demo_game --out build/context_pack.json --cap 200
./build/bin/rkgctl context diff --baseline build/context_pack.json --project projects/demo_game --cap 200
```

## Content Pipeline (rkgctl)
Validate and cook project content:
```
./build/bin/rkgctl content validate --project projects/demo_game
./build/bin/rkgctl content cook --project projects/demo_game
./build/bin/rkgctl content list --cooked build/content_cache/demo_game
```
Cook outputs a `content.index.json` plus a mirrored `prefabs/` and `levels/` tree under the cooked directory.

Auto-recook (recommended dev loop):
```
./build/bin/rkgctl content watch --project projects/demo_game --debounce-ms 300
```
Run the demo in another terminal; with `project.dev_mode: true`, the demo auto-reloads when cook_status changes (no F5 required).
`cook_status.json` lives under `build/content_cache/<project>/` and contains the last cook result + paths.

Pack v0:
- `rkgctl content cook` emits `content.pack` alongside `content.index.json`.
- Runtime prefers pack when present; it falls back to the cooked YAML mirror.
Inspect a pack:
```
./build/bin/rkgctl content dump-pack --pack build/content_cache/demo_game/content.pack
```

## Packaging
```
./build/bin/rkgctl package --project projects/demo_game
```

## Replay (Deterministic)
Record and replay from the demo game:
```
./build/bin/rkg_demo_game --record build/replays/baseline.json --fixed-dt 0.016
./build/bin/rkg_demo_game --replay build/replays/baseline.json --fixed-dt 0.016
```
Helper commands (print instructions):
```
./build/bin/rkgctl replay record --project projects/demo_game --out build/replays/baseline.json
./build/bin/rkgctl replay verify --project projects/demo_game --replay build/replays/baseline.json
```

## Optional Python (Tests/Tools Only)
RKG does **not** require Python to build or run. If you want optional tests/tools:
```
conda env create -f tools/environment.yml
conda activate rkg-dev
pytest tools/python_optional/tests
```
