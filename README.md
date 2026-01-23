# RKG (rkengine) — AI-Orchestrated Game Dev, Manual Control First

RKG is a small, modular game engine where **agents propose changes** and humans keep full control.
The compiled CLI (`rkgctl`) generates and validates **Dev Task Graph** plans, stages **patches**,
prints diffs, and applies changes only after approval. The runtime is intentionally minimal and
stable; most features live in plugins. The current focus is **Linux + Windows** development with a
working demo game, a deterministic content pipeline, and a safe dev-time AI workflow.

**What’s implemented today**
- Vulkan renderer on Linux (visible geometry in demo)
- D3D12 renderer on Windows (triangle/quad + resize)
- SDL3 platform + input + hot reload support
- Content pipeline: validate/cook/watch + Pack v0
- AI orchestration: OpenAI (Responses API) + offline plans, patch-first, conflicts, rollback, audit
- Debug UI (ImGui) on Vulkan only

**Known limitations / stubs**
- Debug UI does **not** work on D3D12 yet (Vulkan-only)
- JS/Python scripting runtimes are stubbed when VM backends are unavailable
- Cloud/Bedrock runtime integration is not implemented

## Editor (primary entrypoint)
The Editor is the intended first-run experience. It hosts the demo scene in-process, exposes
Play/Pause/Step/Stop, and embeds the chat-driven plan/apply workflow.
```
cmake --preset linux-debug
cmake --build --preset linux-debug
./build/bin/rkg_editor --project projects/demo_game
```
What you’ll see in the Editor:
- A real viewport panel rendering content-driven meshes (Vulkan/Linux)
- Click-to-select with a highlighted entity + Scene/Inspector sync
- Inspector edits with undo/redo and per-project overrides (Save/Revert)
- Manual edits can be staged to a diff, reviewed, then applied back to content
- Runs Browser unifies AI + commit-overrides history with diffs, conflicts, and snapshots
- Edit-mode camera orbit/zoom with viewport focus gating
- Play-mode movement (WASD) when the viewport is focused

Manual edits workflow (in-editor):
- Save overrides -> Stage diff -> Review in Diff Preview -> Apply staged patch
- Cook/reload if raw content changed
Notes:
- Diff Preview auto-loads the last staged run on startup
- Stage Selected Only targets a single entity override
- Apply Staged (Force) is available with explicit confirmation and snapshots current files for audit

## Quickstart — Linux
```
cmake --preset linux-debug
cmake --build --preset linux-debug
./build/bin/rkg_demo_game
```

## Quickstart — Windows (deps + build)
Requires Visual Studio 2022 Build Tools + Windows SDK, CMake, Ninja.
Optional: libcurl (OpenAI provider in rkgctl).
```
cmake --preset windows-debug
cmake --build --preset windows-debug
./build/bin/rkg_demo_game.exe
```

## Content dev loop (watch + auto-reload)
Terminal 1:
```
./build/bin/rkgctl content watch --project projects/demo_game --debounce-ms 300
```
Terminal 2:
```
./build/bin/rkg_demo_game
```
With `dev_mode: true`, the demo auto-reloads on `cook_status.json` changes.

## AI dev loop (OpenAI + artifacts)
```
./build/bin/rkgctl agent plan --project projects/demo_game --goal "Add a demo NPC prefab"
./build/bin/rkgctl agent apply --project projects/demo_game --goal "Add a demo NPC prefab" --yes
./build/bin/rkgctl agent status --project projects/demo_game
```
Artifacts are saved under `build/ai_runs/<run_id>/` and summarized in `build/ai_results.json`.

## Docs
- `docs/VISION.md` — editor-first direction and UX goals
- `docs/TECHNICAL_OVERVIEW.md` — full system overview (runtime + tooling)
- `docs/API_REFERENCE.md` — public headers, CLI surface, file formats
- `docs/LLM_REFERENCE.md` — OpenAI templates, structured outputs, safe workflows
