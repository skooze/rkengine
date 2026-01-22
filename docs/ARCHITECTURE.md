# RKG Architecture

RKG is an AI-orchestrated game development engine with **full manual control**. The core idea is **agent plans → task graph → patch set → human approval → updated content → running demo**. The engine is intentionally small and stable, while all major systems live as plugins.

## Core Narrative
1) An agent (OpenAI via `rkgctl` today; Bedrock scaffolding later) produces a **Dev Task Graph**.
2) The compiled CLI (`rkgctl`) validates the plan and converts tasks into **allowlisted actions**.
3) Actions are staged as **patches** (diffs + base/proposed files) and require approval to apply.
4) The demo app loads updated content and can apply **Runtime Instructions**.

## Core (Small + Stable)
Core provides:
- Logging (`rkg::log`)
- ECS (entities + `Transform`)
- Plugin host (static today, dynamic-ready)
- Config loading (YAML/JSON)
- Event bus (lightweight)
- Replay hooks (record/replay for deterministic testing)

## Platform Layer
- SDL3 window + input on Linux.
- Windows stubs exist; no proprietary SDK headers.
- Console/mobile stubs exist with TODOs only.

## Plugins (Everything Else)
Plugins provide:
- **Renderer**: RHI + backends (null, Vulkan on Linux, D3D12 on Windows).
- **Scripting**: Lua, JS, Python (optional/stubbed).
- **Data**: YAML/JSON + SQLite.
- **AI**: orchestration interfaces + offline stub agent.
- **Cloud**: stubs for storage, telemetry, remote config, Bedrock.

## Renderer / RHI
RHI is minimal: `Device`, `Swapchain`, `CommandList`, `Buffer`, `Texture`, `Shader`.
Vulkan is the Linux target and renders visible geometry in the demo. D3D12 is implemented on Windows behind
`RKG_ENABLE_D3D12` and the Windows SDK; other platforms build without it.

## Scripting
`IScriptRuntime` supports:
- init(bindings)
- load_script(path/string)
- tick(dt)
- call(fn, args)
- bind_native(name, fn_ptr)

Lua is wired to basic logging and entity movement when available. JS and Python are optional and stubbed if deps are missing.

## Data / Content
Content lives under `content/prefabs` and `content/levels`. YAML is preferred, JSON is accepted. SQLite stores task/audit history locally.

## Record/Replay
RKG can record input/action streams to a replay file and re-run deterministically using a fixed timestep. This is used to detect regressions after AI-driven changes.

## AI Orchestration
Two schemas exist:
- **Dev Task Graph**: drives file/content changes.
- **Runtime Instructions**: safe updates to the running world.

See `docs/AI_ORCHESTRATION.md` and `docs/AI_INSTRUCTIONS.md`.
