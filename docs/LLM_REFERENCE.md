# LLM Reference (Dev-Time Orchestration)

This document describes how RKG integrates LLMs for **dev-time planning** only.
No runtime NPC chat or in-game inference is implemented in this repo.

## OpenAI Provider (Responses API)
RKG uses OpenAI’s Responses API via `rkgctl` (compiled, optional libcurl).

### Environment Variables
- `OPENAI_API_KEY` (required)
- `OPENAI_MODEL` (optional, default: `gpt-5.2-codex`)
- `OPENAI_BASE_URL` (optional, default: `https://api.openai.com`)

### project.yaml config
```
project:
  ai:
    provider: openai
    model: gpt-5.2-codex
    base_url: https://api.openai.com
    templates_dir: docs/agent_templates/openai
    timeout_seconds: 60
```

CLI flags always override project settings:
- `--provider`, `--model`, `--base-url`, `--templates-dir`, `--timeout-seconds`

## Prompt Templates
Templates live in `docs/agent_templates/openai/`:
- `planner_system.md`
- `planner_user.md`
- `responses_request_template.json`

Templates are combined with:
- **Goal** (`--goal`)
- **Context pack JSON** (from `rkgctl context dump`)
- **Constraints summary**

## Structured Outputs Contract
OpenAI requests use:
```
text.format = { "type": "json_schema", "strict": true, "schema": <rkg_plan.schema.json> }
```
Returned plans are validated against `docs/schemas/rkg_plan.schema.json`.

If the plan does not match schema, `rkgctl` prints keypath errors and aborts.
Use `--no-strict-enums` to relax enum validation for `task.type`/`task.status` only.

## Response Extraction Shapes Supported
`rkgctl` extracts JSON from common Responses API shapes, including:
- `output_text`
- `output[].content[]`
- `choices[].message.content`
- `tool_calls[].arguments`

If extraction fails:
- raw response is saved to `build/ai_runs/<run_id>/openai_response.json`
- `rkgctl` exits with a clear diagnostic

## Offline Provider (Air-Gapped / Manual)
Use offline plans when OpenAI is unavailable:
```
./build/bin/rkgctl agent offline plan --in <plan.json> --project <path>
./build/bin/rkgctl agent offline apply --in <plan.json> --project <path> --yes
```

Offline is also used automatically if OpenAI is disabled and `--in` is provided.

## Context Drift Safety
Each run captures a context fingerprint in `context_pack.json`:
- repo root
- git HEAD (if available)
- tree hashes
- capped file lists (project/core/plugins)

If drift is detected at apply-time:
- `rkgctl` warns loudly
- requires interactive approval or `--allow-context-drift`
- drift details are recorded in `build/ai_results.json`

Use:
```
./build/bin/rkgctl context diff --baseline <context_pack.json> --project <path> --include-core --include-plugins
```

## Recommended Agent Operating Procedure
1) Generate a **small, focused plan**
2) Review diffs in `build/ai_staging/<run_id>/patches/`
3) Apply with approval (`--yes` only when confident)
4) Roll back if needed (`rkgctl rollback --last`)
5) Keep plans scoped to allowlisted actions only

## Troubleshooting OpenAI Errors
- **401/403**: bad key or permissions → verify `OPENAI_API_KEY`
- **429**: rate limit/quota → wait or adjust usage
- **400**: invalid request → check model name and schema

## Security Note (Important)
Do **not** ship API keys in game binaries. LLM usage in RKG is **dev-time only**.
For runtime AI, use a server-side service and proxy requests securely.

## Future Runtime AI (High-Level Only)
Planned direction (not implemented):
- server-side orchestration layer
- short-lived signed tokens
- telemetry and audit trails
