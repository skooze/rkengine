# AI Orchestration

RKG centers on **Dev Task Graphs** produced by agents. The compiled CLI (`rkgctl`) validates these plans and produces **patch-first** change sets that are reviewed and applied by humans.

## Dev Task Graph Schema (JSON)
Each plan contains a `tasks` array. Each task has:
- `id`: unique string
- `title`: short title
- `description`: detailed intent
- `type`: one of `content_change`, `asset_build`, `build`, `test`, `run_demo`, `doc_update`
- `priority`: integer
- `deps`: array of task IDs
- `status`: `planned|in_progress|done|blocked`
- `inputs`: object
- `outputs`: object
- `artifacts`: array of paths
- `approvals_required`: boolean
- `retry_policy`: string
- `actions`: array of allowlisted actions

The canonical JSON schema lives at `docs/schemas/rkg_plan.schema.json`.
See `docs/examples/plan_spawn_cube.json`.

## Allowlisted Actions (Dev)
Only these actions are executed by default:
- `write_text_file(path, contents, mode)`
- `update_yaml(path, keypath, value)`
- `create_prefab(name, components, output_path)`
- `add_level_entity(level_path, prefab_ref, transform, name)`
- `record_audit(event)`

`run_process` is not allowed by default. If introduced later it must be explicitly enabled and restricted to known commands.

## Safety Gates
`rkgctl` defaults to:
- `--dry-run` ON (no file writes)
- `--require-approval` ON (interactive confirmation)

Use `rkgctl apply --plan ... --yes` to auto-approve. Use `--force` only if you accept conflicts being overridden.

## Patch-First Workflow
When applying a plan, `rkgctl` **stages patches** and prints unified diffs before any write:
```
build/ai_staging/<run_id>/
  patches/*.diff
  base_files/...
  proposed_files/...
  summary.json
```

The CLI **refuses to overwrite** if the base file hash has changed, and reports a conflict with the file path.

## Commands (Core)
- `rkgctl plan validate --plan <file>`
- `rkgctl plan summarize --plan <file>`
- `rkgctl plan schema --out <file>`
- `rkgctl apply --plan <file> [--yes] [--dry-run] [--force]`
- `rkgctl status`
- `rkgctl rollback --last`
- `rkgctl context dump --project <path> --out <file>`
- `rkgctl context diff --baseline <context_pack.json> --project <path> [--include-core] [--include-plugins]`
- `rkgctl agent status [--project <path>] [--runs <n>] [--json]`

## Audit Logging
Every action is audited to:
- `build/ai_audit.log` (JSON lines)
- `build/ai_tasks.sqlite` (if SQLite available)
- `build/ai_results.json` (summary)

Each audit record includes:
- timestamp
- action type
- params hash
- result
- files touched

`build/ai_results.json` includes `run_id`, `applied`, `conflicts`, `dry_run`, and `context_drift` (baseline/current fingerprints + change counts).
Schemas:
- `docs/schemas/rkg_run_info.schema.json`
- `docs/schemas/rkg_run_results.schema.json`

## OpenAI (Dev-Time Provider)
RKG uses OpenAI’s Responses API for dev-time plan generation in `rkgctl`.

Environment variables:
- `OPENAI_API_KEY` (required)
- `OPENAI_BASE_URL` (optional, default `https://api.openai.com`)
- `OPENAI_MODEL` (optional, default `gpt-5.2-codex`)

Commands:
- `rkgctl agent plan --goal "<text>" --project <path>` (uses project.ai provider by default)
- `rkgctl agent plan --provider openai|offline --goal "<text>" --project <path>`
- `rkgctl agent openai plan --project <path> --goal "<text>" [--model <name>] [--out <plan.json>]`
- `rkgctl agent openai apply --project <path> --goal "<text>" [--model <name>] [--yes] [--dry-run]`

Structured outputs:
- Requests include `text.format` with `json_schema` and `strict=true`.
- Returned plans are validated before writing or applying.
Templates live at `docs/agent_templates/openai/`.
Use `--print-raw` to print the extracted plan JSON to stdout for debugging.
Use `--no-strict-enums` to relax enum validation for `task.type`/`task.status` if needed.

Artifacts for each run:
```
build/ai_runs/<run_id>/
  context_pack.json
  openai_request.json
  openai_response.json
  plan.json
  run_info.json
  staged_patches/
  results.json
```

Security note:
- This is **dev-time only**. Do not ship API keys in games or binaries.

Troubleshooting:
- 401/403: invalid API key or permissions
- 429: rate limit or insufficient quota
- 400: invalid request (check model name and schema)

## Project AI Config (project.yaml)
Example:
```
project:
  name: demo_game
  ai:
    provider: openai
    model: gpt-5.2-codex
    base_url: https://api.openai.com
    templates_dir: docs/agent_templates/openai
    timeout_seconds: 60
```

## Offline Provider
Use offline plans for air‑gapped or manual workflows:
- `rkgctl agent offline plan --in <plan.json> --project <path>`
- `rkgctl agent offline apply --in <plan.json> --project <path> [--yes]`

Offline mode is used automatically when OpenAI is unavailable **and** `--in` is provided.

## Context Drift Safety
Every run captures a context fingerprint in `context_pack.json` (tree hash + git head).
When applying, if the current repo fingerprint differs:
- rkgctl prints a loud warning
- requires `--allow-context-drift` or interactive confirmation
Drift status and change counts are recorded in `build/ai_results.json` and `build/ai_runs/<run_id>/results.json`.

Context packs also include capped file lists for:
- project files (`files`)
- core (`core_files`)
- plugins (`plugins_files`)
Use `rkgctl context dump --cap <n>` to control list size and reduce diff noise for large repos.

## Offline / Manual Plans
Manual plan files are still supported:
- author/edit JSON directly
- validate and apply with `rkgctl plan validate` and `rkgctl apply`

## Future Bedrock Wiring (Scaffold)
Planned integration points remain in `plugins/cloud` and `plugins/ai`.

No credentials or proprietary SDKs are included in this repo.
