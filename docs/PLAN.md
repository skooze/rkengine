# RKG ALPHA 0.4 Plan (Stability + Fast Dev Loop + Pack v0)

This plan is the required implementation order for Alpha 0.4. “Done” means each checklist item below passes its acceptance criteria.

## A) D3D12 resize handling + robustness
**Scope**
- Detect SDL window resize events and propagate to renderer.
- Add renderer hook: `OnWindowResized(width, height)`.
- D3D12: wait for GPU, release RTVs, resize swapchain, rebuild RTVs, update viewport/scissor.
- Handle width/height == 0 (skip rendering until restored).
- Log resize begin/end + failure reasons.

**Acceptance**
- Windows resize does not crash, swapchain rebuilds, logs show success/failure.
- Linux unaffected.

## B) Dev loop “no F5” in dev_mode
**Scope**
- `rkgctl content watch` writes a richer `cook_status.json`.
- `rkg_demo_game` (dev_mode=true) auto-reloads when cook_status changes and cook succeeded.
- Non-dev_mode keeps manual reload only.

**Acceptance**
- Edit prefab/level → watch auto-cooks → game auto-reloads without F5 (dev_mode only).
- Changes are logged with success/failure details.

## C) Pack format v0 (deterministic)
**Scope**
- `rkgctl content cook` emits `content.pack` + `content.index.json`.
- Pack entries are sorted by canonical asset path for deterministic output.
- Runtime loads pack when present; fallback to cooked YAML mirror if not.
- Provide `rkgctl content dump-pack` for inspection.

**Acceptance**
- Pack files are deterministic for identical content.
- Runtime uses pack when present and logs actionable errors.

## D) Renderer selection robustness
**Scope**
- If requested renderer isn’t available, choose a valid fallback (Vulkan → D3D12 → Null).
- Use renderer caps + availability checks; do not rewrite configs.
- Log fallback decision.

**Acceptance**
- Unsupported renderer requests do not crash; a supported renderer is chosen with clear logs.

## Stretch: ImGui DX12 backend
Optional if A–D are complete and stable.

---

## Alpha 0.4 Demo Definition
- Windows: resizing the D3D12 demo window does not crash, and swapchain rebuilds.
- Linux: Vulkan path remains stable.
- Dev loop: `rkgctl content watch` + auto reload with cook_status.
- Pack v0 produced and loaded by runtime when present.

---

# Alpha 0.6 – OpenAI Provider Integration
Checklist for dev-time orchestration via OpenAI (Responses API).

## A) OpenAI provider integrated
- `rkgctl agent openai plan` generates a validated plan JSON.
- Missing API key and HTTP errors yield clear messages and nonzero exit.

## B) Structured outputs enforced
- Requests include `text.format` with `json_schema` and `strict=true`.
- Returned plan JSON is validated before write/apply.

## C) Templates + schema
- `docs/schemas/rkg_plan.schema.json` is the canonical schema.
- Templates exist under `docs/agent_templates/openai/`.
- Example request/response files exist in `docs/examples/`.

## D) End-to-end apply flow
- `rkgctl agent openai apply` generates, stages, and applies patches.
- Artifacts saved under `build/ai_runs/<run_id>/`.

## E) Documentation
- `docs/AI_ORCHESTRATION.md` and `docs/DEVELOPMENT.md` updated with OpenAI usage and security note.
