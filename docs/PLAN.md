# Support decently generated 3D models with textures + skeletons/rigs + animations that can be scripted by AI.
## 0) What “Done” Looks Like (Definition of Done)
- A rigged GLB (`testmanny.glb`) can be imported via `rkgctl asset import` into **project content** under `projects/<project>/content/assets/testmanny/` with deterministic outputs.
- The editor can render a textured **static mesh** and a **skinned** `testmanny` mesh in Vulkan without debug fallbacks.
- `testmanny` animations (idle/walk/run) can be previewed in-editor (play/pause/scrub) and played at runtime.
- AI instructions can import assets, create prefabs, spawn entities, attach items to bones, and control animator clips **via allowlisted commands**.
- Text on surfaces is implemented through **materials/decals/textures**, not by UI overlays.

## 1) Current Repo Capabilities and Gaps (Reality Check)
**Capabilities (already present)**
- Vulkan renderer plugin, ImGui editor, ECS with Transform/Renderable/Velocity/RigidBody/Collider/CharacterController/Skeleton.
- physics_basic plugin and renderer_hooks debug line list + viewport camera hook.
- Runtime host plugin system and demo_game content YAML.

**Gaps (must be solved for the goal)**
- No GLB importer or asset pipeline for textured meshes/materials/skins/animations.
- No skinned mesh renderer (GPU skinning) or animator component.
- No editor UX for skeleton/animation preview or socket authoring.
- No AI instruction schema for asset/animation workflows.

**Assumptions**
- Canonical asset format is glTF 2.0 / GLB.
- Vulkan remains primary backend for this scope; RHI remains viable for future backends.
- `testmanny.glb` is provided locally by the developer (see Section 10).

## 1.1) Offline / No-Network Build Policy
- **No network at configure/build time**: Phases must succeed with no external downloads.
- **No required FetchContent**: New required dependencies must be vendored under `third_party/`.
- **Pinned versions + licenses**: Vendor exact versions with `LICENSE` files and a `VERSION.txt` (commit SHA or tag).
- **Optional fetches must be guarded**: If FetchContent is kept for optional features, it must be OFF by default and not required for Phase 1 outputs.
- **No fetched JSON library requirement**: Phase 1 importer outputs (`asset.json`, `materials.json`) must be written with a tiny in‑tree JSON writer (not a fetched dependency).

## 2) Terminology and Key Concepts (Mesh, Material, Skin, Rig, Joint, Bone, Clip, Socket)
- **Mesh**: vertex + index data used for rendering.
- **Material**: shader parameters + texture bindings (PBR inputs).
- **Skin**: joint bindings + inverse bind matrices.
- **Rig / Skeleton**: hierarchy of joints and local transforms.
- **Joint / Bone**: node in skeleton; bone often refers to transform between joints.
- **Clip**: animation data (T/R/S tracks over time) for a skeleton.
- **Socket**: named attachment transform anchored to a joint.

## 3) Canonical Asset Format and Coordinate Conventions (glTF 2.0 / GLB)
- **Primary format**: glTF 2.0 / `.glb`.
- **Coordinate conventions**:
  - Engine: **Y-up, Z-forward, X-right**.
  - Rotation units: **radians**.
  - Units: **1.0 = 1 meter**.
- **Import policy**:
  - **No axis conversion by default**.
  - Apply explicit **orientation correction** only if validation shows the asset is rotated/mirrored.
  - Validation must use `testmanny.glb` with an axis gizmo; log any correction applied.
- **Ecosystem note**:
  - Tooling around handedness and “forward” can be inconsistent.
  - Therefore we validate with test assets and apply explicit corrections if required; we do **not** guess.

## 4) Asset Pipeline Overview (Source -> Imported -> Cooked -> Runtime)
**Source**
- Developer provides `testmanny.glb` locally.
- Preferred locations:
  - Option A (preferred): `projects/<project>/content/source_assets/testmanny/testmanny.glb`
  - Option B: pass absolute/relative path to `rkgctl asset import --in <path>`

**Imported (project content, versionable)**
- `rkgctl asset import` outputs:
  - `projects/<project>/content/assets/<asset_name>/asset.json`
  - `mesh.bin`, `materials.json`, `textures/*`, `skeleton.json`, `skin.bin`, `clips/*.clip.json`

**Cooked (derived)**
- Cooked content is written to `build/content_cache/<project>/` (not checked in).

**Runtime**
- Runtime loads from cooked cache if present, falls back to project content.

**Optional upstream (Meshy pipeline)**
- Meshy (or other generators) outputs **GLB**; engine only consumes GLB.
- Flow: generate model → remesh → texture → download GLB → `rkgctl asset import` → AI instructions place/animate.
- No direct Meshy API integration is required for the engine to work.

## 5) Data Model Specifications
### 5.1 Asset Manifest Format (asset.json)
- Fields: `name`, `type`, `source_path`, `importer_version`, `deps[]`, `hash`, `timestamp`.

### 5.2 Mesh Asset Format (mesh.bin + metadata)
- Binary layout: positions, normals, UV0, tangents, joint indices, joint weights, indices.
- Metadata: bounds, vertex layout, index type.

### 5.3 Material Asset Format (materials.*)
- JSON: baseColorFactor, metallicFactor, roughnessFactor, texture slots (baseColor, normal, metallicRoughness, occlusion, emissive).
- References are **project‑relative paths**.

### 5.4 Texture Asset Format (textures/*, sRGB rules, mipmaps)
- PNG/JPG decoded via stb_image (Phase 1/2A).
- sRGB for baseColor/emissive; linear for normal/roughness/metallic/occlusion.
- Mipmaps generated on import or at runtime later.

### 5.5 Skeleton/Skin Asset Format (skeleton/skin data, inverse bind matrices)
- `skeleton.json`: joints, parent indices, bind pose transforms.
- `skin.bin`: inverse bind matrices (Mat4 array).

### 5.6 Animation Clip Asset Format (clips, sampling, compression later)
- JSON: clips with channels for joint index + T/R/S tracks (time + value arrays).
- Sampling: linear for T/S, slerp for R (quaternions).

### 5.7 ECS Components Required (SkinnedMeshRenderer, Animator, Socket/Attachment)
- `SkinnedMeshRenderer`: mesh asset, material asset, skin asset, skeleton ref.
- `Animator`: current clip, time, speed, looping.
- `Socket`: name + joint index + local offset.

## 6) Rendering Architecture Plan (Vulkan-first, RHI-aware)
### 6.1 Static Mesh Rendering (textured)
- Vulkan pipeline for textured meshes and descriptor sets.
- Keep RHI abstraction in mind for future backends.

### 6.2 Skinned Mesh Rendering (GPU skinning path)
- Vertex shader reads joint matrices from storage buffer.
- CPU skinning only as **temporary debug** fallback.

### 6.3 Animation Application (pose -> skin matrices)
- Sample clip → local joint transforms → world pose → skin matrices.

### 6.4 Debug Visualization vs True Surface Text/Decals (explicit separation)
- ImGui overlays are screen-space only.
- Text on surfaces must use textures/materials/decals.

### 6.5 Shader + SPIR‑V Workflow
- **Shader locations**: `plugins/renderer/vulkan/shaders/`.
- **Compilation**: `glslc` to SPIR‑V, either embedded or runtime‑loaded.
- **Descriptor layouts**: versioned and documented in `docs/TECHNICAL_OVERVIEW.md`.
- **Minimum shader set**:
  - Textured static mesh (vertex + fragment)
  - Skinned mesh (vertex + fragment)
  - Debug lines (existing)

## 7) Editor UX Plan
### 7.1 Editor Camera vs Player Camera (Play/Stop behavior)
- Keep Play/Stop transitions and viewport focus gating.

### 7.2 Asset Browser + Inspector Requirements
- GLB import panel, asset list, dependency view.

### 7.3 Skeleton Tree View + Bone Selection + Socket Authoring
- Hierarchy tree, bone selection, socket creation and offsets.

### 7.4 Animation Preview Controls (play/pause/scrub)
- Timeline slider, play/pause, loop toggle, clip selection.

### 7.5 Debug Overlays (world->screen labels, velocity arrows, bone lines)
- Keep existing overlays and add skeleton debug lines.

## 8) Tooling Plan (rkgctl)
### 8.1 `rkgctl asset import` (GLB -> engine assets)
- Command: `rkgctl asset import --in <glb> --out projects/<project>/content/assets/<asset_name>`.
- Loader: **cgltf** (vendored); textures via **stb_image** (vendored).

### 8.2 Determinism, Validation, and Error Reporting
- Deterministic output order; hash tracking.
- Validation errors with file/mesh/primitive context.

### 8.3 Optional: `rkgctl asset doctor` (sanity checks)
- Checks for missing textures, invalid joints, animation ranges.

## 9) AI Scriptability Plan (Allowlisted Instructions)
### 9.1 Instruction Schema (JSON) — Supported Commands
- `import_asset` (GLB → asset dir)
- `create_prefab` (mesh/material references)
- `spawn_entity` (level placement)
- `attach_socket` (entity → bone)
- `set_animator_state` (clip/playback)
- `assign_script` (Lua/JS/Python by reference)

### 9.2 Safety Model (allowlist, no arbitrary code)
- Only allowlisted commands are executed.
- Scripts are referenced by asset path; no arbitrary code execution.
- Apply instructions via diff‑based, reviewable updates.

### 9.3 Example “AI Builds a Character” Workflows
- Example JSON bundles in docs demonstrating import → prefab → spawn → animate → attach.

## 10) Test Asset Policy: testmanny.glb (MANDATORY)
- `testmanny.glb` is provided locally by the developer.
- Preferred path: `projects/<project>/content/source_assets/testmanny/testmanny.glb`.
- Alternative: CLI path passed to `rkgctl asset import --in <path>`.
- Large binaries should use Git LFS or remain local‑only.

## 11) Performance Targets (2K–4K readiness relevant to this scope)
- 60 FPS at 1080p on mid‑tier GPU with static + skinned mesh + UI.
- 30+ FPS at 2K/4K (future: LODs/streaming).

## 12) Testing Strategy (Unit + Smoke + Demo)
- Unit: glTF parsing, animation sampling, skin matrix generation.
- Smoke: import `testmanny.glb` and render static + skinned.
- Demo: editor preview + runtime playback.

## 13) Phased Implementation Plan (Executable)
### PHASE 0 — Repo Audit + Decisions Lock‑in (docs + tiny fixtures policy)
**Objective**: Confirm repo state, lock library choices, document policies.
**Dependencies**: None.
**Inputs/Outputs**:
- Input: repo audit.
- Output: updated docs and decision record.
**New files expected**: none.
**Modified files expected**:
- `docs/PLAN.md`, `docs/ROADMAP.md`, `docs/TECHNICAL_OVERVIEW.md`.
**Implementation steps**:
1) Audit current content pipeline and renderer paths.
2) Confirm library choices (cgltf, stb_image) and record licenses.
3) Document coordinate policy and validation steps.
4) Document **Offline / No‑Network Build Policy** and ensure Phase 1 obeys it.
**Data model changes**: None.
**Editor UX changes**: None.
**CLI/tooling changes**: None.
**Tests to add/update**: None.
**Acceptance criteria**:
- Docs updated and consistent with this plan.
- Offline / no‑network policy is explicitly documented.
**Verification commands**:
- `rg -n "cgltf|stb_image|testmanny" docs/PLAN.md`.
**testmanny validation steps**:
- Policy documented.
**Stop point**:
- Stop after Phase 0 and report doc diff summary.
**Next Phase Preview**:
- Phase 1: minimal GLB import pipeline.
**Risks/edge cases**:
- None.

### PHASE 1 — Minimal GLB Import (static mesh + materials/textures extraction)
**Objective**: Import `testmanny.glb` into engine asset formats.
**Dependencies**: Phase 0 decisions locked.
**Inputs/Outputs**:
- Input: `testmanny.glb` path.
- Output: deterministic assets under `projects/<project>/content/assets/testmanny/`.
**New files expected**:
- `core/include/rkg/asset_types.h`
- `core/include/rkg/json_write.h`
- `core/src/asset_import.cpp`
- `core/src/json_write.cpp`
- `third_party/cgltf/cgltf.h` (vendored)
- `third_party/cgltf/LICENSE`
- `third_party/cgltf/VERSION.txt` (pin: cgltf **v1.13** tag)
- `third_party/stb/stb_image.h` (vendored)
- `third_party/stb/LICENSE`
- `third_party/stb/VERSION.txt` (pin: stb commit **`a2f0d3e`**)
**Modified files expected**:
- `apps/rkgctl/src/main.cpp`
- root `CMakeLists.txt` to include third_party (no FetchContent)
**Phase 1 Implementation Rule**:
- The importer **must use cgltf** for parsing (no handwritten GLB/JSON parser).
- If a split (Phase 1A/1B) is ever needed, update **PLAN.md + ROADMAP.md first** before writing code.
**Implementation steps**:
1) Vendor **cgltf v1.13** and **stb_image** into `third_party/` with LICENSE + VERSION.txt.
2) Implement `json_write.cpp` (tiny JSON writer) for `asset.json` and `materials.json`.
3) Implement `asset_import.cpp` for static mesh + textures + materials using **cgltf** (required).
4) Add `rkgctl asset import` command wiring.
5) Output deterministic `asset.json`, `mesh.bin`, `materials.json`, `textures/*`.
6) Ensure no FetchContent is required for Phase 1.
**Data model changes**:
- Add asset metadata structs in `asset_types.h`.
**Editor UX changes**:
- None.
**CLI/tooling changes**:
- `rkgctl asset import`.
**Tests to add/update**:
- `tests/src/test_smoke.cpp`: verify deterministic output sizes.
**Acceptance criteria**:
- Import succeeds and output is deterministic (same hash on repeated runs).
- `cmake --preset linux-debug` config succeeds **with no network access**.
- No FetchContent downloads are required for Phase 1.
**Verification commands**:
- `./build/bin/rkgctl asset import --in <path>/testmanny.glb --out projects/demo_game/content/assets/testmanny`
**testmanny validation steps**:
- Output files present and non‑empty.
**Stop point**:
- Stop after CLI output verified.
**Next Phase Preview**:
- Phase 2A: runtime asset registry and CPU loading.
**Risks/edge cases**:
- Missing textures or unsupported GLB features.
- Incorrect or missing pinned versions in `third_party/`.

### PHASE 2A — Runtime Asset Registry + CPU‑side Loading/Validation Logs
**Objective**: Load imported assets into runtime cache (no rendering yet).
**Dependencies**: Phase 1 import output.
**Inputs/Outputs**:
- Input: asset directory under project content.
- Output: runtime logs confirming load and validation.
**New files expected**:
- `runtime/include/rkg/asset_cache.h`
- `runtime/src/asset_cache.cpp`
**Modified files expected**:
- `runtime/src/runtime_host.cpp`
**Implementation steps**:
1) Add asset cache data structures.
2) Implement file loading + validation logs.
3) Integrate asset cache into runtime startup.
**Data model changes**:
- Asset cache structs for mesh/material/texture handles.
**Editor UX changes**:
- None.
**CLI/tooling changes**:
- None.
**Tests to add/update**:
- `tests/src/test_smoke.cpp`: load cache and verify counts.
**Acceptance criteria**:
- Runtime log confirms asset load with valid counts.
**Verification commands**:
- `cmake --build --preset linux-debug`
- `./build/bin/rkg_demo_game`
**testmanny validation steps**:
- Log line: “asset_cache loaded testmanny”.
**Stop point**:
- Stop after logs verified.
**Next Phase Preview**:
- Phase 2B: Vulkan textured rendering.
**Risks/edge cases**:
- Asset path mismatch.

### PHASE 2B — Vulkan Pipeline + Descriptors + Textured Rendering
**Objective**: Render static textured meshes.
**Dependencies**: Phase 2A asset cache.
**Inputs/Outputs**:
- Input: loaded mesh/material/texture assets.
- Output: visible textured mesh in viewport.
**New files expected**:
- `plugins/renderer/vulkan/shaders/mesh_textured.vert`
- `plugins/renderer/vulkan/shaders/mesh_textured.frag`
**Modified files expected**:
- `plugins/renderer/vulkan/src/vulkan_renderer.cpp`
- `plugins/renderer/vulkan/include/*` (pipeline structs)
**Implementation steps**:
1) Add shader compilation/loading.
2) Create descriptor layouts for textures and material constants.
3) Upload textures and bind descriptors.
4) Render static mesh with textured pipeline.
**Data model changes**:
- None beyond asset cache usage.
**Editor UX changes**:
- None.
**CLI/tooling changes**:
- None.
**Tests to add/update**:
- `tests/src/test_smoke.cpp`: verify pipeline created.
**Acceptance criteria**:
- Textured testmanny visible in editor (no debug-only draw).
**Verification commands**:
- `./build/bin/rkg_editor --project projects/demo_game`
**testmanny validation steps**:
- Screenshot shows textured model.
**Stop point**:
- Stop after screenshot captured.
**Next Phase Preview**:
- Phase 3: skeleton import + debug draw.
**Risks/edge cases**:
- Descriptor layout mismatches.

### PHASE 3 — Skin/Skeleton Import + Debug Skeleton Draw (no deformation yet)
**Objective**: Load skeleton + skin data, draw bone lines.
**Dependencies**: Phase 1 import outputs.
**Inputs/Outputs**:
- Input: `skeleton.json`, `skin.bin`.
- Output: debug bone lines in editor.
**New files expected**:
- `core/include/rkg/skeleton_asset.h`
- `core/src/skeleton_asset.cpp`
**Modified files expected**:
- `apps/rkg_editor/src/main.cpp`
- `runtime/src/runtime_host.cpp`
**Implementation steps**:
1) Load skeleton + skin assets into cache.
2) Compute world pose for debug draw.
3) Draw bone lines via debug line list.
**Data model changes**:
- Skeleton asset structs.
**Editor UX changes**:
- Toggle for skeleton debug view.
**CLI/tooling changes**:
- None.
**Tests to add/update**:
- `tests/src/test_smoke.cpp`: skeleton load + pose compute.
**Acceptance criteria**:
- Bone hierarchy visible in editor.
**Verification commands**:
- `./build/bin/rkg_editor --project projects/demo_game`
**testmanny validation steps**:
- Bone lines match expected hierarchy.
**Stop point**:
- Stop after visual verification.
**Next Phase Preview**:
- Phase 4: GPU skinning.
**Risks/edge cases**:
- Incorrect joint hierarchy order.

### PHASE 4 — GPU Skinning (deform testmanny)
**Objective**: GPU‑deform mesh using skin matrices.
**Dependencies**: Phase 3 skeleton import.
**Inputs/Outputs**:
- Input: skin matrices + mesh weights.
- Output: deformed skinned mesh.
**New files expected**:
- `plugins/renderer/vulkan/shaders/mesh_skinned.vert`
**Modified files expected**:
- `plugins/renderer/vulkan/src/vulkan_renderer.cpp`
**Implementation steps**:
1) Add GPU skinning buffers.
2) Bind joint matrices to shader.
3) Render skinned mesh pipeline.
**Data model changes**:
- None (uses existing skin asset data).
**Editor UX changes**:
- None.
**CLI/tooling changes**:
- None.
**Tests to add/update**:
- `tests/src/test_smoke.cpp`: validate joint matrix count.
**Acceptance criteria**:
- Skinned mesh deforms (not static).
**Verification commands**:
- `./build/bin/rkg_editor --project projects/demo_game`
**testmanny validation steps**:
- Visual deformation in editor.
**Stop point**:
- Stop after deformation verified.
**Next Phase Preview**:
- Phase 5: animation import + playback.
**Risks/edge cases**:
- Weight normalization issues.

### PHASE 5 — Animation Import + Playback + Animator Component
**Objective**: Play animation clips.
**Dependencies**: Phase 4 GPU skinning.
**Inputs/Outputs**:
- Input: `clips/*.clip.json`.
- Output: animated playback in editor/runtime.
**New files expected**:
- `core/include/rkg/animation_asset.h`
- `core/src/animation_asset.cpp`
**Modified files expected**:
- `apps/rkg_editor/src/main.cpp`
- `runtime/src/runtime_host.cpp`
**Implementation steps**:
1) Parse clip data from import output.
2) Add Animator component + update loop.
3) Editor playback controls.
**Data model changes**:
- Animator component definitions.
**Editor UX changes**:
- Timeline + clip selection.
**CLI/tooling changes**:
- None.
**Tests to add/update**:
- `tests/src/test_smoke.cpp`: clip sampling values.
**Acceptance criteria**:
- Idle/walk/run play correctly.
**Verification commands**:
- `./build/bin/rkg_editor --project projects/demo_game`
**testmanny validation steps**:
- Animated playback visible.
**Stop point**:
- Stop after preview verified.
**Next Phase Preview**:
- Phase 6: sockets/attachments.
**Risks/edge cases**:
- Incorrect clip time sampling.

### PHASE 6 — Sockets/Attachments (“armor”) bound to bones
**Objective**: Attach items to bones.
**Dependencies**: Phase 5 animation playback.
**Inputs/Outputs**:
- Input: socket definitions.
- Output: attachment follows bone in motion.
**New files expected**:
- `core/include/rkg/socket_asset.h`
- `core/src/socket_asset.cpp`
**Modified files expected**:
- `apps/rkg_editor/src/main.cpp`
**Implementation steps**:
1) Define socket data model.
2) Add socket editor UI.
3) Apply socket transforms at runtime.
**Data model changes**:
- Socket component/asset.
**Editor UX changes**:
- Socket authoring panel.
**CLI/tooling changes**:
- None.
**Tests to add/update**:
- `tests/src/test_smoke.cpp`: socket transform correctness.
**Acceptance criteria**:
- Attachment follows hand bone.
**Verification commands**:
- `./build/bin/rkg_editor --project projects/demo_game`
**testmanny validation steps**:
- Attachment sticks to hand during animation.
**Stop point**:
- Stop after attachment verified.
**Next Phase Preview**:
- Phase 7: AI instruction expansion.
**Risks/edge cases**:
- Offset orientation mistakes.

### PHASE 7 — AI Instruction Expansion + rkgctl integration
**Objective**: AI can import/spawn/animate/attach assets.
**Dependencies**: Phase 6 sockets.
**Inputs/Outputs**:
- Input: allowlisted instruction JSON.
- Output: scene updated and logged.
**New files expected**:
- `docs/examples/ai_import_spawn.json`
- `docs/examples/ai_attach_socket.json`
**Modified files expected**:
- `core/src/instructions.cpp`
- `core/include/rkg/instruction_schema.h`
**Implementation steps**:
1) Define instruction schema and validation.
2) Implement import/spawn/attach/animate commands.
3) Add example bundles.
**Data model changes**:
- Instruction schema structs.
**Editor UX changes**:
- Optional status panel for instruction apply results.
**CLI/tooling changes**:
- `rkgctl instruction apply --in <json>` (optional).
**Tests to add/update**:
- `tests/src/test_smoke.cpp`: apply instruction bundle.
**Acceptance criteria**:
- AI JSON spawns animated testmanny with attached item.
**Verification commands**:
- `./build/bin/rkgctl instruction apply --in docs/examples/ai_import_spawn.json`
**testmanny validation steps**:
- Scene shows animated `testmanny` after applying instructions.
**Stop point**:
- Stop after AI workflow verified.
**Next Phase Preview**:
- Phase 8: editor polish.
**Risks/edge cases**:
- Unsafe instruction inputs; enforce schema.

### PHASE 8 — Editor Polish (inspector, preview timeline, authoring UX)
**Objective**: Usable authoring workflow for rigs/animation.
**Dependencies**: Phases 1–7.
**Inputs/Outputs**:
- Input: imported assets + animation.
- Output: polished editor UX.
**New files expected**: none.
**Modified files expected**:
- `apps/rkg_editor/src/main.cpp`
- `docs/TECHNICAL_OVERVIEW.md`
**Implementation steps**:
1) Refine asset browser and inspector.
2) Improve animation preview timeline.
3) Improve socket authoring UX.
**Data model changes**:
- None.
**Editor UX changes**:
- Enhanced panels and controls.
**CLI/tooling changes**:
- None.
**Tests to add/update**:
- Manual UX checklist in docs.
**Acceptance criteria**:
- Authoring flow feels complete; no broken UI paths.
**Verification commands**:
- `./build/bin/rkg_editor --project projects/demo_game`
**testmanny validation steps**:
- Preview + authoring works for testmanny.
**Stop point**:
- Stop after UX review with screenshots.
**Next Phase Preview**:
- Post‑plan enhancements (LODs, KTX2, retargeting).
**Risks/edge cases**:
- UI complexity.

## 14) Risks, Edge Cases, and Mitigations
- GLB size/texture count → memory spikes; use streaming later.
- Coordinate mismatches → validate with axis gizmo and log corrections.
- GPU skinning bugs → add debug overlays and CPU fallback (temporary).
- Licensing issues → restrict assets to known‑good sources.

## 15) Agent Execution Protocol (How Codex will run these phases across chats)
- Start each phase by re‑auditing repo state and updating **Plan Status** below.
- Never advance to the next phase until current acceptance criteria is met and reported:
  - Build output summary
  - Test output summary
  - Demo run behavior summary
- After each phase, update **Plan Status** and `docs/ROADMAP.md` progress markers.

**Plan Status (to be updated after each phase)**
- Phase 0: ☑
- Phase 1: ☐
- Phase 2A: ☐
- Phase 2B: ☐
- Phase 3: ☐
- Phase 4: ☐
- Phase 5: ☐
- Phase 6: ☐
- Phase 7: ☐
- Phase 8: ☐
