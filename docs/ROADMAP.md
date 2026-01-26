# Roadmap — Rigged 3D Assets + AI Scripting

## Milestone Summary (6–12 milestones)
This roadmap is the story from **AI‑first game creation** to fully rigged, textured, animated characters. Each milestone maps directly to PLAN phases and clarifies what becomes possible for users/AI and what is *not* included yet.
**Build Policy**: All milestones must remain **offline‑buildable** (no required FetchContent/network during configure/build). Required third‑party libs are vendored under `third_party/` with pinned versions and licenses.

## Progress
- Phase 0 complete (docs + policy lock‑in).
- M1 complete (Phase 1 done, offline import).
- M2 complete (Phase 2A done, runtime CPU‑side load + logs).
- M3 complete (Phase 2B done, textured static mesh demo validated).

## Milestones (each with outcome, acceptance criteria, demo, phases)
### M1 — Import Foundation (Decisions + GLB Import)
- **Outcome**: Deterministic GLB import into project assets.
- **Phases**: Phase 0, Phase 1
- **What becomes possible**: AI/user can import `testmanny.glb` into `projects/<project>/content/assets/`.
- **Not included yet**: Runtime loading or rendering.
- **Acceptance criteria**: `rkgctl asset import` produces deterministic outputs **and** `cmake --preset linux-debug` succeeds with no network access (no required FetchContent).
- **Demo**: CLI import of `testmanny.glb`.

### M2 — Runtime Asset Registry + CPU Validation
- **Outcome**: Runtime loads imported assets and logs validation.
- **Phases**: Phase 2A
- **What becomes possible**: Editor/runtime can list assets from content.
- **Not included yet**: Textured rendering pipeline.
- **Acceptance criteria**: Runtime logs show `testmanny` asset load.
- **Demo**: Run `rkg_demo_game`; log confirms asset cache load.

### M3 — Textured Static Mesh Rendering
- **Outcome**: Vulkan pipeline renders textured static meshes.
- **Phases**: Phase 2B
- **What becomes possible**: Textured static mesh appears in the editor via the demo path.
- **Not included yet**: Skeleton import or skinning.
- **Acceptance criteria**: Textured mesh visible in viewport and log line confirms textured demo path.
- **Demo**: Editor shows `textured_fixture` (2x2 colored PNG); screenshot waived by author after visual confirmation (2026‑01‑25).

### M4 — Skeleton Import + Authoring + Debug Bones
- **Outcome**: Skeletons can be **imported or authored in-editor**, persisted to content, and visualized.
- **Phases**: Phase 3
- **What becomes possible**: Bone hierarchy visible; joints inspectable; users can create bones/root and save a skeleton asset.
- **Not included yet**: Mesh deformation or animation playback.
- **Acceptance criteria**:
  - Bone lines drawn for a rigged asset **or** an authored skeleton.
  - Editor can create bones, set parent/root, edit local pose, and **persist** the skeleton to content.
- **Demo**: Editor shows skeleton debug view and authored skeleton saved/loaded.

### M5 — GPU Skinning
- **Outcome**: Skinned mesh deformation in Vulkan.
- **Phases**: Phase 4
- **What becomes possible**: `testmanny` deforms with skin weights.
- **Not included yet**: Animation clip playback.
- **Acceptance criteria**: Visible deformation (not static).
- **Demo**: Editor shows deformed mesh under test pose.

### M6 — Animation Import + Playback
- **Outcome**: Animation clips load and play in editor/runtime.
- **Phases**: Phase 5
- **What becomes possible**: Idle/walk/run preview and runtime playback.
- **Not included yet**: Sockets/attachments.
- **Acceptance criteria**: Clips scrub and play correctly.
- **Demo**: Editor timeline plays `testmanny` clips.

### M7 — Sockets + Attachments
- **Outcome**: Items can be attached to bones and follow animation.
- **Phases**: Phase 6
- **What becomes possible**: AI/user can attach props to bones.
- **Not included yet**: AI instruction flow for full asset pipeline.
- **Acceptance criteria**: Attachment follows hand bone during animation.
- **Demo**: Editor shows attached prop moving with `testmanny`.

### M8 — AI Instruction Workflow (Import → Spawn → Animate)
- **Outcome**: AI can orchestrate asset import, spawning, and animation.
- **Phases**: Phase 7
- **What becomes possible**: End‑to‑end AI‑driven character creation in a scene.
- **Not included yet**: Full editor polish + timeline UX.
- **Acceptance criteria**: Instruction bundle spawns animated `testmanny`.
- **Demo**: Apply JSON instructions; verify scene.

### M9 — Editor Polish & Authoring UX
- **Outcome**: Usable authoring workflow for rigs/animation.
- **Phases**: Phase 8
- **What becomes possible**: Smooth editor workflow for assets, rigs, sockets, animations.
- **Not included yet**: Advanced PBR, LODs, streaming, retargeting.
- **Acceptance criteria**: Editor UX checklist passes.
- **Demo**: Recorded walkthrough of asset → animation → attach flow.

## Backlog / Future Enhancements (LODs, streaming, KTX2, advanced PBR, post, retargeting)
- Advanced PBR materials, post‑processing, and HDR pipeline.
- KTX2/BasisU texture compression and streaming.
- LODs and runtime asset streaming.
- Animation retargeting and blend trees.
- D3D12 backend parity with Vulkan.
- Multi‑agent AI orchestration and project‑scale automation.
