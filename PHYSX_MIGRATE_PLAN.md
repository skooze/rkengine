# PhysX Migration Plan (RKG Engine)

Date: 2026-02-07  
Owner: RKG Engine Team  
Scope: Replace `physics_basic` with PhysX via a clean physics abstraction layer, while preserving
LLM-driven IK and action control. This plan includes explicit removal of legacy code to prevent
overlap and confusion.

---

## Summary

We will integrate PhysX as the primary physics backend, but **keep IK / locomotion / action control
as an engine-owned layer**. PhysX will provide rigid bodies, shapes, collisions, contacts, and scene
queries. The LLM will issue **high-level Action Specs**, and our IK system will translate those into
per-frame bone targets. PhysX is responsible for collisions, impulses, and contact events, not for
animation logic.

This plan is structured to:
1) Create a stable **physics abstraction** first.
2) Integrate PhysX behind that abstraction.
3) Migrate existing systems.
4) Remove old code that overlaps.

---

## Current State (for reference)

Existing files and systems:
- `plugins/physics/basic/src/physics_basic.cpp`  
  Basic character-controller-like capsule sweeps vs `Plane` and `AABB`, no rigid-body solver.
- `core/include/rkg/ecs.h`  
  `RigidBody`, `Collider` components exist but are not fully integrated beyond basic physics.
- `core/src/locomotion.cpp`  
  Uses `registry.colliders()` for simple ground probing and self-collision world constraints.
- `runtime/src/runtime_host.cpp`  
  Registers `physics_basic` plugin and fallback legacy movement behavior.

Key gaps vs PhysX:
- No robust rigid body dynamics, stacking, CCD, friction/restitution stability.
- No constraints/joints, ragdolls, articulated bodies.
- Limited collision shapes and no mesh/heightfield support.

---

## Design Goals

1) **LLM Control Surface remains engine-owned**  
   - The LLM emits actions in a structured form (Action Spec).
   - IK/motion layer interprets the action and produces bone targets.
   - PhysX only handles collisions/contacts and forces/impulses.

2) **Backend independence**  
   - All gameplay and animation systems use a stable `PhysicsWorld` API.
   - PhysX is one backend (others can be added later).

3) **No overlap / no ambiguity**  
   - Remove legacy `physics_basic` and any old colliders once PhysX is integrated.
   - Avoid two physics systems running at once.

---

## Phased Implementation Plan (Expanded)

### Phase 0 — Decisions & Scope Lock

**Goal:** Make the irreversible decisions before touching core code.

Decisions to confirm:
- Platforms: Linux only, or Linux + Windows + Mac.
- Character Controller: PhysX CCT vs custom kinematic capsule with sweeps.
- Feature scope for first integration:
  - Rigid bodies, shapes, queries, and contacts only.
  - No vehicles/cloth initially.
- Whether to keep `physics_basic` as a fallback (recommended: remove after parity).

Deliverable:
- Signed-off decisions document (can be a section in this file).

---

### Phase 1 — Physics Abstraction Layer (No PhysX Yet)

**Goal:** Isolate the engine from any single physics implementation.

New interfaces (suggested):
- `PhysicsWorld`
  - create/destroy bodies
  - create/destroy shapes
  - set transforms
  - apply forces/impulses
  - step simulation
  - scene queries: raycast / sweep / overlap
  - contact callbacks

ECS-level components (new or refactor):
- `PhysicsBody`  
  - type: static / dynamic / kinematic
  - mass, velocity, damping, gravity scale
- `PhysicsShape`  
  - type: box / sphere / capsule / mesh / heightfield
  - local offset, material
- `PhysicsMaterial`  
  - friction, restitution
- `PhysicsController`  
  - character controller parameters

Actions:
- Add new headers for the interface (`core/include/rkg/physics_api.h`)
- Make the runtime call through the interface, not `physics_basic`.
- Implement a stub backend that does nothing (so engine still builds).

Deliverable:
- Engine compiles with an abstraction layer even without PhysX.

---

### Phase 2 — PhysX Backend Plugin

**Goal:** Implement PhysX behind the abstraction.

Tasks:
- Add PhysX as third_party dependency.
- Create new plugin: `plugins/physics/physx`
  - Implements `PhysicsWorld` interface.
  - Converts ECS components to PhysX actors/shapes.
  - Handles contact callbacks and query pipeline.
- Add build integration (CMake and presets).

Deliverable:
- Plugin loads and runs in the editor with a basic static scene and a dynamic cube.

---

### Phase 3 — Migration of Runtime Systems

**Goal:** Replace all old collision usage with PhysX queries and contact events.

Tasks:
- Replace `physics_basic` character movement with:
  - PhysX CCT *or* kinematic capsule + sweeps.
- Modify `core/src/locomotion.cpp`:
  - Replace `registry.colliders()` usage with `PhysicsWorld` queries.
  - Keep IK and gait logic unchanged, only change ground probing and collision tests.
- Add contact-driven action logic:
  - If hand hits wall → clamp IK target, add recoil.
  - If hand hits dynamic body → apply impulse, notify target entity.

Deliverable:
- Character movement and IK work using PhysX.
- Action collisions behave as expected (wall stops punch, target receives hit).

---

### Phase 4 — Scrub Legacy Code (No Overlap)

**Goal:** Remove old code to avoid accidental overlap and confusion.

Files to remove or refactor:
- `plugins/physics/basic/src/physics_basic.cpp`
- `plugins/physics/basic/CMakeLists.txt`
- `RKG_ENABLE_PHYSICS_BASIC` compile flag and references
- `runtime/src/runtime_host.cpp` plugin registration for `physics_basic`
- `ecs::Collider` and `ecs::RigidBody` if fully superseded by PhysX components
- `core/src/locomotion.cpp` collider-based world collision sections

Deliverable:
- No build path for legacy physics.
- Single source of truth for physics: PhysX backend.

---

### Phase 5 — LLM Action/IK Layer (Physics-Agnostic)

**Goal:** Formalize the LLM control surface that generates actions.

Define an Action Spec schema:
- `action`: "punch" / "kick" / "reach" / "step"
- `targets`: bones + desired world positions
- `timing`: start / peak / follow-through
- `constraints`: max extension, joint limits, alignment
- `collision_rules`: stop_on_hit, transfer_impulse, redirect, etc.

Runtime flow:
1) LLM emits Action Spec.
2) IK stack solves to the spec each frame.
3) PhysX collisions drive events:
   - On hit with dynamic → apply impulse, trigger hit reaction.
   - On hit with static → clamp IK target, add recoil.

Deliverable:
- LLM can create interactive actions without authored animation clips.

---

## Deletion Checklist (Scrub List)

Legacy removals after parity:
- `plugins/physics/basic/` entire folder
- `RKG_ENABLE_PHYSICS_BASIC` defines and usage
- `ecs::Collider` and `ecs::RigidBody` maps in `core/include/rkg/ecs.h` and `core/src/ecs.cpp`
- `locomotion.cpp` collider-based queries replaced by `PhysicsWorld`
- legacy movement fallback in `runtime_host.cpp`

---

## Risks & Mitigations

1) **Complexity of PhysX integration**
   - Mitigation: start with rigid bodies + simple shapes only.

2) **IK instability due to contact conflicts**
   - Mitigation: define clean hand/foot contact policies and clamp IK goals on contact.

3) **Overlap of physics systems**
   - Mitigation: hard delete `physics_basic` after PhysX parity is proven.

---

## Immediate Next Steps

1) Confirm Phase 0 decisions (platform, CCT choice, scope).
2) I will create the abstraction layer scaffolding.
3) Integrate PhysX plugin behind the abstraction.
4) Migrate locomotion to queries.
5) Remove old code.

---

## Appendix: References in Current Codebase

- `plugins/physics/basic/src/physics_basic.cpp`  
- `core/include/rkg/ecs.h`  
- `core/src/locomotion.cpp`  
- `runtime/src/runtime_host.cpp`

