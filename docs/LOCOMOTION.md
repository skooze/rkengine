# Procedural Locomotion (Phase 4)

This engine uses a procedural gait system that drives the skeleton pose each frame from
movement state (speed, direction, grounded) without authored clips. The Vulkan renderer
skins the mesh from the ECS skeleton pose; no locomotion logic lives in the renderer.

## Enable / Disable

- **Enable**: `CharacterController.enable_procedural_gait = true` (default).
- **Per-entity settings** live in `ecs::ProceduralGait`. If the component is absent,
  the rig stays in bind pose.

## How it Works

1. **Physics** (`plugins/physics/basic/src/physics_basic.cpp`)
   - Kinematic capsule movement with acceleration, friction, braking, jump, and ground detection.
2. **Locomotion** (`core/src/locomotion.cpp`)
   - Builds a rig map by bone name (Meshy.ai naming).
   - Computes gait phase, foot planting, foot targets, pelvis motion, arm swing, and IK.
   - Writes `Skeleton::bones[i].local_pose` each tick.
3. **Rendering**
   - Renderer receives the current ECS skeleton pose via `set_vulkan_viewport_skinned_pose()`
     and computes skin matrices.

## Tuning Parameters (ProceduralGait)

Key tunables (per entity):

- `walk_speed`, `sprint_speed`: gait speed references (sync with controller speeds).
- `stride_scale`, `step_height_scale`: stride length and foot lift scaling.
- `pelvis_bob_scale`, `pelvis_sway_scale`, `pelvis_roll_scale`, `pelvis_lean_scale`: body motion.
- `arm_swing_scale`, `arm_tuck`: arm swing amplitude and tuck.
- `turn_in_place_speed`, `turn_step_rate`: in-place turning thresholds.
- `foot_lock_in`, `foot_lock_out`: stance timing for foot locking.
- `ik_blend_speed`: IK blend in/out.
- `landing_compress`, `landing_recover`: landing compression + rebound.
- `input_smooth_tau`: smoothing of velocity/intent driving the gait.

Rig measurements (leg length, hip width, foot length) are derived from the bind pose
to keep motion consistent across Meshy.ai humanoids.

### Runtime Overrides (Environment)

You can override key gait parameters per run:

- `RKG_GAIT_WALK_SPEED`
- `RKG_GAIT_SPRINT_SPEED`
- `RKG_GAIT_STRIDE_SCALE`
- `RKG_GAIT_STEP_HEIGHT`
- `RKG_GAIT_PELVIS_BOB`
- `RKG_GAIT_PELVIS_SWAY`
- `RKG_GAIT_PELVIS_ROLL`
- `RKG_GAIT_PELVIS_LEAN`
- `RKG_GAIT_ARM_SWING`
- `RKG_GAIT_ARM_TUCK`
- `RKG_GAIT_IK_BLEND`

## Debug

Set `ProceduralGait.debug_draw = true` to render:
- Foot targets and lock positions.
- IK chain lines (hip->knee->foot).

Use this to validate foot planting and IK stability.

### Debug Toggle

`RKG_GAIT_DEBUG=0` disables gait debug draw/logging at runtime. If unset, the
default game mode enables debug draw for the player rig.

### Regression Check

After a play session (which writes `build_logs/rkg_play_movement.log`), run:

```bash
./scripts/check_gait_log.py build_logs/rkg_play_movement.log
```

The script fails on knee-plane flips, midline crossings, or foot targets collapsing
inside the minimum radial clamp.
