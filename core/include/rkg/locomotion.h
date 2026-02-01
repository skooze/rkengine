#pragma once

#include "rkg/ecs.h"

namespace rkg::locomotion {

void update_procedural_gaits(ecs::Registry& registry, float dt);
void update_procedural_gait(ecs::Registry& registry, ecs::Entity entity, float dt);

// Debug/test helpers (used by rkg_tests).
ecs::Transform debug_solve_two_bone_ik(const ecs::Transform& hip,
                                       const ecs::Transform& knee,
                                       const ecs::Transform& ankle,
                                       const ecs::Transform& target,
                                       const ecs::Transform& plane_hint);

bool debug_update_foot_lock(bool& locked,
                            float lock_pos[3],
                            const float foot_pos[3],
                            bool grounded,
                            float swing_phase,
                            float lock_in,
                            float lock_out,
                            float dt);

} // namespace rkg::locomotion
