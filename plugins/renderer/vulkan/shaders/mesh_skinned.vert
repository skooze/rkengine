#version 450

layout(location = 0) in vec3 in_pos;
layout(location = 1) in vec2 in_uv;
layout(location = 2) in uvec4 in_joints;
layout(location = 3) in vec4 in_weights;

layout(set = 0, binding = 1) readonly buffer JointMatrices {
  mat4 joints[];
} joint_mats;

layout(push_constant) uniform PushData {
  mat4 mvp;
  vec4 color;
} push;

layout(location = 0) out vec2 v_uv;

void main() {
  mat4 skin =
      in_weights.x * joint_mats.joints[in_joints.x] +
      in_weights.y * joint_mats.joints[in_joints.y] +
      in_weights.z * joint_mats.joints[in_joints.z] +
      in_weights.w * joint_mats.joints[in_joints.w];
  vec4 world_pos = skin * vec4(in_pos, 1.0);
  gl_Position = push.mvp * world_pos;
  v_uv = in_uv;
}
