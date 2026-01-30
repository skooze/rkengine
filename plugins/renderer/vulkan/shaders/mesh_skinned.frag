#version 450

layout(set = 0, binding = 0) uniform sampler2D base_color;

layout(push_constant) uniform PushData {
  mat4 mvp;
  vec4 color;
} push;

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 out_color;

void main() {
  vec4 tex = texture(base_color, v_uv);
  out_color = tex * push.color;
}
