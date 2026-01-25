#version 450
layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uBaseColor;

layout(push_constant) uniform PushData {
  mat4 mvp;
  vec4 color;
} pc;

void main() {
  vec4 tex = texture(uBaseColor, vUV);
  outColor = tex * pc.color;
}
