#include "rkg/math.h"

namespace rkg {

Mat4 mat4_identity() {
  Mat4 out{};
  out.m[0] = 1.0f;
  out.m[5] = 1.0f;
  out.m[10] = 1.0f;
  out.m[15] = 1.0f;
  return out;
}

Mat4 mat4_mul(const Mat4& a, const Mat4& b) {
  Mat4 out{};
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      out.m[col * 4 + row] =
          a.m[0 * 4 + row] * b.m[col * 4 + 0] +
          a.m[1 * 4 + row] * b.m[col * 4 + 1] +
          a.m[2 * 4 + row] * b.m[col * 4 + 2] +
          a.m[3 * 4 + row] * b.m[col * 4 + 3];
    }
  }
  return out;
}

Vec3 vec3_sub(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 vec3_add(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 vec3_mul(const Vec3& a, float s) {
  return {a.x * s, a.y * s, a.z * s};
}

float vec3_dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 vec3_cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3 vec3_normalize(const Vec3& v) {
  const float len = std::sqrt(vec3_dot(v, v));
  if (len <= 0.00001f) {
    return {0.0f, 0.0f, 0.0f};
  }
  const float inv = 1.0f / len;
  return {v.x * inv, v.y * inv, v.z * inv};
}

Mat4 mat4_look_at(const Vec3& eye, const Vec3& target, const Vec3& up) {
  const Vec3 f = vec3_normalize(vec3_sub(target, eye));
  const Vec3 s = vec3_normalize(vec3_cross(up, f));
  const Vec3 u = vec3_cross(f, s);

  Mat4 out = mat4_identity();
  out.m[0] = s.x;
  out.m[4] = s.y;
  out.m[8] = s.z;
  out.m[1] = u.x;
  out.m[5] = u.y;
  out.m[9] = u.z;
  out.m[2] = -f.x;
  out.m[6] = -f.y;
  out.m[10] = -f.z;
  out.m[12] = -vec3_dot(s, eye);
  out.m[13] = -vec3_dot(u, eye);
  out.m[14] = vec3_dot(f, eye);
  return out;
}

Mat4 mat4_perspective(float fov_deg, float aspect, float znear, float zfar) {
  Mat4 out{};
  const float fov_rad = fov_deg * 3.14159265f / 180.0f;
  const float f = 1.0f / std::tan(fov_rad * 0.5f);
  out.m[0] = f / aspect;
  out.m[5] = -f; // flip Y for Vulkan NDC
  out.m[10] = zfar / (znear - zfar);
  out.m[11] = -1.0f;
  out.m[14] = (zfar * znear) / (znear - zfar);
  return out;
}

Mat4 mat4_translation(const Vec3& t) {
  Mat4 out = mat4_identity();
  out.m[12] = t.x;
  out.m[13] = t.y;
  out.m[14] = t.z;
  return out;
}

Mat4 mat4_scale(const Vec3& s) {
  Mat4 out = mat4_identity();
  out.m[0] = s.x;
  out.m[5] = s.y;
  out.m[10] = s.z;
  return out;
}

Mat4 mat4_rotation_x(float r) {
  Mat4 out = mat4_identity();
  const float c = std::cos(r);
  const float s = std::sin(r);
  out.m[5] = c;
  out.m[6] = s;
  out.m[9] = -s;
  out.m[10] = c;
  return out;
}

Mat4 mat4_rotation_y(float r) {
  Mat4 out = mat4_identity();
  const float c = std::cos(r);
  const float s = std::sin(r);
  out.m[0] = c;
  out.m[2] = -s;
  out.m[8] = s;
  out.m[10] = c;
  return out;
}

Mat4 mat4_rotation_z(float r) {
  Mat4 out = mat4_identity();
  const float c = std::cos(r);
  const float s = std::sin(r);
  out.m[0] = c;
  out.m[1] = s;
  out.m[4] = -s;
  out.m[5] = c;
  return out;
}

Mat4 mat4_rotation_xyz(const Vec3& r) {
  return mat4_mul(mat4_rotation_z(r.z), mat4_mul(mat4_rotation_y(r.y), mat4_rotation_x(r.x)));
}

} // namespace rkg
