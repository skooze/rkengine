#pragma once

#include <cmath>

namespace rkg {

struct Vec3 {
  float x;
  float y;
  float z;
};

struct Mat4 {
  float m[16];
};

Mat4 mat4_identity();
Mat4 mat4_mul(const Mat4& a, const Mat4& b);

Vec3 vec3_sub(const Vec3& a, const Vec3& b);
Vec3 vec3_add(const Vec3& a, const Vec3& b);
Vec3 vec3_mul(const Vec3& a, float s);
float vec3_dot(const Vec3& a, const Vec3& b);
Vec3 vec3_cross(const Vec3& a, const Vec3& b);
Vec3 vec3_normalize(const Vec3& v);

Mat4 mat4_look_at(const Vec3& eye, const Vec3& target, const Vec3& up);
Mat4 mat4_perspective(float fov_deg, float aspect, float znear, float zfar);
Mat4 mat4_translation(const Vec3& t);
Mat4 mat4_scale(const Vec3& s);
Mat4 mat4_rotation_x(float r);
Mat4 mat4_rotation_y(float r);
Mat4 mat4_rotation_z(float r);
Mat4 mat4_rotation_xyz(const Vec3& r);

} // namespace rkg
