#pragma once

#include "core.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================================================== */
/* >TYPEDEFS                                                                                      */
/* ============================================================================================== */

#define VEC2_TYPEDEF(NAME, T) typedef union NAME { struct { T x, y;       }; T v[2]; } NAME;
#define VEC3_TYPEDEF(NAME, T) typedef union NAME { struct { T x, y, z;    }; T v[3]; } NAME;
#define VEC4_TYPEDEF(NAME, T) typedef union NAME { struct { T x, y, z, w; }; T v[4]; } NAME;

VEC2_TYPEDEF(vec2f, float)
VEC2_TYPEDEF(vec2i, int32_t)

VEC3_TYPEDEF(vec3f, float)
VEC3_TYPEDEF(vec3i, int32_t)

VEC4_TYPEDEF(vec4f, float)
VEC4_TYPEDEF(vec4i, int32_t)

typedef union mat3f {
    float v[9];
    float cols[3][3];
    vec3f col[3];
} mat3f;

typedef union mat4f {
    float v[16];
    float cols[4][4];
    vec4f col[4];
} mat4f;

#define AABB2_TYPEDEF(NAME, VEC_T, T) typedef union NAME { struct { VEC_T min, max; }; struct { T min_x, min_y,        max_x, max_y;        }; T v[4]; } NAME;
#define AABB3_TYPEDEF(NAME, VEC_T, T) typedef union NAME { struct { VEC_T min, max; }; struct { T min_x, min_y, min_z, max_x, max_y, max_z; }; T v[6]; } NAME;

AABB2_TYPEDEF(aabb2f, vec2f, float)
AABB2_TYPEDEF(aabb2i, vec2i, int32_t)

AABB3_TYPEDEF(aabb3f, vec3f, float)
AABB3_TYPEDEF(aabb3i, vec3i, int32_t)

typedef union quatf {
    struct { float x, y, z, w; };
    vec3f  xyz;
    vec4f  xyzw;
    float  v[4];
} quatf;

/* ============================================================================================== */
/* >VECTOR LOGIC                                                                                  */
/* ============================================================================================== */

#define _DEF_VEC_BINOP(NAME, DIM, FN, OP)                                                          \
    static inline NAME NAME##_##FN(NAME a, NAME b) {                                               \
        NAME r = {0};                                                                              \
        for (size_t i = 0; i < (DIM); ++i) r.v[i] = a.v[i] OP b.v[i];                              \
        return r;                                                                                  \
    }

#define DEF_VEC_OPS(NAME, T, DIM)                                                                  \
    _DEF_VEC_BINOP(NAME, DIM, add, +)                                                              \
    _DEF_VEC_BINOP(NAME, DIM, sub, -)                                                              \
    _DEF_VEC_BINOP(NAME, DIM, mul, *)                                                              \
    _DEF_VEC_BINOP(NAME, DIM, div, /)                                                              \
    static inline NAME NAME##_scale(NAME v, T s) {                                                 \
        NAME r = {0};                                                                              \
        for (size_t i = 0; i < (DIM); ++i) r.v[i] = v.v[i] * s;                                    \
        return r;                                                                                  \
    }                                                                                              \
    static inline T NAME##_dot(NAME a, NAME b) {                                                   \
        T sum = 0;                                                                                 \
        for (size_t i = 0; i < (DIM); ++i) sum += a.v[i] * b.v[i];                                 \
        return sum;                                                                                \
    }                                                                                              \
    static inline T     NAME##_len_sq(NAME v) { return NAME##_dot(v, v); }                         \
    static inline float NAME##_len(NAME v)    { return sqrtf((float)NAME##_len_sq(v)); }

DEF_VEC_OPS(vec2f, float,   2)
DEF_VEC_OPS(vec2i, int32_t, 2)

DEF_VEC_OPS(vec3f, float,   3)
DEF_VEC_OPS(vec3i, int32_t, 3)

DEF_VEC_OPS(vec4f, float,   4)
DEF_VEC_OPS(vec4i, int32_t, 4)

#define DEF_VEC_FLOAT_OPS(NAME)                                                                        \
    static inline NAME NAME##_norm(NAME v) {                                                       \
        float l = NAME##_len(v);                                                                   \
        return l > 0.0f ? NAME##_scale(v, 1.0f / l) : (NAME){0};                                   \
    }                                                                                              \
    static inline NAME NAME##_dir(NAME from, NAME to) { return NAME##_norm(NAME##_sub(to, from)); }

DEF_VEC_FLOAT_OPS(vec2f)
DEF_VEC_FLOAT_OPS(vec3f)
DEF_VEC_FLOAT_OPS(vec4f)

#define DEF_VEC_CROSS2(NAME, T)                                                                    \
    static inline T NAME##_cross(NAME a, NAME b) {                                                 \
        return (T)(a.x * b.y - a.y * b.x);                                                         \
    }

#define DEF_VEC_CROSS3(NAME)                                                                       \
    static inline NAME NAME##_cross(NAME a, NAME b) {                                              \
        return (NAME){                                                                             \
            .x = a.y * b.z - a.z * b.y,                                                            \
            .y = a.z * b.x - a.x * b.z,                                                            \
            .z = a.x * b.y - a.y * b.x,                                                            \
        };                                                                                         \
    }

DEF_VEC_CROSS2(vec2f, float)
DEF_VEC_CROSS2(vec2i, int32_t)

DEF_VEC_CROSS3(vec3f)
DEF_VEC_CROSS3(vec3i)

#define _VEC_GEN_BINARY(FN, a, b) _Generic((a), vec2f: vec2f_##FN, vec2i: vec2i_##FN, vec3f: vec3f_##FN, vec3i: vec3i_##FN, vec4f: vec4f_##FN, vec4i: vec4i_##FN)((a), (b))

#define vec_add(a, b)   _VEC_GEN_BINARY(add, a, b)
#define vec_sub(a, b)   _VEC_GEN_BINARY(sub, a, b)
#define vec_mul(a, b)   _VEC_GEN_BINARY(mul, a, b)
#define vec_div(a, b)   _VEC_GEN_BINARY(div, a, b)
#define vec_scale(a, s) _VEC_GEN_BINARY(scale, a, s)
#define vec_dot(a, b)   _VEC_GEN_BINARY(dot, a, b)

#define _VEC_GEN_UNARY(FN, v) _Generic((v), vec2f: vec2f_##FN, vec2i: vec2i_##FN, vec3f: vec3f_##FN, vec3i: vec3i_##FN, vec4f: vec4f_##FN, vec4i: vec4i_##FN)((v))
    
#define vec_len_sq(v) _VEC_GEN_UNARY(len_sq, v)
#define vec_len(v)    _VEC_GEN_UNARY(len, v)

#define _VEC_GEN_UNARY_FLOAT(FN, v) _Generic((v), vec2f: vec2f_##FN, vec3f: vec3f_##FN, vec4f: vec4f_##FN)((v))

#define vec_norm(v) _VEC_GEN_UNARY_FLOAT(norm, v)

#define _VEC_GEN_BINARY_FLOAT(FN, a, b) _Generic((a), vec2f: vec2f_##FN, vec3f: vec3f_##FN, vec4f: vec4f_##FN)((a), (b))

#define vec_dir(from, to) _VEC_GEN_BINARY_FLOAT(dir, from, to)

#define _VEC_GEN_BINARY_23(FN, a, b)  _Generic((a), vec2f: vec2f_##FN, vec2i: vec2i_##FN, vec3f: vec3f_##FN, vec3i: vec3i_##FN)((a), (b))

#define vec_cross(a, b) _VEC_GEN_BINARY_23(cross, a, b)

/* ============================================================================================== */
/* >MATRIX LOGIC                                                                                  */
/* ============================================================================================== */

#define DEF_MAT_OPS(NAME, VEC_T, DIM)                                                              \
    static inline NAME NAME##_id(void) {                                                           \
        NAME m = {0};                                                                              \
        for (size_t i = 0; i < (DIM); ++i) m.cols[i][i] = 1.0f;                                    \
        return m;                                                                                  \
    }                                                                                              \
    static inline VEC_T NAME##_mul_vec(NAME m, VEC_T v) {                                          \
        VEC_T r = {0};                                                                             \
        for (size_t col = 0; col < (DIM); ++col) {                                                 \
            float s = v.v[col];                                                                    \
            for (size_t row = 0; row < (DIM); ++row)                                               \
                r.v[row] += m.cols[col][row] * s;                                                  \
        }                                                                                          \
        return r;                                                                                  \
    }                                                                                              \
    static inline NAME NAME##_mul(NAME a, NAME b) {                                                \
        NAME r = {0};                                                                              \
        for (size_t col = 0; col < (DIM); ++col)                                                   \
            r.col[col] = NAME##_mul_vec(a, b.col[col]);                                            \
        return r;                                                                                  \
    }                                                                                              \
    static inline NAME NAME##_transp(NAME m) {                                                     \
        NAME r = {0};                                                                              \
        for (size_t col = 0; col < (DIM); ++col)                                                   \
            for (size_t row = 0; row < (DIM); ++row)                                               \
                r.cols[col][row] = m.cols[row][col];                                               \
        return r;                                                                                  \
    }

DEF_MAT_OPS(mat3f, vec3f, 3)
DEF_MAT_OPS(mat4f, vec4f, 4)

static inline mat3f mat3f_from_mat4f(mat4f m) {
    return (mat3f){ .col = {
        { .x = m.col[0].x, .y = m.col[0].y, .z = m.col[0].z },
        { .x = m.col[1].x, .y = m.col[1].y, .z = m.col[1].z },
        { .x = m.col[2].x, .y = m.col[2].y, .z = m.col[2].z },
    } };
}

static inline float mat3f_det(mat3f m) {
    return m.col[0].x * (m.col[1].y * m.col[2].z - m.col[1].z * m.col[2].y) -
           m.col[1].x * (m.col[0].y * m.col[2].z - m.col[0].z * m.col[2].y) +
           m.col[2].x * (m.col[0].y * m.col[1].z - m.col[0].z * m.col[1].y);
}

static inline mat3f mat3f_inv(mat3f m) {
    float c00 =  (m.col[1].y * m.col[2].z - m.col[2].y * m.col[1].z);
    float c10 = -(m.col[1].x * m.col[2].z - m.col[2].x * m.col[1].z);
    float c20 =  (m.col[1].x * m.col[2].y - m.col[2].x * m.col[1].y);

    float det = m.col[0].x * c00 + m.col[0].y * c10 + m.col[0].z * c20;
    if (fabsf(det) <= 1e-6f) return (mat3f){0};

    float inv_det = 1.0f / det;
    return (mat3f){ .col = {
        { .x = c00 * inv_det, .y = -(m.col[0].y * m.col[2].z - m.col[2].y * m.col[0].z) * inv_det, .z =  (m.col[0].y * m.col[1].z - m.col[1].y * m.col[0].z) * inv_det },
        { .x = c10 * inv_det, .y =  (m.col[0].x * m.col[2].z - m.col[2].x * m.col[0].z) * inv_det, .z = -(m.col[0].x * m.col[1].z - m.col[1].x * m.col[0].z) * inv_det },
        { .x = c20 * inv_det, .y = -(m.col[0].x * m.col[2].y - m.col[2].x * m.col[0].y) * inv_det, .z =  (m.col[0].x * m.col[1].y - m.col[1].x * m.col[0].y) * inv_det },
    } };
}

static inline mat3f mat3f_normal(mat4f model) {
    return mat3f_transp(mat3f_inv(mat3f_from_mat4f(model)));
}

static inline mat4f mat4f_trans(vec3f v) {
    mat4f m    = mat4f_id();
    m.col[3].x = v.x;
    m.col[3].y = v.y;
    m.col[3].z = v.z;
    return m;
}

static inline mat4f mat4f_rotz(float angle_rad) {
    mat4f m = mat4f_id();
    float c = cosf(angle_rad);
    float s = sinf(angle_rad);
    m.col[0].x = c; m.col[1].x = -s;
    m.col[0].y = s; m.col[1].y =  c;
    return m;
}

static inline mat4f mat4f_scale(vec3f scale) {
    mat4f m = mat4f_id();
    m.col[0].x = scale.x;
    m.col[1].y = scale.y;
    m.col[2].z = scale.z;
    return m;
}

static inline mat4f mat4f_trans_2d(vec2f pos, float angle_rad, float scale) {
    float c = cosf(angle_rad) * scale;
    float s = sinf(angle_rad) * scale;
    return (mat4f){ .v = {
        c,     s,     0.0f, 0.0f,
       -s,     c,     0.0f, 0.0f,
        0.0f,  0.0f,  1.0f, 0.0f,
        pos.x, pos.y, 0.0f, 1.0f,
    } };
}

static inline mat4f mat4f_ortho(float left, float right, float bot, float top, float near, float far) {
    mat4f m    =  {0};
    m.col[0].x =  2.0f           / (right - left);
    m.col[1].y =  2.0f           / (top   - bot);
    m.col[2].z = -2.0f           / (far   - near);
    m.col[3].x = -(right + left) / (right - left);
    m.col[3].y = -(top   + bot)  / (top   - bot);
    m.col[3].z = -(far   + near) / (far   - near);
    m.col[3].w =  1.0f;
    return m;
}

static inline mat4f mat4f_persp(float fov_rad, float aspect, float near, float far) {
    mat4f m    =  {0};
    float f    =  1.0f                / tanf(fov_rad * 0.5f);
    m.col[0].x =  f                   / aspect;
    m.col[1].y =  f;
    m.col[2].z =  (far + near)        / (near - far);
    m.col[2].w = -1.0f;
    m.col[3].z =  (2.0f * far * near) / (near - far);
    return m;
}

static inline vec4f mat4f_mul_vec4f(mat4f m, vec4f v) {
        return (vec4f){
            .x = m.col[0].x * v.x + m.col[1].x * v.y + m.col[2].x * v.z + m.col[3].x * v.w,
            .y = m.col[0].y * v.x + m.col[1].y * v.y + m.col[2].y * v.z + m.col[3].y * v.w,
            .z = m.col[0].z * v.x + m.col[1].z * v.y + m.col[2].z * v.z + m.col[3].z * v.w,
            .w = m.col[0].w * v.x + m.col[1].w * v.y + m.col[2].w * v.z + m.col[3].w * v.w,
        };
    }

/* ============================================================================================== */
/* >AABB LOGIC                                                                                    */
/* ============================================================================================== */

#define DEF_AABB_OPS(NAME, VEC_T, DIM)                                                             \
    static inline NAME NAME##_pos_size(VEC_T pos, VEC_T size) {                                    \
        return (NAME){ .min = pos, .max = VEC_T##_add(pos, size) };                                \
    }                                                                                              \
    static inline NAME NAME##_center_half(VEC_T center, VEC_T half) {                              \
        return (NAME){ .min = VEC_T##_sub(center, half), .max = VEC_T##_add(center, half) };       \
    }                                                                                              \
    static inline VEC_T NAME##_size(NAME b) {                                                      \
        return VEC_T##_sub(b.max, b.min);                                                          \
    }                                                                                              \
    static inline bool NAME##_inter(NAME a, NAME b) {                                              \
        for (size_t i = 0; i < (DIM); ++i)                                                         \
            if (a.min.v[i] > b.max.v[i] || a.max.v[i] < b.min.v[i]) return false;                  \
        return true;                                                                               \
    }                                                                                              \
    static inline bool NAME##_contains(NAME b, VEC_T p) {                                          \
        for (size_t i = 0; i < (DIM); ++i)                                                         \
            if (p.v[i] < b.min.v[i] || p.v[i] > b.max.v[i]) return false;                          \
        return true;                                                                               \
    }                                                                                              \
    static inline NAME NAME##_merge(NAME a, NAME b) {                                              \
        NAME r = (NAME){0};                                                                        \
        for (size_t i = 0; i < (DIM); ++i) {                                                       \
            r.min.v[i] = MIN(a.min.v[i], b.min.v[i]);                                              \
            r.max.v[i] = MAX(a.max.v[i], b.max.v[i]);                                              \
        }                                                                                          \
        return r;                                                                                  \
    }

#define DEF_AABB_FLOAT_CENTER(NAME, VEC_T)                                                         \
    static inline VEC_T NAME##_center(NAME b) {                                                    \
        return VEC_T##_scale(VEC_T##_add(b.min, b.max), 0.5f);                                     \
    }

DEF_AABB_OPS(aabb2f, vec2f, 2)
DEF_AABB_FLOAT_CENTER(aabb2f, vec2f)
DEF_AABB_OPS(aabb2i, vec2i, 2)

DEF_AABB_OPS(aabb3f, vec3f, 3)
DEF_AABB_FLOAT_CENTER(aabb3f, vec3f)
DEF_AABB_OPS(aabb3i, vec3i, 3)

#define aabb_size(b)                   _Generic((b),      aabb2f: aabb2f_size,        aabb2i: aabb2i_size,        aabb3f: aabb3f_size,         aabb3i: aabb3i_size)((b))
#define aabb_center(b)                 _Generic((b),      aabb2f: aabb2f_center,                                  aabb3f: aabb3f_center                           )((b))
#define aabb_inter(a, b)               _Generic((a),      aabb2f: aabb2f_inter,       aabb2i: aabb2i_inter,       aabb3f: aabb3f_inter,        aabb3i: aabb3i_inter)((a), (b))
#define aabb_contains(b, p)            _Generic((b),      aabb2f: aabb2f_contains,    aabb2i: aabb2i_contains,    aabb3f: aabb3f_contains,     aabb3i: aabb3i_contains)((b), (p))
#define aabb_merge(a, b)               _Generic((a),      aabb2f: aabb2f_merge,       aabb2i: aabb2i_merge,       aabb3f: aabb3f_merge,        aabb3i: aabb3i_merge)((a), (b))
#define aabb_pos_size(pos, size)       _Generic((pos),    vec2f:  aabb2f_pos_size,    vec2i:  aabb2i_pos_size,    vec3f:  aabb3f_pos_size,     vec3i:  aabb3i_pos_size)((pos), (size))
#define aabb_center_half(center, half) _Generic((center), vec2f:  aabb2f_center_half, vec2i:  aabb2i_center_half, vec3f:  aabb3f_center_half,  vec3i:  aabb3i_center_half)((center), (half))

/* ============================================================================================== */
/* >QUATERNION LOGIC                                                                              */
/* ============================================================================================== */

static inline quatf quatf_id(void) { return (quatf){ .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f }; }

static inline quatf quatf_axis_angle(vec3f axis, float angle_rad) {
    float h = angle_rad * 0.5f;
    float s = sinf(h);
    return (quatf){
        .x = axis.x * s,
        .y = axis.y * s,
        .z = axis.z * s,
        .w = cosf(h),
    };
}

static inline quatf quatf_from_euler(float pitch, float yaw, float roll) {
    float hp = pitch * 0.5f, hy = yaw * 0.5f, hr = roll * 0.5f;
    float sp = sinf(hp),     sy = sinf(hy),   sr = sinf(hr);
    float cp = cosf(hp),     cy = cosf(hy),   cr = cosf(hr);
    return (quatf) {
        .x = sp * cy * cr - cp * sy * sr,
        .y = cp * sy * cr + sp * cy * sr,
        .z = cp * cy * sr - sp * sy * cr,
        .w = cp * cy * cr + sp * sy * sr,
    };
}

static inline float quatf_dot(quatf a, quatf b) {
    return vec4f_dot(a.xyzw, b.xyzw);
}

static inline quatf quatf_norm(quatf q) {
    float l = vec4f_len(q.xyzw);
    return l > 0.0f ? (quatf){ .xyzw = vec4f_scale(q.xyzw, 1.0f / l) } : quatf_id();
}

static inline quatf quatf_conj(quatf q) {
    return (quatf){ .x = -q.x, .y = -q.y, .z = -q.z, .w = q.w };
}

static inline quatf quatf_mul(quatf a, quatf b) {
   return (quatf){
       .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
       .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
       .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
       .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
   };
}

static inline vec3f quatf_mul_vec3(quatf q, vec3f v) {
    vec3f uv  = vec3f_cross(q.xyz, v);
    vec3f uuv = vec3f_cross(q.xyz, uv);
    return vec3f_add(v, vec3f_scale(vec3f_add(vec3f_scale(uv, q.w), uuv), 2.0f));
}

static inline mat3f quatf_to_mat3f(quatf q) {
    float x2 = q.x * 2.0f, y2 = q.y * 2.0f, z2 = q.z * 2.0f;
    float xx = q.x * x2,   xy = q.x * y2,   xz = q.x * z2;
    float yy = q.y * y2,   yz = q.y * z2,   zz = q.z * z2;
    float wx = q.w * x2,   wy = q.w * y2,   wz = q.w * z2;

    return (mat3f){ .col = {
        { .x = 1.0f - (yy + zz), .y = xy + wz,          .z = xz - wy          },
        { .x = xy - wz,          .y = 1.0f - (xx + zz), .z = yz + wx          },
        { .x = xz + wy,          .y = yz - wx,          .z = 1.0f - (xx + yy) },
    } };
}

static inline mat4f quatf_to_mat4f(quatf q) {
    mat3f m3 = quatf_to_mat3f(q);
    return (mat4f){ .v = {
        m3.col[0].x, m3.col[0].y, m3.col[0].z, 0.0f,
        m3.col[1].x, m3.col[1].y, m3.col[1].z, 0.0f,
        m3.col[2].x, m3.col[2].y, m3.col[2].z, 0.0f,
        0.0f,        0.0f,        0.0f,        1.0f,
    } };
}

static inline quatf quatf_slerp(quatf a, quatf b, float t) {
    float dot = quatf_dot(a, b);
    if (dot < 0.0f) {
        b   = (quatf){ .x = -b.x, .y = -b.y, .z = -b.z, .w = -b.w };
        dot = -dot;
    }

    if (dot > 1.0f - 1e-3f) {
        return quatf_norm((quatf){
            .x = a.x + (b.x - a.x) * t,
            .y = a.y + (b.y - a.y) * t,
            .z = a.z + (b.z - a.z) * t,
            .w = a.w + (b.w - a.w) * t,
        });
    }

    float theta_0     = acosf(dot);
    float theta       = theta_0 * t;
    float sin_theta_0 = sinf(theta_0);
    float sin_theta   = sinf(theta);

    float s0 = cosf(theta) - dot * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;

    return (quatf){
        .x = a.x * s0 + b.x * s1,
        .y = a.y * s0 + b.y * s1,
        .z = a.z * s0 + b.z * s1,
        .w = a.w * s0 + b.w * s1,
    };
}

/* ============================================================================================== */
/* >CAPTURE LOGIC                                                                                 */
/* ============================================================================================== */

static inline bool project_to_capture(vec3f world, mat4f view_proj, int32_t view_w, int32_t view_h, int32_t *out_x, int32_t *out_y) {
    vec4f clip  = mat4f_mul_vec4f(view_proj, (vec4f){ .x = world.x, .y = world.y, .z = world.z, .w = 1.0f });
    if (clip.w <= 0.0f) return false;
    float inv_w = 1.0f   / clip.w;
    float ndc_x = clip.x * inv_w;
    float ndc_y = clip.y * inv_w;
    *out_x      = (int32_t)(        (ndc_x + 1.0f) * 0.5f  * (float)view_w);
    *out_y      = (int32_t)((1.0f - (ndc_y + 1.0f) * 0.5f) * (float)view_h);
    return true;
}
