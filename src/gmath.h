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

/* ============================================================================================== */
/* >VECTOR LOGIC                                                                                  */
/* ============================================================================================== */

#define DEF_BINOP(FN_NAME, OP) \
    static inline vec2f vec2f_##FN_NAME(vec2f a, vec2f b) { return (vec2f){ .x = a.x OP b.x, .y = a.y OP b.y }; } \
    static inline vec2i vec2i_##FN_NAME(vec2i a, vec2i b) { return (vec2i){ .x = a.x OP b.x, .y = a.y OP b.y }; } \
    static inline vec3f vec3f_##FN_NAME(vec3f a, vec3f b) { return (vec3f){ .x = a.x OP b.x, .y = a.y OP b.y, .z = a.z OP b.z }; } \
    static inline vec3i vec3i_##FN_NAME(vec3i a, vec3i b) { return (vec3i){ .x = a.x OP b.x, .y = a.y OP b.y, .z = a.z OP b.z }; } \
    static inline vec4f vec4f_##FN_NAME(vec4f a, vec4f b) { return (vec4f){ .x = a.x OP b.x, .y = a.y OP b.y, .z = a.z OP b.z, .w = a.w OP b.w }; } \
    static inline vec4i vec4i_##FN_NAME(vec4i a, vec4i b) { return (vec4i){ .x = a.x OP b.x, .y = a.y OP b.y, .z = a.z OP b.z, .w = a.w OP b.w }; }

DEF_BINOP(add, +)
DEF_BINOP(sub, -)
DEF_BINOP(mul, *)
DEF_BINOP(div, /)

static inline vec2f vec2f_scale(vec2f v, float s)   { return (vec2f){ .x = v.x * s, .y = v.y * s }; }
static inline vec2i vec2i_scale(vec2i v, int32_t s) { return (vec2i){ .x = v.x * s, .y = v.y * s }; }
static inline vec3f vec3f_scale(vec3f v, float s)   { return (vec3f){ .x = v.x * s, .y = v.y * s, .z = v.z * s }; }
static inline vec3i vec3i_scale(vec3i v, int32_t s) { return (vec3i){ .x = v.x * s, .y = v.y * s, .z = v.z * s }; }
static inline vec4f vec4f_scale(vec4f v, float s)   { return (vec4f){ .x = v.x * s, .y = v.y * s, .z = v.z * s, .w = v.w * s }; }
static inline vec4i vec4i_scale(vec4i v, int32_t s) { return (vec4i){ .x = v.x * s, .y = v.y * s, .z = v.z * s, .w = v.w * s }; }

static inline float   vec2f_dot(vec2f a, vec2f b) { return a.x * b.x + a.y * b.y; }
static inline int32_t vec2i_dot(vec2i a, vec2i b) { return a.x * b.x + a.y * b.y; }
static inline float   vec3f_dot(vec3f a, vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline int32_t vec3i_dot(vec3i a, vec3i b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float   vec4f_dot(vec4f a, vec4f b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
static inline int32_t vec4i_dot(vec4i a, vec4i b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

#define DEF_FLOAT_OPS(NAME) \
    static inline float NAME##_len_sq(NAME v) { return NAME##_dot(v, v); } \
    static inline float NAME##_len(NAME v)    { return sqrtf(NAME##_len_sq(v)); } \
    static inline NAME  NAME##_norm(NAME v) { \
        float l = NAME##_len(v); \
        return l > 0.0f ? NAME##_scale(v, 1.0f / l) : (NAME){0}; \
    }

DEF_FLOAT_OPS(vec2f)
DEF_FLOAT_OPS(vec3f)
DEF_FLOAT_OPS(vec4f)

static inline int32_t vec2i_len_sq(vec2i v) { return vec2i_dot(v, v); }
static inline float   vec2i_len(vec2i v)    { return sqrtf((float)vec2i_len_sq(v)); }

static inline int32_t vec3i_len_sq(vec3i v) { return vec3i_dot(v, v); }
static inline float   vec3i_len(vec3i v)    { return sqrtf((float)vec3i_len_sq(v)); }

static inline int32_t vec4i_len_sq(vec4i v) { return vec4i_dot(v, v); }
static inline float   vec4i_len(vec4i v)    { return sqrtf((float)vec4i_len_sq(v)); }

static inline vec2f vec2f_dir(vec2f from, vec2f to) { return vec2f_norm(vec2f_sub(to, from)); }
static inline vec3f vec3f_dir(vec3f from, vec3f to) { return vec3f_norm(vec3f_sub(to, from)); }

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

/* ============================================================================================== */
/* >MATRIX LOGIC                                                                                  */
/* ============================================================================================== */

static inline mat4f mat4f_id(void) {
    return (mat4f){ .v = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 
    } };
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

    static inline mat4f mat4f_mul(mat4f a, mat4f b) {
        return (mat4f){
            .col = {
                mat4f_mul_vec4f(a, b.col[0]),
                mat4f_mul_vec4f(a, b.col[1]),
                mat4f_mul_vec4f(a, b.col[2]),
                mat4f_mul_vec4f(a, b.col[3]),
            }
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
