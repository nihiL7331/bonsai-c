#pragma once

/*
====================
gmath.h

Linear algebra and geometric math pipeline.

Features:
- 2D, 3D and 4D vector types (float and int32) with type-generic macros
- Column-major 3x3 and 4x4 matrices
- Axis-aligned bounding boxes
- Hamilton quaternions
- Viewport projection for headless software capture

Each type explicitly states its dimension count and scalar type in name.
====================
*/

#include "core.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GMATH_HALFPI 1.57079633f
#define GMATH_PI     3.14159265f
#define GMATH_TAU    6.28318530f

/* ============================================================================================== */
/* >TYPEDEFS                                                                                      */
/* ============================================================================================== */

#define VEC2_TYPEDEF(NAME, T) typedef union NAME { struct { T x, y;       }; T v[2]; } NAME;
#define VEC3_TYPEDEF(NAME, T) typedef union NAME { struct { T x, y, z;    }; T v[3]; } NAME;
#define VEC4_TYPEDEF(NAME, T) typedef union NAME { struct { T x, y, z, w; }; T v[4]; } NAME;

/*
====================
vec2f

2D single-precision floating-point vector union.
Accessible via named components `x`, `y` or array `v[2]`.
Occupies exactly 8 bytes.
====================
*/
VEC2_TYPEDEF(vec2f, float)

/*
====================
vec2i

2D 32-bit signed integer vector union.
Accessible via named components `x`, `y` or array `v[2]`.
Occupies exactly 8 bytes.
====================
*/
VEC2_TYPEDEF(vec2i, int32_t)

/*
====================
vec3f

3D single-precision floating-point vector union.
Accessible via named components `x`, `y`, `z` or array `v[3]`.
Occupies exactly 12 bytes.
====================
*/
VEC3_TYPEDEF(vec3f, float)

/*
====================
vec3i

3D 32-bit signed integer vector union.
Accessible via named components `x`, `y`, `z` or array `v[3]`.
Occupies exactly 12 bytes.
====================
*/
VEC3_TYPEDEF(vec3i, int32_t)

/*
====================
vec4f

4D single-precision floating-point vector union.
Accessible via named components `x`, `y`, `z`, `w` or array `v[4]`.
Occupies exactly 16 bytes.
====================
*/
VEC4_TYPEDEF(vec4f, float)

/*
====================
vec4i

4D 32-bit signed integer vector union.
Accessible via named components `x`, `y`, `z`, `w` or array `v[4]`.
Occupies exactly 16 bytes.
====================
*/
VEC4_TYPEDEF(vec4i, int32_t)

/*
====================
mat3f

3x3 column-major matrix representation.
Memory stores 3 contiguous column vectors.
Accessible as scalars via `v[9]`, `cols[3][3]`
or by column (`vec3f`) via `col[3]`.
====================
*/
typedef union mat3f {
    float v[9];
    float cols[3][3];
    vec3f col[3];
} mat3f;

/*
====================
mat4f

4x4 column-major matrix representation.
Memory stores 4 contiguous column vectors.
Accessible as scalars via `v[16]`, `cols[4][4]`
or by column (`vec4f`) via `col[4]`.
====================
*/
typedef union mat4f {
    float v[16];
    float cols[4][4];
    vec4f col[4];
} mat4f;

#define AABB2_TYPEDEF(NAME, VEC_T, T) typedef union NAME { struct { VEC_T min, max; }; struct { T min_x, min_y,        max_x, max_y;        }; T v[4]; } NAME;
#define AABB3_TYPEDEF(NAME, VEC_T, T) typedef union NAME { struct { VEC_T min, max; }; struct { T min_x, min_y, min_z, max_x, max_y, max_z; }; T v[6]; } NAME;

/*
====================
aabb2f

2D floating-point axis-aligned bounding box.
Stores extents `min`, `max` (accessible as `vec2f` or scalars `min_x/y`, `max_x/y`).
Invariants assume `min <= max` on all axes.
====================
*/
AABB2_TYPEDEF(aabb2f, vec2f, float)

/*
====================
aabb2i

2D 32-bit integer axis-aligned bounding box.
Stores extents `min`, `max` (accessible as `vec2i` or scalars `min_x/y`, `max_x/y`).
Invariants assume `min <= max` on all axes.
====================
*/
AABB2_TYPEDEF(aabb2i, vec2i, int32_t)

/*
====================
aabb3f

3D floating-point axis-aligned bounding box.
Stores extents `min`, `max` (accessible as `vec3f` or scalars `min_x/y`, `max_x/y`).
Invariants assume `min <= max` on all axes.
====================
*/
AABB3_TYPEDEF(aabb3f, vec3f, float)

/*
====================
aabb3i

3D 32-bit integer axis-aligned bounding box.
Stores extents `min`, `max` (accessible as `vec3i` or scalars `min_x/y`, `max_x/y`).
Invariants assume `min <= max` on all axes.
====================
*/
AABB3_TYPEDEF(aabb3i, vec3i, int32_t)

/*
====================
quatf

Hamilton orientation quaternion.
Stores vector part `xyz` and scalar real part `w`.
Invariants assume unit length (`quatf_norm`) for pure rotation representations.
====================
*/
typedef union quatf {
    struct { float x, y, z, w; };
    vec3f  xyz;
    vec4f  xyzw;
    float  v[4];
} quatf;

/* ============================================================================================== */
/* >CONSTRUCTORS                                                                                  */
/* ============================================================================================== */

/* vectors */
#define DEF_VEC_CAST(DIM)                                                                          \
    static inline vec##DIM##f vec##DIM##f_from_i(vec##DIM##i v) {                                  \
        vec##DIM##f r = {0};                                                                       \
        for (size_t i = 0; i < (DIM); ++i) r.v[i] = (float)v.v[i];                                 \
        return r;                                                                                  \
    }                                                                                              \
    static inline vec##DIM##f vec##DIM##f_from_f(vec##DIM##f v) { return v; }                      \
    static inline vec##DIM##i vec##DIM##i_from_f(vec##DIM##f v) {                                  \
        vec##DIM##i r = {0};                                                                       \
        for (size_t i = 0; i < (DIM); ++i) r.v[i] = (int32_t)v.v[i];                               \
        return r;                                                                                  \
    }                                                                                              \
    static inline vec##DIM##i vec##DIM##i_from_i(vec##DIM##i v) { return v; }                      \

DEF_VEC_CAST(2)
DEF_VEC_CAST(3)
DEF_VEC_CAST(4)

#define _VEC_CAST(DIM, S1, S2, v) _Generic((v),                                                    \
    vec##DIM##S2: vec##DIM##S1##_from_##S2,                                                        \
    vec##DIM##S1: vec##DIM##S1##_from_##S1)                                                        \
    (v)

#define _vec2f_raw(x, y)       ((vec2f){ .v = { (float)(x),   (float)(y)                               }})
#define _vec2i_raw(x, y)       ((vec2i){ .v = { (int32_t)(x), (int32_t)(y)                             }})
#define _vec3f_raw(x, y, z)    ((vec3f){ .v = { (float)(x),   (float)(y),   (float)(z)                 }})
#define _vec3i_raw(x, y, z)    ((vec3i){ .v = { (int32_t)(x), (int32_t)(y), (int32_t)(z)               }})
#define _vec4f_raw(x, y, z, w) ((vec4f){ .v = { (float)(x),   (float)(y),   (float)(z),   (float)(w)   }})
#define _vec4i_raw(x, y, z, w) ((vec4i){ .v = { (int32_t)(x), (int32_t)(y), (int32_t)(z), (int32_t)(w) }})

#define vec2f_1(v)    _VEC_CAST(2, f, i, v)
#define vec2f_2(x, y) _vec2f_raw(x, y)
#define vec2f(...)    VA_JOIN(vec2f_, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define vec2i_1(v)    _VEC_CAST(2, i, f, v)
#define vec2i_2(x, y) _vec2i_raw(x, y)
#define vec2i(...)    VA_JOIN(vec2i_, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define vec3f_1(v)       _VEC_CAST(3, f, i, v)
#define vec3f_3(x, y, z) _vec3f_raw(x, y, z)
#define vec3f(...)       VA_JOIN(vec3f_, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define vec3i_1(v)       _VEC_CAST(3, i, f, v)
#define vec3i_3(x, y, z) _vec3i_raw(x, y, z)
#define vec3i(...)       VA_JOIN(vec3i_, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define vec4f_1(v)          _VEC_CAST(4, f, i, v)
#define vec4f_4(x, y, z, w) _vec4f_raw(x, y, z, w)
#define vec4f(...)          VA_JOIN(vec4f_, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define vec4i_1(v)          _VEC_CAST(4, i, f, v)
#define vec4i_4(x, y, z, w) _vec4i_raw(x, y, z, w)
#define vec4i(...)          VA_JOIN(vec4i_, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define vec2f_splat(s) ((vec2f){ .v = { (float)(s),   (float)(s)   }})
#define vec2i_splat(s) ((vec2i){ .v = { (int32_t)(s), (int32_t)(s) }})

#define vec3f_splat(s) ((vec3f){ .v = { (float)s,   (float)s,   (float)s   }})
#define vec3i_splat(s) ((vec3i){ .v = { (int32_t)s, (int32_t)s, (int32_t)s }})

#define vec4f_splat(s) ((vec4f){ .v = { (float)s,   (float)s,   (float)s,   (float)s   }})
#define vec4i_splat(s) ((vec4i){ .v = { (int32_t)s, (int32_t)s, (int32_t)s, (int32_t)s }})

/* matrices */
#define mat3f(c0, c1, c2)     ((mat3f){ .col = { vec3f(c0), vec3f(c1), vec3f(c2)            }})
#define mat4f(c0, c1, c2, c3) ((mat4f){ .col = { vec4f(c0), vec4f(c1), vec4f(c2), vec4f(c3) }})

/* aabbs */
#define DEF_AABB_CAST(DIM)                                                                         \
    static inline aabb##DIM##f aabb##DIM##f_from_i(aabb##DIM##i b) {                               \
        return (aabb##DIM##f){ .min = vec##DIM##f(b.min), .max = vec##DIM##f(b.max) };             \
    }                                                                                              \
    static inline aabb##DIM##f aabb##DIM##f_from_f(aabb##DIM##f b) { return b; }                   \
    static inline aabb##DIM##i aabb##DIM##i_from_f(aabb##DIM##f b) {                               \
        return (aabb##DIM##i){ .min = vec##DIM##i(b.min), .max = vec##DIM##i(b.max) };             \
    }                                                                                              \
    static inline aabb##DIM##i aabb##DIM##i_from_i(aabb##DIM##i b) { return b; }

DEF_AABB_CAST(2)
DEF_AABB_CAST(3)

#define _AABB_CAST(DIM, S1, S2, b) _Generic((b),                                                   \
    aabb##DIM##S2: aabb##DIM##S1##_from_##S2,                                                      \
    aabb##DIM##S1: aabb##DIM##S1##_from_##S1                                                       \
    )(b)

#define _aabb2f_raw2(vmin, vmax) ((aabb2f){ .min = vec2f(vmin), .max = vec2f(vmax) })
#define _aabb2i_raw2(vmin, vmax) ((aabb2i){ .min = vec2i(vmin), .max = vec2i(vmax) })

#define _aabb3f_raw2(vmin, vmax) ((aabb3f){ .min = vec3f(vmin), .max = vec3f(vmax) })
#define _aabb3i_raw2(vmin, vmax) ((aabb3i){ .min = vec3i(vmin), .max = vec3i(vmax) })

#define _aabb2f_raw4(x0, y0, x1, y1) ((aabb2f){ .min = vec2f((x0), (y0)), .max = vec2f((x1), (y1)) })
#define _aabb2i_raw4(x0, y0, x1, y1) ((aabb2i){ .min = vec2i((x0), (y0)), .max = vec2i((x1), (y1)) })

#define _aabb3f_raw4(x0, y0, z0, x1, y1, z1) ((aabb3f){ .min = vec3f((x0), (y0), (z0)), .max = vec3f((x1), (y1), (z1)) })
#define _aabb3i_raw4(x0, y0, z0, x1, y1, z1) ((aabb3i){ .min = vec3i((x0), (y0), (z0)), .max = vec3i((x1), (y1), (z1)) })

#define aabb2f_1(b)              _AABB_CAST(2, f, i, b)
#define aabb2f_2(vmin, vmax)     _aabb2f_raw2(vmin, vmax)
#define aabb2f_4(x0, y0, x1, y1) _aabb2f_raw4(x0, y0, x1, y1)
#define aabb2f(...)              VA_JOIN(aabb2f_, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define aabb2i_1(b)              _AABB_CAST(2, i, f, b)
#define aabb2i_2(vmin, vmax)     _aabb2i_raw2(vmin, vmax)
#define aabb2i_4(x0, y0, x1, y1) _aabb2i_raw4(x0, y0, x1, y1)
#define aabb2i(...)              VA_JOIN(aabb2i_, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define aabb3f_1(b)                      _AABB_CAST(3, f, i, b)
#define aabb3f_2(vmin, vmax)             _aabb3f_raw2(vmin, vmax)
#define aabb3f_6(x0, y0, z0, x1, y1, z1) _aabb3f_raw4(x0, y0, z0, x1, y1, z1)
#define aabb3f(...)                      VA_JOIN(aabb3f_, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define aabb3i_1(b)                      _AABB_CAST(3, i, f, b)
#define aabb3i_2(vmin, vmax)             _aabb3i_raw2(vmin, vmax)
#define aabb3i_6(x0, y0, z0, x1, y1, z1) _aabb3i_raw4(x0, y0, z0, x1, y1, z1)
#define aabb3i(...)                      VA_JOIN(aabb3i_, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

/* quaternions */
#define quatf(x, y, z, w) ((quatf){{ (float)(x), (float)(y), (float)(z), (float)(w) }})

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
    static inline NAME NAME##_min(NAME a, NAME b) {                                                \
        NAME r = {0};                                                                              \
        for (size_t i = 0; i < (DIM); ++i) r.v[i] = MIN(a.v[i], b.v[i]);                           \
        return r;                                                                                  \
    }                                                                                              \
    static inline NAME NAME##_max(NAME a, NAME b) {                                                \
        NAME r = {0};                                                                              \
        for (size_t i = 0; i < (DIM); ++i) r.v[i] = MAX(a.v[i], b.v[i]);                           \
        return r;                                                                                  \
    }                                                                                              \
    static inline NAME NAME##_clamp(NAME v, NAME lo, NAME hi) {                                    \
        NAME r = {0};                                                                              \
        for (size_t i = 0; i < (DIM); ++i) r.v[i] = CLAMP(v.v[i], lo.v[i], hi.v[i]);               \
        return r;                                                                                  \
    }                                                                                              \
    static inline NAME NAME##_abs(NAME v) {                                                        \
        NAME r = {0};                                                                              \
        for (size_t i = 0; i < (DIM); ++i) r.v[i] = ABS(v.v[i]);                                   \
        return r;                                                                                  \
    }                                                                                              \
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
        return NAME(                                                                               \
            a.y * b.z - a.z * b.y,                                                                 \
            a.z * b.x - a.x * b.z,                                                                 \
            a.x * b.y - a.y * b.x                                                                  \
        );                                                                                         \
    }

DEF_VEC_CROSS2(vec2f, float)
DEF_VEC_CROSS2(vec2i, int32_t)

DEF_VEC_CROSS3(vec3f)
DEF_VEC_CROSS3(vec3i)

#define _VEC_GEN_BINARY(FN, a, b) _Generic((a), vec2f: vec2f_##FN, vec2i: vec2i_##FN, vec3f: vec3f_##FN, vec3i: vec3i_##FN, vec4f: vec4f_##FN, vec4i: vec4i_##FN)((a), (b))

#define _VEC_VAR2(FN, a, b)       _VEC_GEN_BINARY(FN, a, b)
#define _VEC_VAR3(FN, a, b, c)    _VEC_VAR2(FN, _VEC_VAR2(FN, a, b), c)
#define _VEC_VAR4(FN, a, b, c, d) _VEC_VAR2(FN, _VEC_VAR3(FN, a, b, c), d)

#define _vec_add2(a, b)       _VEC_VAR2(add, a, b)
#define _vec_add3(a, b, c)    _VEC_VAR3(add, a, b, c)
#define _vec_add4(a, b, c, d) _VEC_VAR4(add, a, b, c, d)
#define vec_add(...)          VA_JOIN(_vec_add, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define _vec_min2(a, b)       _VEC_VAR2(min, a, b)
#define _vec_min3(a, b, c)    _VEC_VAR3(min, a, b, c)
#define _vec_min4(a, b, c, d) _VEC_VAR4(min, a, b, c, d)
#define vec_min(...)          VA_JOIN(_vec_min, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define _vec_max2(a, b)       _VEC_VAR2(max, a, b)
#define _vec_max3(a, b, c)    _VEC_VAR3(max, a, b, c)
#define _vec_max4(a, b, c, d) _VEC_VAR4(max, a, b, c, d)
#define vec_max(...)          VA_JOIN(_vec_max, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define vec_sub(a, b)   _VEC_GEN_BINARY(sub,   a, b)
#define vec_mul(a, b)   _VEC_GEN_BINARY(mul,   a, b)
#define vec_div(a, b)   _VEC_GEN_BINARY(div,   a, b)
#define vec_scale(a, s) _VEC_GEN_BINARY(scale, a, s)

/*
====================
vec_dot

Computes the inner scalar product of two vectors.

Returns the inner product scalar.
====================
*/
#define vec_dot(a, b)   _VEC_GEN_BINARY(dot, a, b)

#define _VEC_GEN_UNARY(FN, v) _Generic((v), vec2f: vec2f_##FN, vec2i: vec2i_##FN, vec3f: vec3f_##FN, vec3i: vec3i_##FN, vec4f: vec4f_##FN, vec4i: vec4i_##FN)((v))

#define vec_len_sq(v) _VEC_GEN_UNARY(len_sq, v)
#define vec_len(v)    _VEC_GEN_UNARY(len,    v)
#define vec_abs(v)    _VEC_GEN_UNARY(abs,    v)

#define _VEC_GEN_UNARY_FLOAT(FN, v) _Generic((v), vec2f: vec2f_##FN, vec3f: vec3f_##FN, vec4f: vec4f_##FN)((v))

/*
====================
vec_norm

Normalizes a floating-point vector to unit length.
Guards against zero-division by returning zero vector when length is zero.

Returns the normalized unit vector.
====================
*/
#define vec_norm(v) _VEC_GEN_UNARY_FLOAT(norm, v)

#define _VEC_GEN_BINARY_FLOAT(FN, a, b) _Generic((a), vec2f: vec2f_##FN, vec3f: vec3f_##FN, vec4f: vec4f_##FN)((a), (b))

#define _VEC_GEN_TERNARY(FN, v, a, b) _Generic((v), vec2f: vec2f_##FN, vec2i: vec2i_##FN, vec3f: vec3f_##FN, vec3i: vec3i_##FN, vec4f: vec4f_##FN, vec4i: vec4i_##FN)((v), (a), (b))

#define vec_clamp(v, lo, hi) _VEC_GEN_TERNARY(clamp, v, lo, hi)

/*
====================
vec_dir

Calculates the normalized direction vector
from point `from` to point `to`.

Returns the normalized direction vector.
====================
*/
#define vec_dir(from, to) _VEC_GEN_BINARY_FLOAT(dir, from, to)

#define _VEC_GEN_BINARY_23(FN, a, b)  _Generic((a), vec2f: vec2f_##FN, vec2i: vec2i_##FN, vec3f: vec3f_##FN, vec3i: vec3i_##FN)((a), (b))

/*
====================
vec_cross

Calculates the cross product:
- for 3D vectors returns perpendicular 3D vector `(a x b)`.
- for 2D vectors returns scalar determinant area `(a.x * b.y - a.y * b.x)`.
====================
*/
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
    return mat3f(
        vec3f(m.col[0].x, m.col[0].y, m.col[0].z),
        vec3f(m.col[1].x, m.col[1].y, m.col[1].z),
        vec3f(m.col[2].x, m.col[2].y, m.col[2].z)
    );
}

/*
====================
mat3f_det

Calculates the determinant of a 3x3 matrix.

Returns determinant scalar value.
====================
*/
static inline float mat3f_det(mat3f m) {
    return m.col[0].x * (m.col[1].y * m.col[2].z - m.col[1].z * m.col[2].y) -
           m.col[1].x * (m.col[0].y * m.col[2].z - m.col[0].z * m.col[2].y) +
           m.col[2].x * (m.col[0].y * m.col[1].z - m.col[0].z * m.col[1].y);
}

/*
====================
mat3f_inv

Computes the analytic matrix inverse of a 3x3 matrix via adjugate cofactors.

Returns:
- zero matrix if singular (|det| <= 1e-6).
- inverted 3x3 matrix otherwise.
====================
*/
static inline mat3f mat3f_inv(mat3f m) {
    float c00 =  (m.col[1].y * m.col[2].z - m.col[2].y * m.col[1].z);
    float c10 = -(m.col[1].x * m.col[2].z - m.col[2].x * m.col[1].z);
    float c20 =  (m.col[1].x * m.col[2].y - m.col[2].x * m.col[1].y);

    float det = m.col[0].x * c00 + m.col[0].y * c10 + m.col[0].z * c20;
    if (ABS(det) <= 1e-6f) return (mat3f){0};
    float inv_det = 1.0f / det;

    return mat3f(
        vec3f_scale(vec3f(c00, m.col[2].y * m.col[0].z - m.col[0].y * m.col[2].z, m.col[0].y * m.col[1].z - m.col[1].y * m.col[0].z), inv_det),
        vec3f_scale(vec3f(c10, m.col[0].x * m.col[2].z - m.col[2].x * m.col[0].z, m.col[1].x * m.col[0].z - m.col[0].x * m.col[1].z), inv_det),
        vec3f_scale(vec3f(c20, m.col[2].x * m.col[0].y - m.col[0].x * m.col[2].y, m.col[0].x * m.col[1].y - m.col[1].x * m.col[0].y), inv_det)
    );
}

/*
====================
mat3f_normal

Computes the normal transform matrix (inverse transpose of upper 3x3 model matrix).
Corrects normal vectors under non-uniform scaling.

Returns 3x3 normal matrix.
====================
*/
static inline mat3f mat3f_normal(mat4f model) {
    return mat3f_transp(mat3f_inv(mat3f_from_mat4f(model)));
}

/*
====================
mat4f_trans

Generates a 4x4 translation matrix from a 3D displacement vector `v`.

Returns the 4x4 translation matrix.
====================
*/
static inline mat4f mat4f_trans(vec3f v) {
    return mat4f(
        vec4f(1,   0,   0,   0),
        vec4f(0,   1,   0,   0),
        vec4f(0,   0,   1,   0),
        vec4f(v.x, v.y, v.z, 1)
    );
}

/*
====================
mat4f_rotz

Generates a 4x4 rotation matrix around the
Z-axis from an angle in radians `angle_rad`.

Returns the 4x4 rotation matrix.
====================
*/
static inline mat4f mat4f_rotz(float angle_rad) {
    float c = cosf(angle_rad);
    float s = sinf(angle_rad);
    return mat4f(
        vec4f( c, s, 0, 0),
        vec4f(-s, c, 0, 0),
        vec4f( 0, 0, 1, 0),
        vec4f( 0, 0, 0, 1)
    );
}

/*
====================
mat4f_scale

Generates a 4x4 scaling matrix from a
3D per-axis scale vector `scale`.

Returns the 4x4 scaling matrix.
====================
*/
static inline mat4f mat4f_scale(vec3f scale) {
    return mat4f(
        vec4f(scale.x, 0,       0,       0),
        vec4f(0,       scale.y, 0,       0),
        vec4f(0,       0,       scale.z, 0),
        vec4f(0,       0,       0,       1)
    );
}

/*
====================
mat4f_trans_2d

Combines 2D translation (`pos`),
Z-rotation (`angle_rad`),
and uniform scale (`scale`)
into a single 4x4 affine matrix.

Returns the combined 4x4 transformation matrix.
====================
*/
static inline mat4f mat4f_trans_2d(vec2f pos, float angle_rad, float scale) {
    float c = cosf(angle_rad) * scale;
    float s = sinf(angle_rad) * scale;
    return mat4f(
        vec4f(c,     s,     0, 0),
        vec4f(-s,    c,     0, 0),
        vec4f(0,     0,     1, 0),
        vec4f(pos.x, pos.y, 0, 1)
    );
}

/*
====================
mat4f_ortho

Constructs an orthographic projection matrix
mapping view bounds to [-1, 1] NDC space.

Returns the 4x4 orthographic projection matrix.
====================
*/
static inline mat4f mat4f_ortho(float left, float right, float bot, float top, float near, float far) {
    return mat4f(
        vec4f(2.0f / (right - left),            0,                          0,                            0),
        vec4f(0,                                2.0f / (top - bot),         0,                            0),
        vec4f(0,                                0,                          -2.0f / (far - near),         0),
        vec4f(-(right + left) / (right - left), -(top + bot) / (top - bot), -(far + near) / (far - near), 1)
    );
}

/*
====================
mat4f_persp

Constructs a perspective projection matrix
with right-handed depth range [-1, 1].

Returns the 4x4 perspective projection matrix.
====================
*/
static inline mat4f mat4f_persp(float fov_rad, float aspect, float near, float far) {
    float f = 1.0f / tanf(fov_rad * 0.5f);
    return mat4f(
            vec4f(f / aspect, 0, 0,                                  0),
            vec4f(0,          f, 0,                                  0),
            vec4f(0,          0, (far + near) / (near - far),       -1),
            vec4f(0,          0, (2.0f * far * near) / (near - far), 0)
    );
}

static inline vec4f mat4f_mul_vec4f(mat4f m, vec4f v) {
    return vec4f(
        m.col[0].x * v.x + m.col[1].x * v.y + m.col[2].x * v.z + m.col[3].x * v.w,
        m.col[0].y * v.x + m.col[1].y * v.y + m.col[2].y * v.z + m.col[3].y * v.w,
        m.col[0].z * v.x + m.col[1].z * v.y + m.col[2].z * v.z + m.col[3].z * v.w,
        m.col[0].w * v.x + m.col[1].w * v.y + m.col[2].w * v.z + m.col[3].w * v.w
    );
}

/* ============================================================================================== */
/* >AABB LOGIC                                                                                    */
/* ============================================================================================== */

#define DEF_AABB_OPS(NAME, VEC_T, T, DIM)                                                          \
    static inline NAME NAME##_pos_size(VEC_T pos, VEC_T size) {                                    \
        return NAME(pos, VEC_T##_add(pos, size));                                                  \
    }                                                                                              \
    static inline NAME NAME##_center_half(VEC_T center, VEC_T half) {                              \
        return NAME(VEC_T##_sub(center, half), VEC_T##_add(center, half));                         \
    }                                                                                              \
    static inline VEC_T NAME##_size(NAME b) {                                                      \
        return VEC_T##_sub(b.max, b.min);                                                          \
    }                                                                                              \
    static inline bool NAME##_inter(NAME a, NAME b) {                                              \
        for (size_t i = 0; i < (DIM); ++i)                                                         \
            if (a.min.v[i] > b.max.v[i] || a.max.v[i] < b.min.v[i]) return false;                  \
        return true;                                                                               \
    }                                                                                              \
    static inline bool NAME##_valid(NAME b) {                                                      \
        for (size_t i = 0; i < (DIM); ++i)                                                         \
            if (b.min.v[i] >= b.max.v[i]) return false;                                            \
        return true;                                                                               \
    }                                                                                              \
    static inline bool NAME##_contains(NAME b, VEC_T p) {                                          \
        for (size_t i = 0; i < (DIM); ++i)                                                         \
            if (p.v[i] < b.min.v[i] || p.v[i] > b.max.v[i]) return false;                          \
        return true;                                                                               \
    }                                                                                              \
    static inline NAME NAME##_merge(NAME a, NAME b) {                                              \
        return NAME(                                                                               \
            vec_min(a.min, b.min),                                                                 \
            vec_max(a.max, b.max)                                                                  \
        );                                                                                         \
    }                                                                                              \
    static inline bool NAME##_inter_radius(NAME b, VEC_T center, T radius) {                       \
        VEC_T diff = vec_sub(center, vec_clamp(center, b.min, b.max));                             \
        return vec_dot(diff, diff) <= (radius * radius);                                           \
    }                                                                                              \
    static inline NAME NAME##_clamp(NAME b, NAME bounds) {                                         \
        return NAME(                                                                               \
            vec_clamp(b.min, bounds.min, bounds.max),                                              \
            vec_clamp(b.max, bounds.min, bounds.max)                                               \
        );                                                                                         \
    }                                                                                              \
    static inline NAME NAME##_expand(NAME b, VEC_T v) {                                            \
        return NAME(                                                                               \
            vec_sub(b.min, v),                                                                     \
            vec_add(b.max, v)                                                                      \
        );                                                                                         \
    }

#define DEF_AABB_FLOAT_CENTER(NAME, VEC_T)                                                         \
    static inline VEC_T NAME##_center(NAME b) {                                                    \
        return VEC_T##_scale(VEC_T##_add(b.min, b.max), 0.5f);                                     \
    }

DEF_AABB_OPS(aabb2f, vec2f, float, 2)
DEF_AABB_FLOAT_CENTER(aabb2f, vec2f)
DEF_AABB_OPS(aabb2i, vec2i, int32_t, 2)

DEF_AABB_OPS(aabb3f, vec3f, float, 3)
DEF_AABB_FLOAT_CENTER(aabb3f, vec3f)
DEF_AABB_OPS(aabb3i, vec3i, int32_t, 3)

/*
====================
aabb_size

Computes the extents (dimensions) of an AABB (`max - min`).

Returns the vector representing width, height (and, for 3D, depth).
====================
*/
#define aabb_size(b) _Generic((b), aabb2f: aabb2f_size, aabb2i: aabb2i_size, aabb3f: aabb3f_size, aabb3i: aabb3i_size)((b))

/*
====================
aabb_center

Computes the midpoint center of a floating-point AABB `(min + max) * 0.5`.

Returns the center vector.
====================
*/
#define aabb_center(b) _Generic((b), aabb2f: aabb2f_center, aabb3f: aabb3f_center)((b))

/*
====================
aabb_inter

Tests whether two AABBs overlap.

Returns:
- `true`  if boxes overlap on all axes.
- `false` otherwise.
====================
*/
#define aabb_inter(a, b) _Generic((a), aabb2f: aabb2f_inter, aabb2i: aabb2i_inter, aabb3f: aabb3f_inter, aabb3i: aabb3i_inter)((a), (b))

/*
====================
aabb_valid

Tests whether AABB `b` is valid,
that is for each coordinate min_c < max_c is true.

Returns:
- `true` if `b` is a valid AABB.
- `false` otherwise.
====================
*/
#define aabb_valid(b) _Generic((b), aabb2f: aabb2f_valid, aabb2i: aabb2i_valid, aabb3f: aabb3f_valid, aabb3i: aabb3i_valid)((b))

/*
====================
aabb_contains

Tests whether point `p` resides within
the inclusive bounds of AABB `b`.

Returns:
- `true`  if point is enclosed.
- `false` otherwise.
====================
*/
#define aabb_contains(b, p) _Generic((b), aabb2f: aabb2f_contains, aabb2i: aabb2i_contains, aabb3f: aabb3f_contains, aabb3i: aabb3i_contains)((b), (p))

/*
====================
aabb_merge

Computes the minimal bounding box
enclosing both `a` and `b`.

Returns the merged bounding box.
====================
*/
#define aabb_merge(a, b) _Generic((a), aabb2f: aabb2f_merge, aabb2i: aabb2i_merge, aabb3f: aabb3f_merge, aabb3i: aabb3i_merge)((a), (b))

/*
====================
aabb_inter_radius

Tests intersection between an AABB and
a circle (2D) or sphere (3D).
Uses Jim Arvo's clamped distance algorithm:
clamps sphere center to box extents and checks
if squared distance <= radius^2.

Returns:
- `true`  if circle/sphere intersects or is contained within AABB.
- `false` otherwise.
====================
*/
#define aabb_inter_radius(b, center, radius) _Generic((b), aabb2f: aabb2f_inter_radius, aabb2i: aabb2i_inter_radius, aabb3f: aabb3f_inter_radius, aabb3i: aabb3i_inter_radius)((b), (center), (radius))

/*
====================
aabb_pos_size

Constructs an AABB from a position
origin `pos` and dimensions `size`.

Returns the constructed AABB.
====================
*/
#define aabb_pos_size(pos, size) _Generic((pos), vec2f: aabb2f_pos_size, vec2i: aabb2i_pos_size, vec3f: aabb3f_pos_size, vec3i: aabb3i_pos_size)((pos), (size))

/*
====================
aabb_center_half

Constructs an AABB from a center
point `center` and half-extents `half`.

Returns the constructed AABB.
====================
*/
#define aabb_center_half(center, half) _Generic((center), vec2f: aabb2f_center_half, vec2i: aabb2i_center_half, vec3f: aabb3f_center_half, vec3i: aabb3i_center_half)((center), (half))

/*
====================
aabb_clamp

Constructs an AABB by clamping `b` within `bounds`.

Returns the constructed AABB.
====================
*/
#define aabb_clamp(b, bounds) _Generic((b), aabb2f: aabb2f_clamp, aabb2i: aabb2i_clamp, aabb3f: aabb3f_clamp, aabb3i: aabb3i_clamp)((b), (bounds))

/*
====================
aabb_expand

Constructs an AABB expanding `b` by `v` in each direction.

Returns the constructed AABB in form
[(b.min.x - v.x, b.min.y - v.y), (b.max.x + v.x, b.max_y + v.y)].
====================
*/
#define aabb_expand(b, v) _Generic((b), aabb2f: aabb2f_expand, aabb2i: aabb2i_expand, aabb3f: aabb3f_expand, aabb3i: aabb3i_expand)((b), (v))

/* ============================================================================================== */
/* >QUATERNION LOGIC                                                                              */
/* ============================================================================================== */

static inline quatf quatf_id(void) {
    return quatf(0, 0, 0, 1);
}

/*
====================
quatf_axis_angle

Generates a unit quaternion representing rotation
`angle_rad` radians around `axis`.
Precondition: `axis` must be normalized.

Returns the unit orientation quaternion.
====================
*/
static inline quatf quatf_axis_angle(vec3f axis, float angle_rad) {
    float h = angle_rad * 0.5f;
    float s = sinf(h);
    return quatf(axis.x * s, axis.y * s, axis.z * s, cosf(h));
}

/*
====================
quatf_from_euler

Constructs an orientation quaternion from
Euler angles in radians.
Applied in pitch (X) -> yaw (Y) -> roll (Z) order.

Returns the unit orientation quaternion.
====================
*/
static inline quatf quatf_from_euler(float pitch, float yaw, float roll) {
    float hp = pitch * 0.5f, hy = yaw * 0.5f, hr = roll * 0.5f;
    float sp = sinf(hp),     sy = sinf(hy),   sr = sinf(hr);
    float cp = cosf(hp),     cy = cosf(hy),   cr = cosf(hr);
    return quatf(
        sp * cy * cr - cp * sy * sr,
        cp * sy * cr + sp * cy * sr,
        cp * cy * sr - sp * sy * cr,
        cp * cy * cr + sp * sy * sr
    );
}

static inline float quatf_dot(quatf a, quatf b) {
    return vec4f_dot(a.xyzw, b.xyzw);
}

/*
====================
quatf_norm

Normalizes quaternion to unit length.
Guards against zero division by returning `quatf_id()`.

Returns the normalized unit quaternion.
====================
*/
static inline quatf quatf_norm(quatf q) {
    float l = vec4f_len(q.xyzw);
    return l > 0.0f ? (quatf){ .xyzw = vec4f_scale(q.xyzw, 1.0f / l) } : quatf_id();
}

static inline quatf quatf_conj(quatf q) {
    return quatf(-q.x, -q.y, -q.z, q.w);
}

/*
====================
quatf_mul

Calculates the Hamilton product of quaternions `a` and `b`.
Represents the composite rotation: rotation `b` followed by rotation `a`.

Returns the combined orientation quaternion.
====================
*/
static inline quatf quatf_mul(quatf a, quatf b) {
   return quatf(
       a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
       a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
       a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
       a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
   );
}

/*
====================
quatf_mul_vec3

Rotates 3D vector `v` by unit quaternion `q` using
Rodrigues vector formula. Evaluates rotation
without constructing intermediate 3x3 matrix.

Returns the rotated 3D vector.
====================
*/
static inline vec3f quatf_mul_vec3(quatf q, vec3f v) {
    vec3f uv  = vec3f_cross(q.xyz, v);
    vec3f uuv = vec3f_cross(q.xyz, uv);
    return vec3f_add(v, vec3f_scale(vec3f_add(vec3f_scale(uv, q.w), uuv), 2.0f));
}

/*
====================
quatf_to_mat3f

Converts unit quaternion `q` into a 3x3 rotation matrix.

Returns the 3x3 rotation matrix.
====================
*/
static inline mat3f quatf_to_mat3f(quatf q) {
    float x2 = q.x * 2.0f, y2 = q.y * 2.0f, z2 = q.z * 2.0f;
    float xx = q.x * x2,   xy = q.x * y2,   xz = q.x * z2;
    float yy = q.y * y2,   yz = q.y * z2,   zz = q.z * z2;
    float wx = q.w * x2,   wy = q.w * y2,   wz = q.w * z2;

    return mat3f(
        vec3f(1.0f - (yy + zz), xy + wz,          xz - wy),
        vec3f(xy - wz,          1.0f - (xx + zz), yz + wx),
        vec3f(xz + wy,          yz - wx,          1.0f - (xx + yy))
    );
}

/*
====================
quatf_to_mat4f

Converts unit quaternion `q` into a 4x4
homogeneous rotation matrix.

Returns the 4x4 rotation matrix.
====================
*/
static inline mat4f quatf_to_mat4f(quatf q) {
    mat3f m3 = quatf_to_mat3f(q);
    return mat4f(
        vec4f(m3.col[0].x, m3.col[0].y, m3.col[0].z, 0),
        vec4f(m3.col[1].x, m3.col[1].y, m3.col[1].z, 0),
        vec4f(m3.col[2].x, m3.col[2].y, m3.col[2].z, 0),
        vec4f(0,           0,           0,           1)
    );
}

/*
====================
quatf_slerp

Spherical linear interpolation between orientations
`a` and `b` at factor `t` in [0, 1].
Selects shortest rotational arc by testing dot product sign.
Falls back to normalized linear interpolation for nearly
parallel orientations (|dot| > 0.999).

Returns the interpolated unit quaternion.
====================
*/
static inline quatf quatf_slerp(quatf a, quatf b, float t) {
    float dot = CLAMP(quatf_dot(a, b), -1.0f, 1.0f);
    if (dot < 0.0f) {
        b = quatf(-b.x, -b.y, -b.z, -b.w);
        dot = -dot;
    }

    if (dot > 1.0f - 1e-3f) {
        return quatf_norm(quatf(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t,
            a.w + (b.w - a.w) * t
        ));
    }

    float theta_0     = acosf(dot);
    float theta       = theta_0 * t;
    float sin_theta_0 = sinf(theta_0);
    float sin_theta   = sinf(theta);

    float s0 = cosf(theta) - dot * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;

    return quatf(
        a.x * s0 + b.x * s1,
        a.y * s0 + b.y * s1,
        a.z * s0 + b.z * s1,
        a.w * s0 + b.w * s1
    );
}

/* ============================================================================================== */
/* >CAPTURE LOGIC                                                                                 */
/* ============================================================================================== */

/*
====================
project_to_capture

Transforms a 3D world coordinate into
2D viewport pixel coordinates `(out_x, out_y)`.
Applies `view_proj` homogeneous transformation
with inverted Y for top-left blitter origin.

Returns:
- `true`  if world point is in front of camera (`clip.w > 0`).
- `false` otherwise.
====================
*/
static inline bool project_to_capture(vec3f world, mat4f view_proj, int32_t view_w, int32_t view_h, int32_t *out_x, int32_t *out_y) {
    vec4f clip  = mat4f_mul_vec4f(view_proj, vec4f(world.x, world.y, world.z, 1));
    if (clip.w <= 0.0f) return false;
    float inv_w = 1.0f   / clip.w;
    float ndc_x = clip.x * inv_w;
    float ndc_y = clip.y * inv_w;
    *out_x      = (int32_t)(        (ndc_x + 1.0f) * 0.5f  * (float)view_w);
    *out_y      = (int32_t)((1.0f - (ndc_y + 1.0f) * 0.5f) * (float)view_h);
    return true;
}
