#include "core.h"
#include "gmath.h"
#include "test.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define HALF_PI (3.14159265f * 0.5f)
#define EPS 0.0001f

static inline bool feq(float a, float b) {
    return ABS(a - b) <= EPS;
}

int main(void) {
    TEST_CASE("vec_add", "computes component-wise sum") {
        vec2f a2 = vec2f(1, 2);
        vec2f b2 = vec2f(3, 4);
        vec2f r2 = vec_add(a2, b2);
        TEST_ASSERT(feq(r2.x, 4.0f) && feq(r2.y, 6.0f), "exp (4.0, 6.0), got (%f, %f)", r2.x, r2.y);

        vec3i a3 = vec3i(1, -2, 5);
        vec3i b3 = vec3i(4,  2, 1);
        vec3i r3 = vec_add(a3, b3);
        TEST_ASSERT(r3.x == 5 && r3.y == 0 && r3.z == 6, "exp (5, 0, 6), got (%d, %d, %d)", r3.x, r3.y, r3.z);
    } TEST_CASE_END();

    TEST_CASE("vec_sub", "computes component-wise difference") {
        vec2f a = vec2f(5, 8);
        vec2f b = vec2f(2, 3);
        vec2f r = vec_sub(a, b);
        TEST_ASSERT(feq(r.x, 3.0f) && feq(r.y, 5.0f), "exp (3.0, 5.0), got (%f, %f)", r.x, r.y);
    } TEST_CASE_END();

    TEST_CASE("vec_scale", "scales vector components by scalar") {
        vec2f v = vec2f(2, -3);
        vec2f r = vec_scale(v, 2.5f);
        TEST_ASSERT(feq(r.x, 5.0f) && feq(r.y, -7.5f), "exp (5.0, -7.5), got (%f, %f)", r.x, r.y);

        vec2i vi = vec2i(3, 4);
        vec2i ri = vec_scale(vi, 3);
        TEST_ASSERT(ri.x == 9 && ri.y == 12, "exp (9, 12), got (%d, %d)", ri.x, ri.y);
    } TEST_CASE_END();

    TEST_CASE("vec_dot", "calculates inner scalar product") {
        vec2f a2 = vec2f(1, 2);
        vec2f b2 = vec2f(3, 4);
        float d2 = vec_dot(a2, b2);
        TEST_ASSERT(feq(d2, 11.0f), "exp 11.0, got %f", d2);

        vec3f a3 = vec3f(1, 0, 0);
        vec3f b3 = vec3f(0, 1, 0);
        float d3 = vec_dot(a3, b3);
        TEST_ASSERT(feq(d3, 0.0f), "orthogonal vectors exp dot 0.0, got %f", d3);
    } TEST_CASE_END();

    TEST_CASE("vec_len_sq", "calculates squared magnitude") {
        vec2f v = vec2f(3, 4);
        TEST_ASSERT(feq(vec_len_sq(v), 25.0f), "exp 25.0, got %f", vec_len_sq(v));

        vec2i vi = vec2i(3, 4);
        TEST_ASSERT(vec_len_sq(vi) == 25, "exp 25, got %d", vec_len_sq(vi));
    } TEST_CASE_END();

    TEST_CASE("vec_len", "calculates euclidean length") {
        vec2f v = vec2f(3, 4);
        TEST_ASSERT(feq(vec_len(v), 5.0f), "exp 5.0, got %f", vec_len(v));

        vec2i vi = vec2i(3, 4);
        TEST_ASSERT(feq(vec_len(vi), 5.0f), "exp 5.0, got %f", vec_len(vi));
    } TEST_CASE_END();

    TEST_CASE("vec_norm", "normalizes vector and guards zero") {
        vec2f v = vec2f(0, 5);
        vec2f n = vec_norm(v);
        TEST_ASSERT(feq(n.x,        0.0f) && feq(n.y, 1.0f), "exp (0.0, 1.0), got (%f, %f)", n.x,        n.y);
        TEST_ASSERT(feq(vec_len(n), 1.0f),                   "exp len 1.0, got %f",          vec_len(n));

        vec2f zero = vec2f(0, 0);
        vec2f n_zero = vec_norm(zero);
        TEST_ASSERT(feq(n_zero.x, 0.0f) && feq(n_zero.y, 0.0f), "zero norm exp (0, 0), got (%f, %f)", n_zero.x, n_zero.y);
    } TEST_CASE_END();

    TEST_CASE("vec_dir", "computes normalized dir vector") {
        vec2f from = vec2f(1, 1);
        vec2f to   = vec2f(4, 5);
        vec2f d    = vec2f_dir(from, to);
        TEST_ASSERT(feq(d.x,        0.6f) && feq(d.y, 0.8f), "exp (0.6, 0.8), got (%f, %f)", d.x,        d.y);
        TEST_ASSERT(feq(vec_len(d), 1.0f),                   "exp unit length, got %f",      vec_len(d));
    } TEST_CASE_END();

    TEST_CASE("mat4f_id", "initializes id matrix") {
        mat4f id = mat4f_id();
        for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) {
                float exp = (c == r) ? 1.0f : 0.0f;
                TEST_ASSERT(feq(id.cols[c][r], exp), "cols[%d][%d] exp %f, got %f", c, r, exp, id.cols[c][r]);
            }
        }
    } TEST_CASE_END();

    TEST_CASE("mat4f_mul", "multiplies matrices correctly") {
        mat4f id = mat4f_id();
        mat4f t  = mat4f_trans((vec3f){ .x = 10.0f, .y = 20.0f, .z = 30.0f });
        mat4f r  = mat4f_mul(id, t);

        for (int i = 0; i < 16; ++i) {
            TEST_ASSERT(feq(r.v[i], t.v[i]), "id * t != t at index %d", i);
        }
    } TEST_CASE_END();

    TEST_CASE("mat4f_trans", "translates 3D position vector") {
        mat4f t   = mat4f_trans(vec3f(5, -3, 2));
        vec4f p   = vec4f(1, 1, 1, 1);
        vec4f res = mat4f_mul_vec4f(t, p);
        TEST_ASSERT(feq(res.x, 6.0f) && feq(res.y, -2.0f) && feq(res.z, 3.0f) && feq(res.w, 1.0f),
                    "exp (6, -2, 3, 1), got (%f, %f, %f, %f)", res.x, res.y, res.z, res.w);
    } TEST_CASE_END();

    TEST_CASE("mat4f_rotz", "rotates vector 90 degrees around Z") {
        mat4f r = mat4f_rotz(HALF_PI);
        vec4f v = vec4f(1, 0, 0, 1);
        vec4f res = mat4f_mul_vec4f(r, v);
        TEST_ASSERT(feq(res.x, 0.0f) && feq(res.y, 1.0f), "exp (0.0, 1.0), got (%f, %f)", res.x, res.y);
    } TEST_CASE_END();

    TEST_CASE("mat4f_trans_2d", "combines pos, rot, and scale in 1 pass") {
        mat4f m = mat4f_trans_2d(vec2f(10, 20), HALF_PI, 2.0f);

        vec4f local = vec4f(1, 0, 0, 1);
        vec4f world = mat4f_mul_vec4f(m, local);
        TEST_ASSERT(feq(world.x, 10.0f) && feq(world.y, 22.0f), "exp (10.0, 22.0), got (%f, %f)", world.x, world.y);
    } TEST_CASE_END();

    TEST_CASE("mat4f_ortho", "maps view bounds to NDC space") {
        mat4f proj = mat4f_ortho(0.0f, 800.0f, 600.0f, 0.0f, -1.0f, 1.0f);

        vec4f center = vec4f(400, 300, 0, 1);
        vec4f ndc = mat4f_mul_vec4f(proj, center);
        TEST_ASSERT(feq(ndc.x, 0.0f) && feq(ndc.y, 0.0f), "center exp (0, 0), got (%f, %f)", ndc.x, ndc.y);
    } TEST_CASE_END();

    TEST_CASE("aabbf_size_center", "calculates size and center accurately") {
        aabb2f box    = aabb_pos_size(vec2f(10, 20), vec2f(30, 40));
        vec2f  size   = aabb_size(box);
        vec2f  center = aabb_center(box);

        TEST_ASSERT(feq(size.x,   30.0f) && feq(size.y, 40.0f),   "size exp (30, 40), got (%f, %f)",   size.x,   size.y);
        TEST_ASSERT(feq(center.x, 25.0f) && feq(center.y, 40.0f), "center exp (25, 40), got (%f, %f)", center.x, center.y);
    } TEST_CASE_END();

    TEST_CASE("aabbf_inter", "detects overlapping vs separated boxes") {
        aabb2f a = aabb_pos_size(vec2f(0, 0),   vec2f(10, 10));
        aabb2f b = aabb_pos_size(vec2f(5, 5),   vec2f(10, 10));
        aabb2f c = aabb_pos_size(vec2f(20, 20), vec2f(5, 5));

        TEST_ASSERT( aabb_inter(a, b), "overlapping boxes exp true");
        TEST_ASSERT(!aabb_inter(a, c), "disjoint boxes exp false");
    } TEST_CASE_END();

    TEST_CASE("aabbf_contains", "checks if point is inside bounding box") {
        aabb2f box     = aabb_pos_size(vec2f(0, 0), vec2f(10, 10));
        vec2f  inside  = vec2f(5, 5);
        vec2f  outside = vec2f(15, 5);

        TEST_ASSERT( aabb_contains(box, inside),  "point inside exp true");
        TEST_ASSERT(!aabb_contains(box, outside), "point outside exp false");
    } TEST_CASE_END();

    TEST_CASE("aabbf_merge", "computes enclosing bounding box") {
        aabb2f a = aabb_pos_size(vec2f(0, 0),   vec2f(10, 10));
        aabb2f b = aabb_pos_size(vec2f(20, 30), vec2f(5, 5));
        aabb2f m = aabb_merge(a, b);

        TEST_ASSERT(feq(m.min_x, 0.0f)  && feq(m.min_y, 0.0f),  "merge min exp (0, 0), got (%f, %f)",   m.min_x, m.min_y);
        TEST_ASSERT(feq(m.max_x, 25.0f) && feq(m.max_y, 35.0f), "merge max exp (25, 35), got (%f, %f)", m.max_x, m.max_y);
    } TEST_CASE_END();

    TEST_CASE("project_to_capture", "projects world point to screen buffer") {
        mat4f proj = mat4f_ortho(0.0f, 320.0f, 180.0f, 0.0f, -1.0f, 1.0f);
        int32_t px = 0, py = 0;

        bool ok = project_to_capture(vec3f(0, 0, 0), proj, 320, 180, &px, &py);
        TEST_ASSERT(ok,                 "projection failed");
        TEST_ASSERT(px == 0 && py == 0, "origin exp (0, 0), got (%d, %d)", px, py);

        ok = project_to_capture(vec3f(160, 90, 0), proj, 320, 180, &px, &py);
        TEST_ASSERT(ok,                    "center projection failed");
        TEST_ASSERT(px == 160 && py == 90, "center exp (160, 90), got (%d, %d)", px, py);
    } TEST_CASE_END();

    TEST_CASE("vec_cross", "computes 3D cross product and 2D signed area") {
        vec3f x = vec3f(1, 0, 0);
        vec3f y = vec3f(0, 1, 0);
        vec3f z = vec_cross(x, y);
        TEST_ASSERT(feq(z.x, 0.0f) && feq(z.y, 0.0f) && feq(z.z, 1.0f), "exp (0, 0, 1), got (%f, %f, %f)", z.x, z.y, z.z);

        vec3i xi = vec3i(2, 0, 0);
        vec3i yi = vec3i(0, 3, 0);
        vec3i zi = vec_cross(xi, yi);
        TEST_ASSERT(zi.x == 0 && zi.y == 0 && zi.z == 6, "exp (0, 0, 6), got (%d, %d, %d)", zi.x, zi.y, zi.z);

        vec2f a = vec2f(1, 0);
        vec2f b = vec2f(0, 1);
        float area = vec_cross(a, b);
        TEST_ASSERT(feq(area, 1.0f), "exp 1.0, got %f", area);

        vec2i ai = vec2i(4, 1);
        vec2i bi = vec2i(2, 3);
        int32_t areai = vec_cross(ai, bi);
        TEST_ASSERT(areai == 10, "exp 10, got %d", areai);
    } TEST_CASE_END();

    TEST_CASE("mat3f_inv", "verifies matrix inversion produces identity") {
        mat3f m = mat3f(
            vec3f(1, 2, 3),
            vec3f(0, 1, 4),
            vec3f(5, 6, 0)
        );
        mat3f inv = mat3f_inv(m);
        mat3f id  = mat3f_mul(m, inv);

        for (int c = 0; c < 3; ++c) {
            for (int r = 0; r < 3; ++r) {
                float exp = (c == r) ? 1.0f : 0.0f;
                TEST_ASSERT(feq(id.cols[c][r], exp), "col %d row %d exp %f, got %f", c, r, exp, id.cols[c][r]);
            }
        }
    } TEST_CASE_END();

    TEST_CASE("quatf_rotate", "rotates vector by 90 deg around Z") {
        vec3f z_axis = vec3f(0, 0, 1);
        quatf q = quatf_axis_angle(z_axis, HALF_PI);

        vec3f v = vec3f(1, 0, 0);
        vec3f rotated = quatf_mul_vec3(q, v);
        TEST_ASSERT(feq(rotated.x, 0.0f) && feq(rotated.y, 1.0f) && feq(rotated.z, 0.0f),
                    "exp (0, 1, 0), got (%f, %f, %f)", rotated.x, rotated.y, rotated.z);

        mat3f m3 = quatf_to_mat3f(q);
        vec3f m_rotated = mat3f_mul_vec(m3, v);
        TEST_ASSERT(feq(m_rotated.x, 0.0f) && feq(m_rotated.y, 1.0f) && feq(m_rotated.z, 0.0f),
                    "matrix exp (0, 1, 0), got (%f, %f, %f)", m_rotated.x, m_rotated.y, m_rotated.z);

        mat4f m4 = quatf_to_mat4f(q);
        vec4f v4 = vec4f(1, 0, 0, 1);
        vec4f m4_rotated = mat4f_mul_vec4f(m4, v4);
        TEST_ASSERT(feq(m4_rotated.x, 0.0f) && feq(m4_rotated.y, 1.0f) && feq(m4_rotated.z, 0.0f) && feq(m4_rotated.w, 1.0f),
                    "mat4 exp (0, 1, 0, 1), got (%f, %f, %f, %f)", m4_rotated.x, m4_rotated.y, m4_rotated.z, m4_rotated.w);
    } TEST_CASE_END();

    TEST_CASE("quatf_slerp", "interpolates orientation at midpoint (45 deg)") {
        vec3f z_axis = vec3f(0, 0, 1);
        quatf q0     = quatf_id();
        quatf q1     = quatf_axis_angle(z_axis, HALF_PI);

        quatf mid = quatf_slerp(q0, q1, 0.5f);
        vec3f v   = vec3f(1, 0, 0);
        vec3f res = quatf_mul_vec3(mid, v);

        float cos_45 = cosf(HALF_PI * 0.5f);
        float sin_45 = sinf(HALF_PI * 0.5f);
        TEST_ASSERT(feq(res.x, cos_45) && feq(res.y, sin_45) && feq(res.z, 0.0f),
                    "slerp midpoint exp (%f, %f, 0), got (%f, %f, %f)", cos_45, sin_45, res.x, res.y, res.z);
    } TEST_CASE_END();

    TEST_CASE("aabb3f_inter", "verifies 3D box overlap and separation on Z axis") {
        aabb3f a = aabb_pos_size(vec3f(0, 0, 0),  vec3f(10, 10, 10));
        aabb3f b = aabb_pos_size(vec3f(5, 5, 5),  vec3f(10, 10, 10));
        aabb3f c = aabb_pos_size(vec3f(5, 5, 20), vec3f(10, 10, 10));

        TEST_ASSERT( aabb_inter(a, b), "overlapping 3D boxes exp true");
        TEST_ASSERT(!aabb_inter(a, c), "Z-separated 3D boxes exp false");
    } TEST_CASE_END();

    TEST_CASE("aabb_inter_radius", "checks circle and sphere inter") {
        aabb2f b2 = aabb2f(vec2f(10, 10), vec2f(20, 20));
        TEST_ASSERT( aabb_inter_radius(b2, vec2f(15, 15), 1.0f), "center inside exp true");
        TEST_ASSERT( aabb_inter_radius(b2, vec2f(5, 15), 5.1f),  "edge overlap exp true");
        TEST_ASSERT(!aabb_inter_radius(b2, vec2f(5, 15), 4.9f),  "edge separation exp false");
        TEST_ASSERT( aabb_inter_radius(b2, vec2f(23, 24), 5.01f), "corner overlap exp true");
        TEST_ASSERT(!aabb_inter_radius(b2, vec2f(23, 24), 4.99f), "corner separation exp false");

        aabb3f b3 = aabb3f(vec3f(10, 10, 10), vec3f(20, 20, 20));
        TEST_ASSERT( aabb_inter_radius(b3, vec3f(22, 23, 26), 7.01f), "3d corner overlap exp true");
        TEST_ASSERT(!aabb_inter_radius(b3, vec3f(22, 23, 26), 6.99f), "3d corner separation exp false");
    } TEST_CASE_END();

    TEST_END();
}
