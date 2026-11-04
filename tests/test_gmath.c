#include "gmath.h"
#include "test.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define HALF_PI (3.14159265f * 0.5f)
#define EPS 0.0001f

static inline bool feq(float a, float b) {
    return fabsf(a - b) <= EPS;
}

int main(void) {
    TEST_CASE("vec_add", "computes component-wise sum") {
        vec2f a2 = { .x = 1.0f, .y = 2.0f };
        vec2f b2 = { .x = 3.0f, .y = 4.0f };
        vec2f r2 = vec_add(a2, b2);
        TEST_ASSERT(feq(r2.x, 4.0f) && feq(r2.y, 6.0f), "exp (4.0, 6.0), got (%f, %f)", r2.x, r2.y);

        vec3i a3 = { .x = 1, .y = -2, .z = 5 };
        vec3i b3 = { .x = 4, .y =  2, .z = 1 };
        vec3i r3 = vec_add(a3, b3);
        TEST_ASSERT(r3.x == 5 && r3.y == 0 && r3.z == 6, "exp (5, 0, 6), got (%d, %d, %d)", r3.x, r3.y, r3.z);
    } TEST_CASE_END();

    TEST_CASE("vec_sub", "computes component-wise difference") {
        vec2f a = { .x = 5.0f, .y = 8.0f };
        vec2f b = { .x = 2.0f, .y = 3.0f };
        vec2f r = vec_sub(a, b);
        TEST_ASSERT(feq(r.x, 3.0f) && feq(r.y, 5.0f), "exp (3.0, 5.0), got (%f, %f)", r.x, r.y);
    } TEST_CASE_END();

    TEST_CASE("vec_scale", "scales vector components by scalar") {
        vec2f v = { .x = 2.0f, .y = -3.0f };
        vec2f r = vec_scale(v, 2.5f);
        TEST_ASSERT(feq(r.x, 5.0f) && feq(r.y, -7.5f), "exp (5.0, -7.5), got (%f, %f)", r.x, r.y);

        vec2i vi = { .x = 3, .y = 4 };
        vec2i ri = vec_scale(vi, 3);
        TEST_ASSERT(ri.x == 9 && ri.y == 12, "exp (9, 12), got (%d, %d)", ri.x, ri.y);
    } TEST_CASE_END();

    TEST_CASE("vec_dot", "calculates inner scalar product") {
        vec2f a2 = { .x = 1.0f, .y = 2.0f };
        vec2f b2 = { .x = 3.0f, .y = 4.0f };
        float d2 = vec_dot(a2, b2);
        TEST_ASSERT(feq(d2, 11.0f), "exp 11.0, got %f", d2);

        vec3f a3 = { .x = 1.0f, .y = 0.0f, .z = 0.0f };
        vec3f b3 = { .x = 0.0f, .y = 1.0f, .z = 0.0f };
        float d3 = vec_dot(a3, b3);
        TEST_ASSERT(feq(d3, 0.0f), "orthogonal vectors exp dot 0.0, got %f", d3);
    } TEST_CASE_END();

    TEST_CASE("vec_len_sq", "calculates squared magnitude") {
        vec2f v = { .x = 3.0f, .y = 4.0f };
        TEST_ASSERT(feq(vec_len_sq(v), 25.0f), "exp 25.0, got %f", vec_len_sq(v));

        vec2i vi = { .x = 3, .y = 4 };
        TEST_ASSERT(vec_len_sq(vi) == 25, "exp 25, got %d", vec_len_sq(vi));
    } TEST_CASE_END();

    TEST_CASE("vec_len", "calculates euclidean length") {
        vec2f v = { .x = 3.0f, .y = 4.0f };
        TEST_ASSERT(feq(vec_len(v), 5.0f), "exp 5.0, got %f", vec_len(v));

        vec2i vi = { .x = 3, .y = 4 };
        TEST_ASSERT(feq(vec_len(vi), 5.0f), "exp 5.0, got %f", vec_len(vi));
    } TEST_CASE_END();

    TEST_CASE("vec_norm", "normalizes vector and guards zero") {
        vec2f v = { .x = 0.0f, .y = 5.0f };
        vec2f n = vec_norm(v);
        TEST_ASSERT(feq(n.x,        0.0f) && feq(n.y, 1.0f), "exp (0.0, 1.0), got (%f, %f)", n.x,        n.y);
        TEST_ASSERT(feq(vec_len(n), 1.0f),                   "exp len 1.0, got %f",          vec_len(n));

        vec2f zero   = { .x = 0.0f, .y = 0.0f };
        vec2f n_zero = vec_norm(zero);
        TEST_ASSERT(feq(n_zero.x, 0.0f) && feq(n_zero.y, 0.0f), "zero norm exp (0, 0), got (%f, %f)", n_zero.x, n_zero.y);
    } TEST_CASE_END();

    TEST_CASE("vec_dir", "computes normalized dir vector") {
        vec2f from = { .x = 1.0f, .y = 1.0f };
        vec2f to   = { .x = 4.0f, .y = 5.0f };
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
        mat4f t   = mat4f_trans((vec3f){ .x = 5.0f, .y = -3.0f, .z = 2.0f });
        vec4f p   = { .x = 1.0f, .y = 1.0f, .z = 1.0f, .w = 1.0f };
        vec4f res = mat4f_mul_vec4f(t, p);
        TEST_ASSERT(feq(res.x, 6.0f) && feq(res.y, -2.0f) && feq(res.z, 3.0f) && feq(res.w, 1.0f),
                    "exp (6, -2, 3, 1), got (%f, %f, %f, %f)", res.x, res.y, res.z, res.w);
    } TEST_CASE_END();

    TEST_CASE("mat4f_rotz", "rotates vector 90 degrees around Z") {
        float half_pi = 3.14159265f * 0.5f;
        mat4f r = mat4f_rotz(half_pi);
        vec4f v = { .x = 1.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f };
        vec4f res = mat4f_mul_vec4f(r, v);
        TEST_ASSERT(feq(res.x, 0.0f) && feq(res.y, 1.0f), "exp (0.0, 1.0), got (%f, %f)", res.x, res.y);
    } TEST_CASE_END();

    TEST_CASE("mat4f_trans_2d", "combines pos, rot, and scale in 1 pass") {
        mat4f m = mat4f_trans_2d((vec2f){ .x = 10.0f, .y = 20.0f }, HALF_PI, 2.0f);

        vec4f local = { .x = 1.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f };
        vec4f world = mat4f_mul_vec4f(m, local);
        TEST_ASSERT(feq(world.x, 10.0f) && feq(world.y, 22.0f), "exp (10.0, 22.0), got (%f, %f)", world.x, world.y);
    } TEST_CASE_END();

    TEST_CASE("mat4f_ortho", "maps view bounds to NDC space") {
        mat4f proj = mat4f_ortho(0.0f, 800.0f, 600.0f, 0.0f, -1.0f, 1.0f);

        vec4f center = { .x = 400.0f, .y = 300.0f, .z = 0.0f, .w = 1.0f };
        vec4f ndc = mat4f_mul_vec4f(proj, center);
        TEST_ASSERT(feq(ndc.x, 0.0f) && feq(ndc.y, 0.0f), "center exp (0, 0), got (%f, %f)", ndc.x, ndc.y);
    } TEST_CASE_END();

    TEST_CASE("aabbf_size_center", "calculates size and center accurately") {
        aabbf box = aabbf_pos_size((vec2f){ .x = 10.0f, .y = 20.0f }, (vec2f){ .x = 30.0f, .y = 40.0f });
        vec2f size = aabbf_size(box);
        vec2f center = aabbf_center(box);

        TEST_ASSERT(feq(size.x,   30.0f) && feq(size.y, 40.0f),   "size exp (30, 40), got (%f, %f)",   size.x,   size.y);
        TEST_ASSERT(feq(center.x, 25.0f) && feq(center.y, 40.0f), "center exp (25, 40), got (%f, %f)", center.x, center.y);
    } TEST_CASE_END();

    TEST_CASE("aabbf_inter", "detects overlapping vs separated boxes") {
        aabbf a = aabbf_pos_size((vec2f){ .x = 0.0f,  .y = 0.0f  }, (vec2f){ .x = 10.0f, .y = 10.0f });
        aabbf b = aabbf_pos_size((vec2f){ .x = 5.0f,  .y = 5.0f  }, (vec2f){ .x = 10.0f, .y = 10.0f });
        aabbf c = aabbf_pos_size((vec2f){ .x = 20.0f, .y = 20.0f }, (vec2f){ .x = 5.0f,  .y = 5.0f  });

        TEST_ASSERT( aabbf_inter(a, b), "overlapping boxes exp true");
        TEST_ASSERT(!aabbf_inter(a, c), "disjoint boxes exp false");
    } TEST_CASE_END();

    TEST_CASE("aabbf_contains", "checks if point is inside bounding box") {
        aabbf box = aabbf_pos_size((vec2f){ .x = 0.0f, .y = 0.0f }, (vec2f){ .x = 10.0f, .y = 10.0f });
        vec2f inside = { .x = 5.0f, .y = 5.0f };
        vec2f outside = { .x = 15.0f, .y = 5.0f };

        TEST_ASSERT( aabbf_contains(box, inside),  "point inside exp true");
        TEST_ASSERT(!aabbf_contains(box, outside), "point outside exp false");
    } TEST_CASE_END();

    TEST_CASE("aabbf_merge", "computes enclosing bounding box") {
        aabbf a = aabbf_pos_size((vec2f){ .x = 0.0f, .y = 0.0f   }, (vec2f){ .x = 10.0f, .y = 10.0f });
        aabbf b = aabbf_pos_size((vec2f){ .x = 20.0f, .y = 30.0f }, (vec2f){ .x = 5.0f,  .y = 5.0f  });
        aabbf m = aabbf_merge(a, b);

        TEST_ASSERT(feq(m.min_x, 0.0f)  && feq(m.min_y, 0.0f),  "merge min exp (0, 0), got (%f, %f)",   m.min_x, m.min_y);
        TEST_ASSERT(feq(m.max_x, 25.0f) && feq(m.max_y, 35.0f), "merge max exp (25, 35), got (%f, %f)", m.max_x, m.max_y);
    } TEST_CASE_END();

    TEST_CASE("project_to_capture", "projects world point to screen buffer") {
        mat4f proj = mat4f_ortho(0.0f, 320.0f, 180.0f, 0.0f, -1.0f, 1.0f);
        int32_t px = 0, py = 0;

        bool ok = project_to_capture((vec3f){ .x = 0.0f, .y = 0.0f, .z = 0.0f }, proj, 320, 180, &px, &py);
        TEST_ASSERT(ok,                 "projection failed");
        TEST_ASSERT(px == 0 && py == 0, "origin exp (0, 0), got (%d, %d)", px, py);

        ok = project_to_capture((vec3f){ .x = 160.0f, .y = 90.0f, .z = 0.0f }, proj, 320, 180, &px, &py);
        TEST_ASSERT(ok,                    "center projection failed");
        TEST_ASSERT(px == 160 && py == 90, "center exp (160, 90), got (%d, %d)", px, py);
    } TEST_CASE_END();

    TEST_END();
}
