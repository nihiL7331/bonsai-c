#include "spatial.h"
#include "arena.h"
#include "ecs.h"
#include "gmath.h"
#include "test.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TEST_PERM_SZ    MB(32)
#define TEST_SCRATCH_SZ MB(4)

#define PI_F 3.14159265f

static arena_t s_perm_arena;
static arena_t s_scratch_arena;

static ecs_world_t *reset_world(void) {
    arena_reset(&s_perm_arena);
    arena_reset(&s_scratch_arena);
    return ecs_world_create(&s_perm_arena, NULL, &s_scratch_arena);
}

int main(void) {
    void *perm_buf    = malloc(TEST_PERM_SZ);
    void *scratch_buf = malloc(TEST_SCRATCH_SZ);
    if (!perm_buf || !scratch_buf) return 1;

    arena_init(&s_perm_arena,    perm_buf,    TEST_PERM_SZ);
    arena_init(&s_scratch_arena, scratch_buf, TEST_SCRATCH_SZ);

    TEST_CASE("spatial_init_2d", "initializes 2D grid defaults and custom values") {
        spatial_grid2d_t g_def;
        res_t r1 = spatial_grid2d_init(&g_def, 0.0f, 0);
        TEST_ASSERT(r1 == OK,                                                        "default init failed: %d",   r1);
        TEST_ASSERT(g_def.bucket_cnt == SPATIAL_DEF_BUCKETS,                         "bucket_cnt exp %u, got %u", SPATIAL_DEF_BUCKETS, g_def.bucket_cnt);
        TEST_ASSERT(fabsf(g_def.inv_cell_sz - (1.0f / SPATIAL_DEF_CELL_SZ)) < 1e-6f, "inv_cell_sz mismatch");

        spatial_grid2d_t g_custom;
        res_t r2 = spatial_grid2d_init(&g_custom, 32.0f, 512);
        TEST_ASSERT(r2 == OK,                                             "custom init failed: %d",     r2);
        TEST_ASSERT(g_custom.bucket_cnt == 512,                           "bucket_cnt exp 512, got %u", g_custom.bucket_cnt);
        TEST_ASSERT(fabsf(g_custom.inv_cell_sz - (1.0f / 32.0f)) < 1e-6f, "inv_cell_sz mismatch");
    } TEST_CASE_END();

    TEST_CASE("spatial_init_3d", "initializes 3D grid defaults and custom values") {
        spatial_grid3d_t g_def;
        res_t r1 = spatial_grid3d_init(&g_def, 0.0f, 0);
        TEST_ASSERT(r1 == OK,                                "default 3d init failed: %d", r1);
        TEST_ASSERT(g_def.bucket_cnt == SPATIAL_DEF_BUCKETS, "bucket_cnt exp %u, got %u", SPATIAL_DEF_BUCKETS, g_def.bucket_cnt);

        spatial_grid3d_t g_custom;
        res_t r2 = spatial_grid3d_init(&g_custom, 16.0f, 256);
        TEST_ASSERT(r2 == OK,                                             "custom 3d init failed: %d",  r2);
        TEST_ASSERT(g_custom.bucket_cnt == 256,                           "bucket_cnt exp 256, got %u", g_custom.bucket_cnt);
        TEST_ASSERT(fabsf(g_custom.inv_cell_sz - (1.0f / 16.0f)) < 1e-6f, "inv_cell_sz mismatch");
    } TEST_CASE_END();

    TEST_CASE("spatial_empty", "handles empty input safely in 2D and 3D") {
        arena_reset(&s_scratch_arena);
        spatial_grid2d_t g2d;
        spatial_grid2d_init(&g2d, 32.0f, 256);

        res_t r2d = spatial_grid2d_build(&g2d, NULL, 0, &s_scratch_arena);
        TEST_ASSERT(r2d == WEMPTY,       "empty build exp WEMPTY, got %d", r2d);
        TEST_ASSERT(g2d.entity_cnt == 0, "entity_cnt exp 0, got %u",       g2d.entity_cnt);
        TEST_ASSERT(g2d.entities == NULL, "entities exp NULL");

        entity_handle_t out[4];
        TEST_ASSERT(!spatial_query_point2d(&g2d, vec2f(0, 0), 0, out, 4),                                             "empty point query must return 0");
        TEST_ASSERT(!spatial_query_aabb2d(&g2d, aabb2f(vec2f(-10, -10), vec2f(10, 10)), 0, out, 4, &s_scratch_arena), "empty aabb query must return 0");

        spatial_grid3d_t g3d;
        spatial_grid3d_init(&g3d, 32.0f, 256);
        res_t r3d = spatial_grid3d_build(&g3d, NULL, 0, &s_scratch_arena);
        TEST_ASSERT(r3d == WEMPTY,                                                                                             "empty 3D build exp WEMPTY, got %d", r3d);
        TEST_ASSERT(!spatial_query_point3d(&g3d, vec3f(0, 0, 0), 0, out, 4),                                                   "empty 3D point query must return 0");
        TEST_ASSERT(!spatial_query_aabb3d(&g3d, aabb3f(vec3f(-10, -10, -10), vec3f(10, 10, 10)), 0, out, 4, &s_scratch_arena), "empty 3D aabb query must return 0");
    } TEST_CASE_END();

    TEST_CASE("spatial_point2d", "locates points inside bounds and filters masks") {
        arena_reset(&s_scratch_arena);
        spatial_grid2d_t g;
        spatial_grid2d_init(&g, 64.0f, 256);

        spatial_entity2d_t ents[] = {
            { .ent = (entity_handle_t)1, .bounds = aabb2f(vec2f(0,   0),  vec2f(20,  20)), .mask = 0x01 },
            { .ent = (entity_handle_t)2, .bounds = aabb2f(vec2f(10,  10), vec2f(30,  30)), .mask = 0x02 },
            { .ent = (entity_handle_t)3, .bounds = aabb2f(vec2f(100, 0),  vec2f(120, 20)), .mask = 0x01 }
        };

        res_t r = spatial_grid2d_build(&g, ents, ARR_CNT(ents), &s_scratch_arena);
        TEST_ASSERT(r == OK, "build failed: %d", r);

        entity_handle_t out[4];

        /* point (5, 5): inside ent 1 only */
        uint32_t cnt = spatial_query_point2d(&g, vec2f(5, 5), 0, out, 4);
        TEST_ASSERT(cnt == 1, "point (5, 5) exp 1, got %u", cnt);
        TEST_ASSERT(out[0] == (entity_handle_t)1, "out[0] exp 1, got %u", (uint32_t)out[0]);

        /* point (15, 15): inside ent 1 and ent 2 */
        cnt = spatial_query_point2d(&g, vec2f(15, 15), 0, out, 4);
        TEST_ASSERT(cnt == 2, "point (15, 15) exp 2, got %u", cnt);

        /* mask filtering on (15, 15) */
        cnt = spatial_query_point2d(&g, vec2f(15, 15), 0x02, out, 4);
        TEST_ASSERT(cnt == 1, "point (15, 15) mask 0x02 exp 1, got %u", cnt);
        TEST_ASSERT(out[0] == (entity_handle_t)2, "out[0] exp 2, got %u", (uint32_t)out[0]);

        /* point miss (50, 50) */
        cnt = spatial_query_point2d(&g, vec2f(50, 50), 0, out, 4);
        TEST_ASSERT(cnt == 0, "point (50, 50) exp 0, got %u", cnt);

        /* max_out capping */
        cnt = spatial_query_point2d(&g, vec2f(15, 15), 0, out, 1);
        TEST_ASSERT(cnt == 1, "max_out=1 exp 1, got %u", cnt);
    } TEST_CASE_END();

    TEST_CASE("spatial_aabb2d", "queries intersecting boxes and deduplicates") {
        arena_reset(&s_scratch_arena);
        spatial_grid2d_t g;
        spatial_grid2d_init(&g, 32.0f, 256);

        /* entity spanning across cell boundaries (0,0)-(48,48) touches cells (0,0), (1,0), (0,1), (1,1) */
        spatial_entity2d_t ents[] = {
            { .ent = (entity_handle_t)10, .bounds = aabb2f(vec2f(10, 10), vec2f(50, 50)), .mask = 0x01 },
            { .ent = (entity_handle_t)20, .bounds = aabb2f(vec2f(70, 70), vec2f(80, 80)), .mask = 0x02 }
        };

        spatial_grid2d_build(&g, ents, ARR_CNT(ents), &s_scratch_arena);

        entity_handle_t out[4];

        /* query covering multiple cells of ent 10 */
        uint32_t cnt = spatial_query_aabb2d(&g, aabb2f(vec2f(0, 0), vec2f(60, 60)), 0, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 1,                      "exp 1 (deduplicated), got %u", cnt);
        TEST_ASSERT(out[0] == (entity_handle_t)10, "exp ent 10, got %u",           (uint32_t)out[0]);

        /* query covering both ents */
        cnt = spatial_query_aabb2d(&g, aabb2f(vec2f(0, 0), vec2f(100, 100)), 0, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 2, "exp 2, got %u", cnt);

        /* mask filtering */
        cnt = spatial_query_aabb2d(&g, aabb2f(vec2f(0, 0), vec2f(100, 100)), 0x02, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 1,                      "exp 1 with mask 0x02, got %u", cnt);
        TEST_ASSERT(out[0] == (entity_handle_t)20, "exp ent 20, got %u",           (uint32_t)out[0]);

        /* query miss */
        cnt = spatial_query_aabb2d(&g, aabb2f(vec2f(200, 200), vec2f(210, 210)), 0, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 0, "miss exp 0, got %u", cnt);
    } TEST_CASE_END();

    TEST_CASE("spatial_sbo", "switches between stack buffer and scratch arena") {
        arena_reset(&s_scratch_arena);
        spatial_grid2d_t g;
        spatial_grid2d_init(&g, 64.0f, 1024);

        /* <=2048 ents uses stack buffer without touching scratch */
        spatial_entity2d_t e_small = { .ent = (entity_handle_t)1, .bounds = aabb2f(vec2f(0, 0), vec2f(10, 10)), .mask = 1 };
        spatial_grid2d_build(&g, &e_small, 1, &s_scratch_arena);

        size_t off_before = s_scratch_arena.off;
        entity_handle_t out[4];
        /* pass scratch=NULL to prove it doesnt touch scratch when <=2048 */
        uint32_t cnt = spatial_query_aabb2d(&g, aabb2f(vec2f(-5, -5), vec2f(15, 15)), 0, out, 4, NULL);
        TEST_ASSERT(cnt == 1,                          "sbo stack query exp 1, got %u", cnt);
        TEST_ASSERT(s_scratch_arena.off == off_before, "sbo stack query must not allocate from scratch");

        /* >2048 entities triggers scratch-alloc and arena_temp unwinds */
        const uint32_t large_cnt = 2100;
        spatial_entity2d_t *large_ents = arena_alloc(&s_scratch_arena, large_cnt * sizeof(spatial_entity2d_t));
        TEST_ASSERT(large_ents, "large_ents alloc failed");
        for (uint32_t i = 0; i < large_cnt; ++i) {
            large_ents[i] = (spatial_entity2d_t){
                .ent    = (entity_handle_t)(i + 1),
                .bounds = aabb2f(vec2f(i * 100, 0), vec2f(i * 100 + 10, 10)),
                .mask   = 1
            };
        }

        spatial_grid2d_build(&g, large_ents, large_cnt, &s_scratch_arena);
        size_t off_after_build = s_scratch_arena.off;

        cnt = spatial_query_aabb2d(&g, aabb2f(vec2f(-5, -5), vec2f(15, 15)), 0, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 1,                               "large query exp 1, got %u",                  cnt);
        TEST_ASSERT(s_scratch_arena.off == off_after_build, "scratch offset exp rewound to %zu, got %zu", off_after_build, s_scratch_arena.off);
    } TEST_CASE_END();

    TEST_CASE("spatial_ecs2d", "builds 2D grid from ECS transforms and colliders") {
        reset_world();
        spatial_grid2d_t g;
        spatial_grid2d_init(&g, 64.0f, 256);

        entity_handle_t e1 = ecs_spawn();
        trans2d    t1 = { .x      = 100.0f, .y      = 100.0f, .scale = 1.0f };
        collider2d c1 = { .half_x = 10.0f,  .half_y = 10.0f,  .mask  = 0x01 };
        ecs_add(e1, trans2d,    &t1);
        ecs_add(e1, collider2d, &c1);

        entity_handle_t e2 = ecs_spawn();
        trans2d    t2 = { .x      = 200.0f, .y      = 200.0f, .rot  = PI_F * 0.5f, .scale = 1.0f };
        collider2d c2 = { .half_x = 20.0f,  .half_y = 10.0f,  .mask = 0x02                       };
        ecs_add(e2, trans2d,    &t2);
        ecs_add(e2, collider2d, &c2);

        entity_handle_t e3 = ecs_spawn();
        trans2d    t3 = { .x      = 300.0f, .y      = 300.0f, .scale = -1.0f };
        collider2d c3 = { .half_x = 15.0f,  .half_y = 15.0f,  .mask  = 0x01  };
        ecs_add(e3, trans2d,    &t3);
        ecs_add(e3, collider2d, &c3);

        ecs_sync();

        res_t r = spatial_grid2d_build_from_ecs(&g, NULL, &s_scratch_arena);
        TEST_ASSERT(r == OK,           "ecs build failed: %d",   r);
        TEST_ASSERT(g.entity_cnt == 3, "exp 3 entities, got %u", g.entity_cnt);

        entity_handle_t out[4];

        /* query e1 */
        uint32_t cnt = spatial_query_point2d(&g, vec2f(105, 105), 0, out, 4);
        TEST_ASSERT(cnt == 1 && out[0] == e1, "query e1 failed");

        /* query e2 rotated (200, 200) +/- (10, 20)           */
        /* (200, 215) should be inside (200 + 15 < 200 + 20)  */
        /* (215, 200) should be outside (200 + 15 > 200 + 10) */
        cnt = spatial_query_point2d(&g, vec2f(200, 215), 0, out, 4);
        TEST_ASSERT(cnt == 1 && out[0] == e2, "rotated e2 point (200, 215) exp inside");

        cnt = spatial_query_point2d(&g, vec2f(215, 200), 0, out, 4);
        TEST_ASSERT(cnt == 0, "rotated e2 point (215, 200) exp outside");

        /* query e3 <0 scale */
        cnt = spatial_query_point2d(&g, vec2f(300, 300), 0, out, 4);
        TEST_ASSERT(cnt == 1 && out[0] == e3, "negative scale e3 query failed");
    } TEST_CASE_END();

    TEST_CASE("spatial_point3d", "queries 3D points inside bounds with masks") {
        arena_reset(&s_scratch_arena);
        spatial_grid3d_t g;
        spatial_grid3d_init(&g, 32.0f, 256);

        spatial_entity3d_t ents[] = {
            { .ent = (entity_handle_t)100, .bounds = aabb3f(vec3f(0,   0,  0),  vec3f(20,  20, 20)),  .mask = 0x01 },
            { .ent = (entity_handle_t)200, .bounds = aabb3f(vec3f(15,  15, 15), vec3f(40,  40, 40)),  .mask = 0x02 },
            { .ent = (entity_handle_t)300, .bounds = aabb3f(vec3f(100, 0,  0),  vec3f(120, 20, 20)),  .mask = 0x01 }
        };

        res_t r = spatial_grid3d_build(&g, ents, ARR_CNT(ents), &s_scratch_arena);
        TEST_ASSERT(r == OK, "3d build failed: %d", r);

        entity_handle_t out[4];

        /* point query at (10, 10, 10): ent 100 only */
        uint32_t cnt = spatial_query_point3d(&g, vec3f(10, 10, 10), 0, out, 4);
        TEST_ASSERT(cnt == 1 && out[0] == (entity_handle_t)100, "3D point (10,10,10) exp 100");

        /* point query at (18, 18, 18): ent 100 and ent 200 */
        cnt = spatial_query_point3d(&g, vec3f(18, 18, 18), 0, out, 4);
        TEST_ASSERT(cnt == 2, "3D point (18,18,18) exp 2, got %u", cnt);

        /* mask query */
        cnt = spatial_query_point3d(&g, vec3f(18, 18, 18), 0x02, out, 4);
        TEST_ASSERT(cnt == 1 && out[0] == (entity_handle_t)200, "3D mask query exp 200");

        /* miss */
        cnt = spatial_query_point3d(&g, vec3f(50, 50, 50), 0, out, 4);
        TEST_ASSERT(cnt == 0, "3D point (50,50,50) exp 0");
    } TEST_CASE_END();

    TEST_CASE("spatial_aabb3d", "queries 3D boxes and deduplicates multi-cells") {
        arena_reset(&s_scratch_arena);
        spatial_grid3d_t g;
        spatial_grid3d_init(&g, 32.0f, 256);

        spatial_entity3d_t ents[] = {
            { .ent = (entity_handle_t)100, .bounds = aabb3f(vec3f(0,  0,  0),  vec3f(20, 20, 20)), .mask = 0x01 },
            { .ent = (entity_handle_t)200, .bounds = aabb3f(vec3f(15, 15, 15), vec3f(40, 40, 40)), .mask = 0x02 }
        };

        spatial_grid3d_build(&g, ents, ARR_CNT(ents), &s_scratch_arena);

        entity_handle_t out[4];

        /* aabb query covering both ents */
        uint32_t cnt = spatial_query_aabb3d(&g, aabb3f(vec3f(5, 5, 5), vec3f(25, 25, 25)), 0, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 2, "3D aabb query exp 2, got %u", cnt);

        /* mask query */
        cnt = spatial_query_aabb3d(&g, aabb3f(vec3f(5, 5, 5), vec3f(25, 25, 25)), 0x02, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 1 && out[0] == (entity_handle_t)200, "3D aabb mask exp 200");
    } TEST_CASE_END();

    TEST_CASE("spatial_radius2d", "queries circle area with exact corner dist") {
        arena_reset(&s_scratch_arena);
        spatial_grid2d_t g;
        spatial_grid2d_init(&g, 32.0f, 256);

        spatial_entity2d_t ents[] = {
            { .ent = (entity_handle_t)1, .bounds = aabb2f(vec2f(10, 10), vec2f(20, 20)), .mask = 0x01 },
            { .ent = (entity_handle_t)2, .bounds = aabb2f(vec2f(50, 50), vec2f(60, 60)), .mask = 0x02 },
        };

        spatial_grid2d_build(&g, ents, ARR_CNT(ents), &s_scratch_arena);
        entity_handle_t out[4];

        uint32_t cnt = spatial_query_radius(&g, vec2f(15, 15), 1.0f, 0, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 1 && out[0] == (entity_handle_t)1, "radius inside exp 1");

        cnt = spatial_query_radius(&g, vec2f(23, 24), 4.9f, 0, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 0, "radius 4.9 outside corner exp 0");

        cnt = spatial_query_radius(&g, vec2f(23, 24), 5.1f, 0, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 1 && out[0] == (entity_handle_t)1, "radius 5.1 inside corner exp 1");

        cnt = spatial_query_radius2d(&g, vec2f(15, 15), 10.0f, 0x02, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 0, "radius mask mismatch exp 0");
    } TEST_CASE_END();

    TEST_CASE("spatial_radius3d", "queries sphere volume with corner distance") {
        arena_reset(&s_scratch_arena);
        spatial_grid3d_t g;
        spatial_grid3d_init(&g, 32.0f, 256);

        spatial_entity3d_t ents[] = {
            { .ent = (entity_handle_t)10, .bounds = aabb3f(vec3f(10, 10, 10), vec3f(20, 20, 20)), .mask = 0x01 },
        };

        spatial_grid3d_build(&g, ents, ARR_CNT(ents), &s_scratch_arena);
        entity_handle_t out[4];

        uint32_t cnt = spatial_query_radius(&g, vec3f(22, 23, 26), 6.9f, 0, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 0, "radius 6.9 outside 3d corner exp 0");

        cnt = spatial_query_radius3d(&g, vec3f(22, 23, 26), 7.1f, 0, out, 4, &s_scratch_arena);
        TEST_ASSERT(cnt == 1 && out[0] == (entity_handle_t)10, "radius 7.1 inside 3d corner exp 1");
    } TEST_CASE_END();

    TEST_CASE("spatial_ecs3d", "builds 3D grid from ECS transforms and colliders") {
        reset_world();
        spatial_grid3d_t g;
        spatial_grid3d_init(&g, 64.0f, 256);

        entity_handle_t e1 = ecs_spawn();
        trans3d    t1 = { .x      = 10.0f, .y      = 20.0f, .z      = 30.0f, .qw   = 1.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f };
        collider3d c1 = { .half_x = 5.0f,  .half_y = 5.0f,  .half_z = 5.0f,  .mask = 0x01 };
        ecs_add(e1, trans3d,    &t1);
        ecs_add(e1, collider3d, &c1);

        float half_ang = PI_F * 0.25f;
        entity_handle_t e2 = ecs_spawn();
        trans3d t2 = {
            .x       = 100.0f, .y       = 100.0f, .z       = 100.0f,
            .qx      = 0.0f,   .qy      = 0.0f,   .qz      = sinf(half_ang), .qw = cosf(half_ang),
            .scale_x = 1.0f,   .scale_y = 1.0f,   .scale_z = 1.0f
        };
        collider3d c2 = { .half_x = 20.0f, .half_y = 10.0f, .half_z = 5.0f, .mask = 0x02 };
        ecs_add(e2, trans3d,    &t2);
        ecs_add(e2, collider3d, &c2);

        ecs_sync();

        res_t r = spatial_grid3d_build_from_ecs(&g, NULL, &s_scratch_arena);
        TEST_ASSERT(r == OK,           "ecs 3d build failed: %d", r);
        TEST_ASSERT(g.entity_cnt == 2, "exp 2 entities, got %u",  g.entity_cnt);

        entity_handle_t out[4];

        /* query e1 */
        uint32_t cnt = spatial_query_point3d(&g, vec3f(12, 22, 32), 0, out, 4);
        TEST_ASSERT(cnt == 1 && out[0] == e1, "query e1 in 3D failed");

        /* query e2 rotated around Z: half_x=20, half_y=10 rotated 90 deg gives extents (10, 20, 5) */
        cnt = spatial_query_point3d(&g, vec3f(100, 115, 100), 0, out, 4);
        TEST_ASSERT(cnt == 1 && out[0] == e2, "rotated e2 point (100, 115, 100) exp inside");

        cnt = spatial_query_point3d(&g, vec3f(115, 100, 100), 0, out, 4);
        TEST_ASSERT(cnt == 0, "rotated e2 point (115, 100, 100) exp outside");
    } TEST_CASE_END();

    TEST_CASE("spatial_generics", "dispatches correctly via generic macros") {
        arena_reset(&s_scratch_arena);
        spatial_grid2d_t g2;
        spatial_grid3d_t g3;

        TEST_ASSERT(spatial_grid_init(&g2, 32.0f, 128) == OK, "generic init 2d failed");
        TEST_ASSERT(spatial_grid_init(&g3, 32.0f, 128) == OK, "generic init 3d failed");

        spatial_entity2d_t e2 = { .ent = (entity_handle_t)1, .bounds = aabb2f(vec2f(0, 0),    vec2f(10, 10)),     .mask = 1 };
        spatial_entity3d_t e3 = { .ent = (entity_handle_t)2, .bounds = aabb3f(vec3f(0, 0, 0), vec3f(10, 10, 10)), .mask = 1 };

        TEST_ASSERT(spatial_grid_build(&g2, &e2, 1, &s_scratch_arena) == OK, "generic build 2d failed");
        TEST_ASSERT(spatial_grid_build(&g3, &e3, 1, &s_scratch_arena) == OK, "generic build 3d failed");

        entity_handle_t out[4];

        TEST_ASSERT(spatial_query_point(&g2, vec2f(5, 5),    0, out, 4) == 1, "generic point 2d failed");
        TEST_ASSERT(spatial_query_point(&g3, vec3f(5, 5, 5), 0, out, 4) == 1, "generic point 3d failed");

        TEST_ASSERT(spatial_query_aabb(&g2, aabb2f(vec2f(0, 0),    vec2f(5, 5)),    0, out, 4, &s_scratch_arena) == 1, "generic aabb 2d failed");
        TEST_ASSERT(spatial_query_aabb(&g3, aabb3f(vec3f(0, 0, 0), vec3f(5, 5, 5)), 0, out, 4, &s_scratch_arena) == 1, "generic aabb 3d failed");
    } TEST_CASE_END();

    free(perm_buf);
    free(scratch_buf);
    TEST_END();
}
