#include "render.h"
#include "arena.h"
#include "assets.h"
#include "ecs.h"
#include "gmath.h"
#include "mesh.h"
#include "test.h"

#include <math.h>
#include <string.h>

#define EPS 0.001f

static inline bool feq(float a, float b) {
    return fabsf(a - b) <= EPS;
}

static inline bool vec3f_eq(vec3f a, vec3f b) {
    return feq(a.x, b.x) && feq(a.y, b.y) && feq(a.z, b.z);
}

static arena_t s_perm_arena;
static arena_t s_scratch_arena;

static ecs_world_t *reset_world(void) {
    arena_reset(&s_perm_arena);
    arena_reset(&s_scratch_arena);
    return ecs_world_create(&s_perm_arena, NULL, &s_scratch_arena);
}

int main(void) {
    if (arena_init_vm(&s_perm_arena,    MB(64)) != OK) return 1;
    if (arena_init_vm(&s_scratch_arena, MB(16)) != OK) return 1;

    TEST_CASE("render_cam3d_and_vp", "validates lookat, perspective, and z01 remapping") {
        render_cam3d_t cam = render_cam3d_def();
        TEST_ASSERT(vec3f_eq(cam.pos,    vec3f(0, 0, 0)), "default eye at origin");
        TEST_ASSERT(vec3f_eq(cam.target, vec3f(0, 0, 0)), "default target at origin");
        TEST_ASSERT(vec3f_eq(cam.up,     vec3f(0, 1, 0)), "default up along +Y");

        /* default lookat should produce identity matrix */
        mat4f v_def = render_mat4f_lookat(cam.pos, cam.target, cam.up);
        mat4f id    = mat4f_id();
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                TEST_ASSERT(feq(v_def.cols[c][r], id.cols[c][r]), "v_def[%d][%d] must be identity", c, r);

        /* degenerate lookat (eye == target) falls back to stable basis without NaNs */
        mat4f v_degen = render_mat4f_lookat(vec3f(1, 1, 1), vec3f(1, 1, 1), vec3f(0, 1, 0));
        TEST_ASSERT(!isnan(v_degen.cols[0][0]) && !isnan(v_degen.cols[3][3]), "degenerate lookat must not produce NaNs");

        /* lookat from +Z looking back to origin: forward is -Z in view space */
        mat4f v_z = render_mat4f_lookat(vec3f(0, 0, 10), vec3f(0, 0, 0), vec3f(0, 1, 0));
        vec4f p_world = vec4f(0, 0, 0, 1);
        vec4f p_view  = mat4f_mul_vec(v_z, p_world);
        TEST_ASSERT(feq(p_view.x, 0.0f) && feq(p_view.y, 0.0f) && feq(p_view.z, -10.0f),
                    "point (0,0,0) in view space is (0,0,-10)");

        /* view-projection matrix */
        vec2i vp = vec2i(1920, 1080);
        mat4f vp_mat = render_cam3d_vp(&cam, vp);
        TEST_ASSERT(!isnan(vp_mat.cols[0][0]), "vp matrix valid");

        /* z01 remapping: OpenGL [-1, 1] clip depth remapped to [0, 1] */
        mat4f proj_gl  = mat4f_persp(GMATH_PI / 3.0f, 16.0f / 9.0f, 0.1f, 100.0f);
        mat4f proj_z01 = render_proj_to_z01(proj_gl);

        /* near plane point (z = -0.1f) in clip space */
        vec4f clip_gl_near  = mat4f_mul_vec(proj_gl,  vec4f(0, 0, -0.1f, 1));
        vec4f clip_z01_near = mat4f_mul_vec(proj_z01, vec4f(0, 0, -0.1f, 1));
        TEST_ASSERT(feq(clip_gl_near.z  / clip_gl_near.w,  -1.0f), "OpenGL near plane NDC z == -1");
        TEST_ASSERT(feq(clip_z01_near.z / clip_z01_near.w,  0.0f), "D3D/Metal near plane NDC z == 0");

        /* far plane point (z = -100.0f) in clip space */
        vec4f clip_gl_far  = mat4f_mul_vec(proj_gl,  vec4f(0, 0, -100.0f, 1));
        vec4f clip_z01_far = mat4f_mul_vec(proj_z01, vec4f(0, 0, -100.0f, 1));
        TEST_ASSERT(feq(clip_gl_far.z  / clip_gl_far.w,  1.0f), "OpenGL far plane NDC z == 1");
        TEST_ASSERT(feq(clip_z01_far.z / clip_z01_far.w, 1.0f), "D3D/Metal far plane NDC z == 1");
    } TEST_CASE_END();

    TEST_CASE("render_cam2d_quad", "validates 2D ortho VP and quad vertex expansion") {
        vec2i vp = vec2i(320, 180);

        /* top-left uncentered ortho: (0, 0) -> (-1, 1), (320, 180) -> (1, -1) */
        render_cam2d_t c_tl = render_cam2d_def();
        mat4f vp_tl = render_cam2d_vp(&c_tl, vp);
        vec4f tl_ndc = mat4f_mul_vec(vp_tl, vec4f(0, 0, 0, 1));
        vec4f br_ndc = mat4f_mul_vec(vp_tl, vec4f(320, 180, 0, 1));
        TEST_ASSERT(feq(tl_ndc.x, -1.0f) && feq(tl_ndc.y,  1.0f), "top-left maps to (-1, 1)");
        TEST_ASSERT(feq(br_ndc.x,  1.0f) && feq(br_ndc.y, -1.0f), "bottom-right maps to (1, -1)");

        /* centered ortho: (0, 0) -> (0, 0) */
        render_cam2d_t c_center = (render_cam2d_t){ .pos = vec2f_splat(0), .zoom = 1.0f, .centered = true };
        mat4f vp_center = render_cam2d_vp(&c_center, vp);
        vec4f center_ndc = mat4f_mul_vec(vp_center, vec4f(0, 0, 0, 1));
        TEST_ASSERT(feq(center_ndc.x, 0.0f) && feq(center_ndc.y, 0.0f), "centered camera maps origin to NDC (0, 0)");

        /* render_sprite_quad_verts with top-left pivot <0, 0> */
        render_sprite_inst_t inst_tl = {
            .pos   = vec2f(50, 100),
            .size  = vec2f(32, 16),
            .pivot = vec2f(0, 0),
            .rot   = 0.0f,
        };
        vec2f corners[4];
        render_sprite_quad_verts(&inst_tl, corners);
        TEST_ASSERT(feq(corners[0].x, 50.0f) && feq(corners[0].y, 100.0f), "c0 top-left");
        TEST_ASSERT(feq(corners[1].x, 82.0f) && feq(corners[1].y, 100.0f), "c1 top-right");
        TEST_ASSERT(feq(corners[2].x, 82.0f) && feq(corners[2].y, 116.0f), "c2 bottom-right");
        TEST_ASSERT(feq(corners[3].x, 50.0f) && feq(corners[3].y, 116.0f), "c3 bottom-left");

        /* render_sprite_quad_verts with center pivot <0.5, 0.5> */
        render_sprite_inst_t inst_c = {
            .pos   = vec2f(100, 100),
            .size  = vec2f(20, 20),
            .pivot = vec2f(0.5f, 0.5f),
            .rot   = 0.0f,
        };
        render_sprite_quad_verts(&inst_c, corners);
        TEST_ASSERT(feq(corners[0].x,  90.0f) && feq(corners[0].y,  90.0f), "center pivot c0 top-left");
        TEST_ASSERT(feq(corners[2].x, 110.0f) && feq(corners[2].y, 110.0f), "center pivot c2 bottom-right");
    } TEST_CASE_END();

    TEST_CASE("render_trans3d_mat4", "builds model matrices and handles zero quaternions") {
        /* zero quaternion orientation must default to identity rotation */
        trans3d t_zero_q = {
            .x = 10.0f, .y = 20.0f, .z = 30.0f,
            .qx = 0.0f, .qy = 0.0f, .qz = 0.0f, .qw = 0.0f,
            .scale_x = 2.0f, .scale_y = 3.0f, .scale_z = 4.0f,
        };
        mat4f m = render_trans3d_mat4(&t_zero_q);

        /* test translation */
        TEST_ASSERT(feq(m.cols[3][0], 10.0f) && feq(m.cols[3][1], 20.0f) && feq(m.cols[3][2], 30.0f),
                    "translation column holds pos");

        /* test scaling on diagonal */
        TEST_ASSERT(feq(m.cols[0][0], 2.0f) && feq(m.cols[1][1], 3.0f) && feq(m.cols[2][2], 4.0f),
                    "scaling applied along diagonal");

        /* test normal matrix inverse transpose */
        mat3f n = mat3f_normal(m);
        /* normal along X should scale by 1 / 2.0 = 0.5 */
        vec3f nx = mat3f_mul_vec(n, vec3f(1, 0, 0));
        TEST_ASSERT(feq(nx.x, 0.5f) && feq(nx.y, 0.0f) && feq(nx.z, 0.0f), "normal scaled inversely");

        /* NULL returns identity */
        mat4f m_null = render_trans3d_mat4(NULL);
        mat4f id     = mat4f_id();
        TEST_ASSERT(memcmp(&m_null, &id, sizeof(mat4f)) == 0, "NULL trans3d returns identity matrix");
    } TEST_CASE_END();

    TEST_CASE("render_unlit_batch", "extracts unlit batches when no lights exist") {
        ecs_world_t *world = reset_world();
        TEST_ASSERT(world != NULL, "world created");

        mesh_registry_t meshes;
        TEST_ASSERT(mesh_registry_init(&meshes, &s_perm_arena, 16) == OK, "mesh reg initialized");

        asset_atlas_t atlas;
        TEST_ASSERT(asset_atlas_init(&atlas, &s_perm_arena, vec2i(128, 128), 16) == OK, "atlas initialized");

        /* spawn 3D mesh entity without any lights in the scene */
        entity_handle_t e1 = ecs_spawn();
        trans3d t1 = { .x = 0, .y = 0, .z = -5, .scale_x = 1, .scale_y = 1, .scale_z = 1 };
        mesh3d  m1 = { .mesh_id = MESH_ID_CUBE, .tex_id = 0, .color = 0xFFFFFFFFU };
        ecs_add(e1, trans3d, &t1);
        ecs_add(e1, mesh3d,  &m1);

        /* spawn 2D sprite entity */
        entity_handle_t e2 = ecs_spawn();
        trans2d  t2 = { .x = 10, .y = 20, .scale = 1.0f };
        sprite2d s2 = { .id = ASSET_SPRITE_WHITE, .layer = 0, .color = 0xFF0000FFU };
        ecs_add(e2, trans2d,  &t2);
        ecs_add(e2, sprite2d, &s2);

        render_view_t  view = render_view_def(vec2i(320, 180));
        render_batch_t batch;
        res_t r = render_build_batch(world, &atlas, &meshes, &view, &batch);

        TEST_ASSERT(r == OK, "batch build succeeds");
        TEST_ASSERT(batch.cmd3d_cnt == 1,    "1 mesh command extracted");
        TEST_ASSERT(batch.sprite2d_cnt == 1, "1 sprite instance extracted");

        /* STRICT UNLIT GUARANTEE: zero lights present -> light_cnt == 0 */
        TEST_ASSERT(batch.light_cnt == 0, "batch light_cnt must be 0 when no lights are in world");
        for (uint32_t i = 0; i < RENDER_MAX_LIGHTS; ++i) {
            TEST_ASSERT(batch.lights[i].pos_range.w == 0.0f && batch.lights[i].color.w == 0.0f,
                        "light slot %d must be zeroed", i);
        }

        /* 2D sprite instance invariants */
        TEST_ASSERT(batch.sprite_ents[0] == e2, "sprite entity handle tracked");
        TEST_ASSERT(feq(batch.sprites2d[0].pos.x, 10.0f) && feq(batch.sprites2d[0].pos.y, 20.0f), "sprite pos matches");
    } TEST_CASE_END();

    TEST_CASE("render_light_ranking", "ranks directional then nearby lights, caps at 8") {
        ecs_world_t *world = reset_world();
        mesh_registry_t meshes;
        mesh_registry_init(&meshes, &s_perm_arena, 16);
        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &s_perm_arena, vec2i(128, 128), 16);

        /* Camera at <0, 0, 10> */
        render_view_t view = render_view_def(vec2i(320, 180));
        view.cam3d.pos    = vec3f(0, 0, 10);
        view.cam3d.target = vec3f(0, 0, 0);

        /* Directional light 1: color red, travels along -Y */
        entity_handle_t ed1 = ecs_spawn();
        trans3d td1 = { .x = 100, .y = 100, .z = 100 }; /* position ignored for dir */
        light3d ld1 = { .type = LIGHT_DIR, .color = 0xFF0000FFU, .intensity = 1.0f };
        ecs_add(ed1, trans3d, &td1);
        ecs_add(ed1, light3d, &ld1);

        /* Directional light 2: color green */
        entity_handle_t ed2 = ecs_spawn();
        trans3d td2 = { .x = -50, .y = 0, .z = 0 };
        light3d ld2 = { .type = LIGHT_DIR, .color = 0x00FF00FFU, .intensity = 2.0f };
        ecs_add(ed2, trans3d, &td2);
        ecs_add(ed2, light3d, &ld2);

        /* Point light 1: pos <0, 0, 5> -> dist to cam = 5 (dist_sq = 25) */
        entity_handle_t ep1 = ecs_spawn();
        trans3d tp1 = { .x = 0, .y = 0, .z = 5 };
        light3d lp1 = { .type = LIGHT_POINT, .range = 15.0f, .intensity = 1.0f, .color = 0xFFFFFFFFU };
        ecs_add(ep1, trans3d, &tp1);
        ecs_add(ep1, light3d, &lp1);

        /* Point light 2: pos <0, 0, 8> -> dist to cam = 2 (dist_sq = 4) [CLOSEST POINT] */
        entity_handle_t ep2 = ecs_spawn();
        trans3d tp2 = { .x = 0, .y = 0, .z = 8 };
        light3d lp2 = { .type = LIGHT_POINT, .range = 10.0f, .intensity = 1.0f, .color = 0xFFFFFFFFU };
        ecs_add(ep2, trans3d, &tp2);
        ecs_add(ep2, light3d, &lp2);

        /* Spot light 1: pos <0, 0, 7> -> dist to cam = 3 (dist_sq = 9) */
        entity_handle_t es1 = ecs_spawn();
        trans3d ts1 = { .x = 0, .y = 0, .z = 7 };
        light3d ls1 = {
            .type = LIGHT_SPOT, .range = 12.0f, .intensity = 1.5f,
            .cone_in = 0.35f, .cone_out = 0.50f, .color = 0xFFFF00FFU
        };
        ecs_add(es1, trans3d, &ts1);
        ecs_add(es1, light3d, &ls1);

        /* Point light 3: pos <0, 0, 0> -> dist to cam = 10 (dist_sq = 100) */
        entity_handle_t ep3 = ecs_spawn();
        trans3d tp3 = { .x = 0, .y = 0, .z = 0 };
        light3d lp3 = { .type = LIGHT_POINT, .range = 20.0f, .intensity = 1.0f };
        ecs_add(ep3, trans3d, &tp3);
        ecs_add(ep3, light3d, &lp3);

        /* Point light 4: pos <0, 0, 20> -> dist to cam = 10 (dist_sq = 100) */
        entity_handle_t ep4 = ecs_spawn();
        trans3d tp4 = { .x = 0, .y = 0, .z = 20 };
        light3d lp4 = { .type = LIGHT_POINT, .range = 20.0f, .intensity = 1.0f };
        ecs_add(ep4, trans3d, &tp4);
        ecs_add(ep4, light3d, &lp4);

        /* Spot light 2: pos <0, 0, 50> -> dist to cam = 40 (dist_sq = 1600) [RANK 7] */
        entity_handle_t es2 = ecs_spawn();
        trans3d ts2 = { .x = 0, .y = 0, .z = 50 };
        light3d ls2 = { .type = LIGHT_SPOT, .range = 50.0f, .intensity = 1.0f, .cone_in = 0.2f, .cone_out = 0.4f };
        ecs_add(es2, trans3d, &ts2);
        ecs_add(es2, light3d, &ls2);

        /* Point light 5: pos <0, 0, 100> -> dist to cam = 90 (dist_sq = 8100) [OUT OF TOP 8] */
        entity_handle_t ep5 = ecs_spawn();
        trans3d tp5 = { .x = 0, .y = 0, .z = 100 };
        light3d lp5 = { .type = LIGHT_POINT, .range = 10.0f };
        ecs_add(ep5, trans3d, &tp5);
        ecs_add(ep5, light3d, &lp5);

        /* Spot light 3: pos <0, 0, 200> -> dist to cam = 190 (dist_sq = 36100) [OUT OF TOP 8] */
        entity_handle_t es3 = ecs_spawn();
        trans3d ts3 = { .x = 0, .y = 0, .z = 200 };
        light3d ls3 = { .type = LIGHT_SPOT, .range = 10.0f };
        ecs_add(es3, trans3d, &ts3);
        ecs_add(es3, light3d, &ls3);

        render_batch_t batch;
        res_t r = render_build_batch(world, &atlas, &meshes, &view, &batch);
        TEST_ASSERT(r == OK, "batch build succeeded");

        /* Capped at RENDER_MAX_LIGHTS = 8 */
        TEST_ASSERT(batch.light_cnt == 8, "batch light_cnt must be capped at 8, got %" PRIu32, batch.light_cnt);

        /* Rank 0 and 1: Directional lights always first */
        TEST_ASSERT(batch.lights[0].dir_type.w == (float)LIGHT_DIR, "slot 0 is directional");
        TEST_ASSERT(batch.lights[1].dir_type.w == (float)LIGHT_DIR, "slot 1 is directional");
        TEST_ASSERT(batch.lights[0].pos_range.w == 0.0f,            "directional light pos_range.w == 0");
        TEST_ASSERT(batch.lights[1].pos_range.w == 0.0f,            "directional light pos_range.w == 0");

        /* Rank 2: Point light 2 at dist 2 */
        TEST_ASSERT(batch.lights[2].dir_type.w == (float)LIGHT_POINT, "slot 2 is point light");
        TEST_ASSERT(feq(batch.lights[2].pos_range.z, 8.0f),           "slot 2 is point light at z=8 (dist=2)");
        TEST_ASSERT(feq(batch.lights[2].pos_range.w, 10.0f),          "slot 2 range is 10");

        /* Rank 3: Spot light 1 at dist 3 */
        TEST_ASSERT(batch.lights[3].dir_type.w == (float)LIGHT_SPOT, "slot 3 is spot light");
        TEST_ASSERT(feq(batch.lights[3].pos_range.z, 7.0f),          "slot 3 is spot light at z=7 (dist=3)");
        TEST_ASSERT(feq(batch.lights[3].cone.x, cosf(0.35f)),        "slot 3 cone_in is cos(0.35)");
        TEST_ASSERT(feq(batch.lights[3].cone.y, cosf(0.50f)),        "slot 3 cone_out is cos(0.50)");

        /* Rank 4: Point light 1 at dist 5 */
        TEST_ASSERT(batch.lights[4].dir_type.w == (float)LIGHT_POINT, "slot 4 is point light");
        TEST_ASSERT(feq(batch.lights[4].pos_range.z, 5.0f),           "slot 4 is point light at z=5 (dist=5)");

        /* Rank 5 & 6: Point lights 3 and 4 at dist 10 */
        TEST_ASSERT(batch.lights[5].dir_type.w == (float)LIGHT_POINT, "slot 5 is point light (dist 10)");
        TEST_ASSERT(batch.lights[6].dir_type.w == (float)LIGHT_POINT, "slot 6 is point light (dist 10)");

        /* Rank 7: Spot light 2 at dist 40 */
        TEST_ASSERT(batch.lights[7].dir_type.w == (float)LIGHT_SPOT, "slot 7 is spot light (dist 40)");
        TEST_ASSERT(feq(batch.lights[7].pos_range.z, 50.0f),         "slot 7 is spot light at z=50");
    } TEST_CASE_END();

    TEST_CASE("render_sort_layers", "sorts 3D keys and stably sorts 2D layers") {
        ecs_world_t *world = reset_world();
        mesh_registry_t meshes;
        mesh_registry_init(&meshes, &s_perm_arena, 16);
        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &s_perm_arena, vec2i(128, 128), 16);

        /* 3D mesh commands with unsorted (mesh_id, sprite_id) keys */
        entity_handle_t em1 = ecs_spawn();
        trans3d tm = { .scale_x = 1, .scale_y = 1, .scale_z = 1 };
        mesh3d  mm1 = { .mesh_id = MESH_ID_CUBE, .tex_id = 5 };
        ecs_add(em1, trans3d, &tm);
        ecs_add(em1, mesh3d,  &mm1);

        entity_handle_t em2 = ecs_spawn();
        mesh3d  mm2 = { .mesh_id = MESH_ID_CUBE, .tex_id = 1 };
        ecs_add(em2, trans3d, &tm);
        ecs_add(em2, mesh3d,  &mm2);

        entity_handle_t em3 = ecs_spawn();
        mesh3d  mm3 = { .mesh_id = MESH_ID_CUBE, .tex_id = 3 };
        ecs_add(em3, trans3d, &tm);
        ecs_add(em3, mesh3d,  &mm3);

        /* 2D sprites with varying layers and flip masks */
        entity_handle_t es1 = ecs_spawn();
        trans2d ts = { .scale = 1.0f };
        sprite2d ss1 = { .id = ASSET_SPRITE_WHITE, .layer = 100, .flip_mask = 0 };
        ecs_add(es1, trans2d,  &ts);
        ecs_add(es1, sprite2d, &ss1);

        entity_handle_t es2 = ecs_spawn();
        sprite2d ss2 = { .id = ASSET_SPRITE_WHITE, .layer = 10, .flip_mask = RENDER_FLIP_H };
        ecs_add(es2, trans2d,  &ts);
        ecs_add(es2, sprite2d, &ss2);

        entity_handle_t es3 = ecs_spawn();
        sprite2d ss3 = { .id = ASSET_SPRITE_WHITE, .layer = 255, .flip_mask = RENDER_FLIP_V };
        ecs_add(es3, trans2d,  &ts);
        ecs_add(es3, sprite2d, &ss3);

        entity_handle_t es4 = ecs_spawn();
        sprite2d ss4 = { .id = ASSET_SPRITE_WHITE, .layer = 0, .flip_mask = 0 };
        ecs_add(es4, trans2d,  &ts);
        ecs_add(es4, sprite2d, &ss4);

        entity_handle_t es5 = ecs_spawn();
        sprite2d ss5 = { .id = ASSET_SPRITE_WHITE, .layer = 10, .flip_mask = 0 };
        ecs_add(es5, trans2d,  &ts);
        ecs_add(es5, sprite2d, &ss5);

        render_view_t  view = render_view_def(vec2i(320, 180));
        render_batch_t batch;
        res_t r = render_build_batch(world, &atlas, &meshes, &view, &batch);
        TEST_ASSERT(r == OK, "batch build OK");

        /* Verify 3D radix sort ascending order */
        TEST_ASSERT(batch.cmd3d_cnt == 3, "3 mesh commands");
        TEST_ASSERT(batch.cmds3d[0].sort_key <= batch.cmds3d[1].sort_key &&
                    batch.cmds3d[1].sort_key <= batch.cmds3d[2].sort_key,
                    "3D commands sorted ascending by key");
        TEST_ASSERT((batch.cmds3d[0].sort_key & 0xFFFFU) == 1, "first mesh sprite_id is 1");
        TEST_ASSERT((batch.cmds3d[1].sort_key & 0xFFFFU) == 3, "second mesh sprite_id is 3");
        TEST_ASSERT((batch.cmds3d[2].sort_key & 0xFFFFU) == 5, "third mesh sprite_id is 5");

        /* Verify 2D stable counting sort by layer: 0, 10, 10, 100, 255 */
        TEST_ASSERT(batch.sprite2d_cnt == 5, "5 sprite instances");
        TEST_ASSERT(batch.sprite_ents[0] == es4, "layer 0 entity first");
        TEST_ASSERT(batch.sprite_ents[1] == es2, "layer 10 first entity second (stable order)");
        TEST_ASSERT(batch.sprite_ents[2] == es5, "layer 10 second entity third (stable order)");
        TEST_ASSERT(batch.sprite_ents[3] == es1, "layer 100 entity fourth");
        TEST_ASSERT(batch.sprite_ents[4] == es3, "layer 255 entity fifth");

        /* Verify UV flipping */
        aabb2f base_uv = asset_get_uv(&atlas, ASSET_SPRITE_WHITE);
        /* es2 had RENDER_FLIP_H: min.x and max.x swapped */
        TEST_ASSERT(feq(batch.sprites2d[1].uv.min.x, base_uv.max.x) &&
                    feq(batch.sprites2d[1].uv.max.x, base_uv.min.x), "H-flip swapped horizontal UVs");
        /* es3 had RENDER_FLIP_V: min.y and max.y swapped */
        TEST_ASSERT(feq(batch.sprites2d[4].uv.min.y, base_uv.max.y) &&
                    feq(batch.sprites2d[4].uv.max.y, base_uv.min.y), "V-flip swapped vertical UVs");
    } TEST_CASE_END();

    TEST_CASE("render_errors", "validates boundary error checks and null fallbacks") {
        ecs_world_t *world = reset_world();
        mesh_registry_t meshes;
        mesh_registry_init(&meshes, &s_perm_arena, 16);
        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &s_perm_arena, vec2i(128, 128), 16);

        render_view_t  view = render_view_def(vec2i(320, 180));
        render_batch_t batch;

        /* Non-positive viewport returns EINVAL */
        render_view_t invalid_vp = view;
        invalid_vp.viewport = vec2i(0, 100);
        TEST_ASSERT(render_build_batch(world, &atlas, &meshes, &invalid_vp, &batch) == EINVAL, "zero viewport.x -> EINVAL");
        invalid_vp.viewport = vec2i(100, -5);
        TEST_ASSERT(render_build_batch(world, &atlas, &meshes, &invalid_vp, &batch) == EINVAL, "negative viewport.y -> EINVAL");

        /* Meshes present but NULL meshes parameter returns EINVAL */
        entity_handle_t e3d = ecs_spawn();
        trans3d tm = { .scale_x = 1, .scale_y = 1, .scale_z = 1 };
        mesh3d  mm = { .mesh_id = MESH_ID_CUBE };
        ecs_add(e3d, trans3d, &tm);
        ecs_add(e3d, mesh3d,  &mm);
        TEST_ASSERT(render_build_batch(world, &atlas, NULL, &view, &batch) == EINVAL, "NULL meshes with 3D renderables -> EINVAL");

        /* Sprites present but NULL atlas parameter returns EINVAL */
        entity_handle_t e2d = ecs_spawn();
        trans2d  ts = { .scale = 1.0f };
        sprite2d ss = { .id = ASSET_SPRITE_WHITE };
        ecs_add(e2d, trans2d,  &ts);
        ecs_add(e2d, sprite2d, &ss);
        TEST_ASSERT(render_build_batch(world, NULL, &meshes, &view, &batch) == EINVAL, "NULL atlas with 2D renderables -> EINVAL");

        /* Unknown mesh ID gracefully falls back to MESH_ID_CUBE */
        mm.mesh_id = 999;
        ecs_add(e3d, mesh3d, &mm);
        /* Unknown sprite ID gracefully falls back to ASSET_SPRITE_CHECKER */
        ss.id = 999;
        ecs_add(e2d, sprite2d, &ss);

        res_t r = render_build_batch(world, &atlas, &meshes, &view, &batch);
        TEST_ASSERT(r == OK, "batch build succeeds with unknown mesh_id and sprite_id");
        TEST_ASSERT(batch.cmds3d[0].mesh == mesh_get(&meshes, MESH_ID_CUBE), "unknown mesh_id falls back to cube");
        TEST_ASSERT(batch.sprite2d_cnt == 1, "1 sprite extracted");
    } TEST_CASE_END();

    TEST_CASE("render_world_bind", "restores world binding on success and error") {
        ecs_world_t *active = reset_world();
        ecs_world_t *other = ecs_world_create(&s_perm_arena, NULL, &s_scratch_arena);
        TEST_ASSERT(other != NULL && ecs_get_world() == active, "creation preserves existing binding");
        ecs_set_world(other);
        entity_handle_t ent = ecs_spawn();
        TEST_ASSERT(ecs_add(ent, trans2d, NULL) != NULL, "override transform added");
        TEST_ASSERT(ecs_add(ent, sprite2d, NULL) != NULL, "override sprite added");
        ecs_set_world(active);

        asset_atlas_t atlas;
        TEST_ASSERT(asset_atlas_init(&atlas, &s_perm_arena, vec2i(32, 32), 16) == OK, "atlas initialized");
        render_view_t view = render_view_def(vec2i(320, 180));
        render_batch_t batch = {0};
        TEST_ASSERT(render_build_batch(other, &atlas, NULL, &view, &batch) == OK, "explicit world extraction succeeds");
        TEST_ASSERT(ecs_get_world() == active, "success restores original binding");
        TEST_ASSERT(batch.sprite2d_cnt == 1 && batch.sprite_ents[0] == ent, "override world supplies renderables");
        TEST_ASSERT(render_build_batch(other, NULL, NULL, &view, &batch) == EINVAL, "missing atlas causes extraction error");
        TEST_ASSERT(ecs_get_world() == active, "error restores original binding");
        TEST_ASSERT(batch.sprite2d_cnt == 0 && batch.sprites2d == NULL && batch.cmds3d == NULL,
                    "error clears previous batch output");

        ecs_set_world(NULL);
        TEST_ASSERT(render_build_batch(other, &atlas, NULL, &view, &batch) == OK, "explicit world works with no binding");
        TEST_ASSERT(ecs_get_world() == NULL, "null binding restored");
        TEST_ASSERT(render_build_batch(NULL, &atlas, NULL, &view, &batch) == ENOTFND, "absent world returns ENOTFND");
        ecs_set_world(active);
    } TEST_CASE_END();

    ecs_set_world(NULL);
    arena_free_vm(&s_perm_arena);
    arena_free_vm(&s_scratch_arena);

    TEST_END();
}
