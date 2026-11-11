#include "mesh.h"
#include "arena.h"
#include "ecs.h"
#include "spatial.h"
#include "test.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define EPS 0.001f

static inline bool feq(float a, float b) {
    return fabsf(a - b) <= EPS;
}

static inline bool _verify_ccw_winding(const mesh_t *m) {
    if (!m || !m->verts || !m->indices || (m->idx_cnt % 3U) != 0U) return false;
    for (uint32_t i = 0; i < m->idx_cnt; i += 3U) {
        uint32_t i0 = m->indices[i + 0U];
        uint32_t i1 = m->indices[i + 1U];
        uint32_t i2 = m->indices[i + 2U];
        if (i0 >= m->vert_cnt || i1 >= m->vert_cnt || i2 >= m->vert_cnt) return false;

        vec3f p0 = m->verts[i0].pos;
        vec3f p1 = m->verts[i1].pos;
        vec3f p2 = m->verts[i2].pos;
        vec3f cr = vec_cross(vec_sub(p1, p0), vec_sub(p2, p0));

        /* degenerate pole triangles on UV spheres have near-zero area */
        if (vec_len_sq(cr) <= 1e-8f) continue;

        vec3f avg_n = vec_add(m->verts[i0].norm, m->verts[i1].norm, m->verts[i2].norm);
        if (vec_dot(cr, avg_n) <= 0.0f) return false;
    }
    return true;
}

int main(void) {
    static uint8_t perm_buf[MB(2)];
    static uint8_t scratch_buf[MB(1)];
    arena_t perm, scratch;

    TEST_CASE("mesh_init_and_reset", "initializes and resets registry with built-in cube") {
        arena_init(&perm, perm_buf, sizeof(perm_buf));

        mesh_registry_t reg;
        res_t r = mesh_registry_init(&reg, &perm, 16);
        TEST_ASSERT(r == OK,              "expected OK, got %d", r);
        TEST_ASSERT(reg.max_meshes == 16, "expected max_meshes 16, got %" PRIu32, reg.max_meshes);
        TEST_ASSERT(reg.mesh_cnt == 1,    "expected 1 built-in mesh, got %" PRIu32, reg.mesh_cnt);

        const mesh_t *cube = mesh_get(&reg, MESH_ID_CUBE);
        TEST_ASSERT(cube != NULL,        "MESH_ID_CUBE must exist");
        TEST_ASSERT(cube->vert_cnt == 24, "unit cube must have 24 verts, got %" PRIu32, cube->vert_cnt);
        TEST_ASSERT(cube->idx_cnt == 36,  "unit cube must have 36 indices, got %" PRIu32, cube->idx_cnt);
        TEST_ASSERT(feq(cube->bounds.min.x, -0.5f) && feq(cube->bounds.max.x, 0.5f), "cube X bounds [-0.5, 0.5]");
        TEST_ASSERT(feq(cube->bounds.min.y, -0.5f) && feq(cube->bounds.max.y, 0.5f), "cube Y bounds [-0.5, 0.5]");
        TEST_ASSERT(feq(cube->bounds.min.z, -0.5f) && feq(cube->bounds.max.z, 0.5f), "cube Z bounds [-0.5, 0.5]");
        TEST_ASSERT(_verify_ccw_winding(cube), "unit cube triangles must have outward CCW winding");

        /* out-of-bounds lookup falls back to MESH_ID_CUBE in mesh_get, NULL in mesh_get_strict */
        TEST_ASSERT(mesh_get(&reg, 99) == cube,        "mesh_get fallback to MESH_ID_CUBE");
        TEST_ASSERT(mesh_get_strict(&reg, 99) == NULL, "mesh_get_strict returns NULL on invalid id");

        mesh_id_t box_id = MESH_ID_NULL;
        TEST_ASSERT(mesh_gen_box(&reg, vec3f(2.0f, 2.0f, 2.0f), &box_id) == OK, "gen_box OK");
        TEST_ASSERT(box_id == 1 && reg.mesh_cnt == 2, "allocated id 1");

        TEST_ASSERT(mesh_registry_reset(&reg) == OK,  "reset OK");
        TEST_ASSERT(reg.mesh_cnt == 1,                "reset restores mesh_cnt = 1");
        TEST_ASSERT(mesh_get_strict(&reg, 1) == NULL, "id 1 cleared after reset");
        TEST_ASSERT(mesh_get(&reg, MESH_ID_CUBE)->vert_cnt == 24, "unit cube preserved after reset");
    } TEST_CASE_END();

    TEST_CASE("mesh_register_custom", "copies indexed geometry and checks bounds/limits") {
        arena_init(&perm, perm_buf, sizeof(perm_buf));

        mesh_registry_t reg;
        mesh_registry_init(&reg, &perm, 2);

        mesh_vert_t verts[3] = {
            { .pos = vec3f(-1.0f,  0.0f, 2.0f), .norm = vec3f(0.0f, 0.0f, 1.0f), .uv = vec2f(0.0f, 0.0f) },
            { .pos = vec3f( 3.0f,  0.0f, 2.0f), .norm = vec3f(0.0f, 0.0f, 1.0f), .uv = vec2f(1.0f, 0.0f) },
            { .pos = vec3f( 1.0f,  4.0f, 2.0f), .norm = vec3f(0.0f, 0.0f, 1.0f), .uv = vec2f(0.5f, 1.0f) },
        };
        uint32_t good_idx[3] = { 0, 1, 2 };
        uint32_t bad_idx[3]  = { 0, 1, 3 };

        mesh_id_t id = MESH_ID_NULL;
        /* non-multiple-of-3 index count rejected */
        TEST_ASSERT(mesh_register(&reg, verts, 3, good_idx, 2, &id) == EINVAL, "idx_cnt=2 must return EINVAL");
        /* out-of-bounds vertex index rejected */
        TEST_ASSERT(mesh_register(&reg, verts, 3, bad_idx,  3, &id) == EINVAL, "index >= vert_cnt must return EINVAL");

        TEST_ASSERT(mesh_register(&reg, verts, 3, good_idx, 3, &id) == OK, "valid triangle registration OK");
        TEST_ASSERT(id == 1, "expected id 1, got %" PRIu16, id);

        const mesh_t *m = mesh_get_strict(&reg, id);
        TEST_ASSERT(m != NULL, "mesh 1 exists");
        TEST_ASSERT(feq(m->bounds.min.x, -1.0f) && feq(m->bounds.max.x, 3.0f), "bounds X [-1, 3]");
        TEST_ASSERT(feq(m->bounds.min.y,  0.0f) && feq(m->bounds.max.y, 4.0f), "bounds Y [0, 4]");
        TEST_ASSERT(feq(m->bounds.min.z,  2.0f) && feq(m->bounds.max.z, 2.0f), "bounds Z [2, 2]");

        verts[0].pos.x = 100.0f;
        good_idx[0] = 2;
        TEST_ASSERT(feq(m->verts[0].pos.x, -1.0f), "registered vertices are owned copies");
        TEST_ASSERT(m->indices[0] == 0, "registered indices are owned copies");

        /* registry capacity limit (max_meshes = 2: id 0 + id 1) */
        mesh_id_t overflow_id = MESH_ID_NULL;
        TEST_ASSERT(mesh_register(&reg, verts, 3, good_idx, 3, &overflow_id) == EFULL, "expected EFULL");
    } TEST_CASE_END();

    TEST_CASE("mesh_shared_reset", "preserves unrelated perm allocations across resets") {
        arena_init(&perm, perm_buf, sizeof(perm_buf));
        mesh_registry_t reg;
        TEST_ASSERT(mesh_registry_init(&reg, &perm, 8) == OK, "registry initialized");
        size_t base = perm.off;
        mesh_id_t id = MESH_ID_NULL;
        TEST_ASSERT(mesh_gen_box(&reg, vec3f_splat(2), &id) == OK, "first custom mesh created");
        TEST_ASSERT(mesh_registry_reset(&reg) == OK, "tail reset succeeds");
        TEST_ASSERT(perm.off == base, "tail geometry reclaimed");

        TEST_ASSERT(mesh_gen_box(&reg, vec3f_splat(3), &id) == OK, "second custom mesh created");
        uint8_t *other = arena_alloc(&perm, CACHE_LINE_SZ);
        TEST_ASSERT(other != NULL, "unrelated allocation succeeds");
        if (other) {
            memset(other, 0xA5, CACHE_LINE_SZ);
            size_t shared_end = perm.off;
            TEST_ASSERT(mesh_registry_reset(&reg) == OK, "shared reset succeeds");
            TEST_ASSERT(perm.off == shared_end, "shared reset preserves arena offset");
            TEST_ASSERT(mesh_get_strict(&reg, id) == NULL, "custom descriptor invalidated");
            TEST_ASSERT(mesh_gen_box(&reg, vec3f_splat(4), &id) == OK, "registry reusable after shared reset");
            TEST_ASSERT(mesh_registry_reset(&reg) == OK, "subsequent reset succeeds");
            TEST_ASSERT(perm.off >= shared_end, "subsequent reset preserves unrelated allocation");
            uint8_t *reused = arena_alloc(&perm, CACHE_LINE_SZ);
            TEST_ASSERT(reused != NULL, "allocation after subsequent reset succeeds");
            if (reused) memset(reused, 0x3C, CACHE_LINE_SZ);
            for (size_t i = 0; i < CACHE_LINE_SZ; ++i)
                TEST_ASSERT(other[i] == 0xA5, "unrelated byte %zu preserved", i);
        }
    } TEST_CASE_END();

    TEST_CASE("mesh_interleaved", "preserves perm allocations between mesh additions") {
        arena_init(&perm, perm_buf, sizeof(perm_buf));
        mesh_registry_t reg;
        TEST_ASSERT(mesh_registry_init(&reg, &perm, 8) == OK, "registry initialized");
        mesh_id_t id = MESH_ID_NULL;
        TEST_ASSERT(mesh_gen_box(&reg, vec3f_splat(2), &id) == OK, "first mesh created");
        uint8_t *other = arena_alloc(&perm, CACHE_LINE_SZ);
        TEST_ASSERT(other != NULL, "unrelated allocation succeeds");
        size_t shared_end = perm.off;
        TEST_ASSERT(mesh_gen_box(&reg, vec3f_splat(3), &id) == OK, "second mesh created after unrelated storage");
        TEST_ASSERT(mesh_registry_reset(&reg) == OK, "reset succeeds");
        TEST_ASSERT(perm.off >= shared_end, "reset cannot reclaim across unrelated allocation");
    } TEST_CASE_END();

    TEST_CASE("mesh_primitives", "generates primitives with outward CCW winding") {
        arena_init(&perm, perm_buf, sizeof(perm_buf));

        mesh_registry_t reg;
        mesh_registry_init(&reg, &perm, 16);

        /* 1. Box */
        mesh_id_t box_id = MESH_ID_NULL;
        TEST_ASSERT(mesh_gen_box(&reg, vec3f(2.0f, 4.0f, 6.0f), &box_id) == OK, "gen_box OK");
        const mesh_t *box = mesh_get_strict(&reg, box_id);
        TEST_ASSERT(box && box->vert_cnt == 24 && box->idx_cnt == 36, "box counts");
        TEST_ASSERT(feq(box->bounds.min.x, -1.0f) && feq(box->bounds.max.x, 1.0f), "box X bounds");
        TEST_ASSERT(feq(box->bounds.min.y, -2.0f) && feq(box->bounds.max.y, 2.0f), "box Y bounds");
        TEST_ASSERT(feq(box->bounds.min.z, -3.0f) && feq(box->bounds.max.z, 3.0f), "box Z bounds");
        TEST_ASSERT(_verify_ccw_winding(box), "box CCW winding");

        /* 2. Subdivided Plane */
        mesh_id_t plane_id = MESH_ID_NULL;
        TEST_ASSERT(mesh_gen_plane(&reg, vec2f(10.0f, 8.0f), 4, 3, &plane_id) == OK, "gen_plane OK");
        const mesh_t *plane = mesh_get_strict(&reg, plane_id);
        TEST_ASSERT(plane && plane->vert_cnt == 20 && plane->idx_cnt == 72, "plane 5x4 verts, 4x3x6 indices");
        TEST_ASSERT(feq(plane->bounds.min.x, -5.0f) && feq(plane->bounds.max.x, 5.0f), "plane X bounds");
        TEST_ASSERT(feq(plane->bounds.min.y,  0.0f) && feq(plane->bounds.max.y, 0.0f), "plane Y=0");
        TEST_ASSERT(feq(plane->bounds.min.z, -4.0f) && feq(plane->bounds.max.z, 4.0f), "plane Z bounds");
        TEST_ASSERT(_verify_ccw_winding(plane), "plane CCW winding");

        /* 3. UV Sphere */
        mesh_id_t sph_id = MESH_ID_NULL;
        TEST_ASSERT(mesh_gen_sphere(&reg, 3.0f, 16, 12, &sph_id) == OK, "gen_sphere OK");
        const mesh_t *sph = mesh_get_strict(&reg, sph_id);
        TEST_ASSERT(sph && sph->vert_cnt == 13 * 17 && sph->idx_cnt == 12 * 16 * 6, "sphere counts");
        for (uint32_t i = 0; i < sph->vert_cnt; ++i) {
            TEST_ASSERT(feq(vec_len(sph->verts[i].pos),  3.0f), "sphere vertex on radius=3");
            TEST_ASSERT(feq(vec_len(sph->verts[i].norm), 1.0f), "sphere unit normal");
        }
        TEST_ASSERT(feq(sph->bounds.min.y, -3.0f) && feq(sph->bounds.max.y, 3.0f), "sphere Y bounds [-3, 3]");
        TEST_ASSERT(_verify_ccw_winding(sph), "sphere CCW winding");

        /* 4. Upright Cylinder */
        mesh_id_t cyl_id = MESH_ID_NULL;
        TEST_ASSERT(mesh_gen_cylinder(&reg, 2.0f, 6.0f, 12, &cyl_id) == OK, "gen_cylinder OK");
        const mesh_t *cyl = mesh_get_strict(&reg, cyl_id);
        TEST_ASSERT(cyl && cyl->idx_cnt == 12 * 12, "cylinder index count");
        TEST_ASSERT(feq(cyl->bounds.min.x, -2.0f) && feq(cyl->bounds.max.x, 2.0f), "cylinder X bounds [-2, 2]");
        TEST_ASSERT(feq(cyl->bounds.min.y, -3.0f) && feq(cyl->bounds.max.y, 3.0f), "cylinder Y bounds [-3, 3]");
        TEST_ASSERT(feq(cyl->bounds.min.z, -2.0f) && feq(cyl->bounds.max.z, 2.0f), "cylinder Z bounds [-2, 2]");
        TEST_ASSERT(_verify_ccw_winding(cyl), "cylinder CCW winding");
    } TEST_CASE_END();

    TEST_CASE("mesh_obj_mem", "parses OBJ quads, indices, UVs and flat normals") {
        arena_init(&perm,    perm_buf,    sizeof(perm_buf));
        arena_init(&scratch, scratch_buf, sizeof(scratch_buf));

        mesh_registry_t reg;
        mesh_registry_init(&reg, &perm, 16);

        /* Quad in XY plane + triangle with explicit normals and negative indices */
        const char *obj_src =
            "# Test Wavefront OBJ\n"
            "o test_mesh\n"
            "v -1.0 -1.0  0.0\n"
            "v  1.0 -1.0  0.0\n"
            "v  1.0  1.0  0.0\n"
            "v -1.0  1.0  0.0\n"
            "vt 0.0 0.0\n"
            "vt 1.0 0.0\n"
            "vt 1.0 1.0\n"
            "vt 0.0 1.0\n"
            "# Quad without vn (auto-computes +Z normal) using v/vt\n"
            "f 1/1 2/2 3/3 4/4\n"
            "# Second triangle using negative relative indices and explicit vn\n"
            "v  0.0  0.0  2.0\n"
            "v  2.0  0.0  2.0\n"
            "v  0.0  2.0  2.0\n"
            "vn 0.0  0.0  1.0\n"
            "f -3//-1 -2//-1 -1//-1\n";

        size_t scratch_before = scratch.off;
        mesh_id_t obj_id = MESH_ID_NULL;
        res_t r = mesh_load_obj_mem(&reg, obj_src, strlen(obj_src), &scratch, &obj_id);
        TEST_ASSERT(r == OK, "mesh_load_obj_mem expected OK, got %d", r);
        TEST_ASSERT(scratch.off == scratch_before, "scratch arena must be unwound after obj parse");

        const mesh_t *m = mesh_get_strict(&reg, obj_id);
        TEST_ASSERT(m != NULL, "parsed obj mesh exists");
        /* 1 quad (2 tris) + 1 tri = 3 tris = 9 verts, 9 indices */
        TEST_ASSERT(m->vert_cnt == 9 && m->idx_cnt == 9, "expected 9 verts/indices, got %" PRIu32, m->vert_cnt);
        TEST_ASSERT(feq(m->verts[0].norm.z, 1.0f), "auto-computed flat normal should be +Z (0, 0, 1)");
        TEST_ASSERT(feq(m->verts[6].pos.z,  2.0f) && feq(m->verts[6].norm.z, 1.0f), "negative index triangle parsed");
        TEST_ASSERT(feq(m->bounds.min.x, -1.0f) && feq(m->bounds.max.x, 2.0f), "bounds X [-1, 2]");
        TEST_ASSERT(feq(m->bounds.min.y, -1.0f) && feq(m->bounds.max.y, 2.0f), "bounds Y [-1, 2]");
        TEST_ASSERT(feq(m->bounds.min.z,  0.0f) && feq(m->bounds.max.z, 2.0f), "bounds Z [0, 2]");

        /* invalid OBJ with out-of-range vertex index must fail cleanly and roll back */
        const char *bad_obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 99\n";
        size_t perm_before = perm.off;
        uint32_t cnt_before = reg.mesh_cnt;
        mesh_id_t bad_id = MESH_ID_NULL;
        TEST_ASSERT(mesh_load_obj_mem(&reg, bad_obj, strlen(bad_obj), &scratch, &bad_id) == ENOTFND, "bad index -> ENOTFND");
        TEST_ASSERT(reg.mesh_cnt == cnt_before && perm.off == perm_before, "perm and mesh_cnt rolled back on parse failure");
        TEST_ASSERT(scratch.off == scratch_before, "scratch unwound on parse failure");
        TEST_ASSERT(mesh_load_obj_mem(&reg, obj_src, strlen(obj_src), &scratch, &obj_id) == OK,
                    "valid load succeeds after failed parse");
    } TEST_CASE_END();

    TEST_CASE("mesh_file_collider", "loads .obj from disk and fits collider3d to bounds") {
        arena_t vm_perm;
        arena_init_vm(&vm_perm, 0);
        arena_init(&scratch, scratch_buf, sizeof(scratch_buf));

        mesh_registry_t reg;
        mesh_registry_init(&reg, &vm_perm, 16);

        const char *tmp_path = "/tmp/bonsai_test_pyramid.obj";
        const char *pyr_obj =
            "v 2.0 0.0  1.0\n"
            "v 6.0 0.0  1.0\n"
            "v 6.0 0.0  5.0\n"
            "v 2.0 0.0  5.0\n"
            "v 4.0 8.0  3.0\n"
            "f 1 2 5\n"
            "f 2 3 5\n"
            "f 3 4 5\n"
            "f 4 1 5\n";

        FILE *fp = fopen(tmp_path, "wb");
        TEST_ASSERT(fp != NULL, "created temp obj file");
        fwrite(pyr_obj, 1, strlen(pyr_obj), fp);
        fclose(fp);

        mesh_id_t pyr_id = MESH_ID_NULL;
        TEST_ASSERT(mesh_load_obj_file(&reg, "/tmp/nonexistent_bonsai.obj", &scratch, &pyr_id) == ENOTFND, "missing file");
        TEST_ASSERT(mesh_load_obj_file(&reg, tmp_path, &scratch, &pyr_id) == OK, "loaded pyramid .obj from file");
        unlink(tmp_path);

        const mesh_t *pyr = mesh_get_strict(&reg, pyr_id);
        TEST_ASSERT(pyr && pyr->vert_cnt == 12, "4 triangles = 12 vertices");

        /* Fit collider3d to offset pyramid bounds [2..6, 0..8, 1..5] */
        collider3d col = mesh_fit_collider3d(pyr, 0x04U, false);
        TEST_ASSERT(feq(col.half_x, 2.0f) && feq(col.half_y, 4.0f) && feq(col.half_z, 2.0f), "fitted half extents (2, 4, 2)");
        TEST_ASSERT(feq(col.off_x,  4.0f) && feq(col.off_y,  4.0f) && feq(col.off_z,  3.0f), "fitted center offset (4, 4, 3)");
        TEST_ASSERT(col.mask == 0x04U && col.shape == COLLIDER_BOX, "fitted mask and COLLIDER_BOX");

        /* Verify ECS mesh3d component reflection and default values */
        ecs_world_t *world = ecs_world_create(&vm_perm, NULL, &scratch);
        ecs_set_world(world);

        entity_handle_t ent = ecs_spawn();
        trans3d t3 = { .x = 10.0f, .y = 0.0f, .z = 0.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f };
        ecs_add(ent, trans3d,    &t3);
        ecs_add(ent, mesh3d,     NULL);
        ecs_add(ent, collider3d, &col);

        mesh3d *mc = ecs_get(ent, mesh3d);
        TEST_ASSERT(mc != NULL, "mesh3d component attached");
        TEST_ASSERT(mc->mesh_id == MESH_ID_CUBE && mc->tex_id == 0 && mc->color == 0xFFFFFFFFU, "mesh3d defaults");
        mc->mesh_id = pyr_id;

        aabb3f world_box = spatial_calc_aabb(ent, ecs_get(ent, collider3d));
        TEST_ASSERT(feq(world_box.min.x, 12.0f) && feq(world_box.max.x, 16.0f), "world AABB X [12, 16]");
        TEST_ASSERT(feq(world_box.min.y,  0.0f) && feq(world_box.max.y,  8.0f), "world AABB Y [0, 8]");
        TEST_ASSERT(feq(world_box.min.z,  1.0f) && feq(world_box.max.z,  5.0f), "world AABB Z [1, 5]");

        arena_free_vm(&vm_perm);
    } TEST_CASE_END();

    TEST_END();
}
