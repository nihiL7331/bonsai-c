/*
====================
render.c

Backend-agnostic render batch extraction:
- camera / model matrix construction
- multi-light extraction (max 8)
- pass 1 mesh command extraction + 4-pass LSD radix sort by (mesh, sprite) key
- pass 2 sprite instance extraction + stable 256-bucket layer counting sort
====================
*/

#include "render.h"
#include "core.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

/* ============================================================================================== */
/* >INTERNAL HELPERS                                                                              */
/* ============================================================================================== */

/* stable LSD radix sort on 32-bit sort_key, 4 passes leave the result in `cmds` */
static void _render_sort_cmds3d(render_cmd3d_t *cmds, render_cmd3d_t *tmp, uint32_t cnt) {
    render_cmd3d_t *src = cmds, *dst = tmp;
    for (uint32_t shift = 0; shift < 32U; shift += 8U) {
        uint32_t offs[257] = {0};
        for (uint32_t i = 0; i  < cnt;  ++i) offs[((src[i].sort_key >> shift) & 0xFFU) + 1U] += 1U;
        for (uint32_t b = 1; b <= 256U; ++b) offs[b]                                         += offs[b - 1U];
        for (uint32_t i = 0; i  < cnt;  ++i) dst[offs[(src[i].sort_key >> shift) & 0xFFU]++]  = src[i];
        SWAP(src, dst);
    }
}

static const asset_sprite_t *_render_resolve_sprite(const asset_atlas_t *atlas, asset_sprite_id_t id) {
    const asset_sprite_t *sp = asset_get_sprite(atlas, id);
    return sp ? sp : asset_get_sprite(atlas, ASSET_SPRITE_CHECKER);
}

/* ============================================================================================== */
/* >MATRIX HELPERS                                                                                */
/* ============================================================================================== */

mat4f render_mat4f_lookat(vec3f eye, vec3f target, vec3f up) {
    vec3f diff = vec_sub(target, eye);
    vec3f f    = vec_len_sq(diff) > 1e-12f ? vec_norm(diff) : vec3f(0, 0, -1);
    vec3f up_n = vec_len_sq(up)   > 1e-12f ? vec_norm(up)   : vec3f(0, 1,  0);

    vec3f s_raw = vec_cross(f, up_n);
    if (vec_len_sq(s_raw) <= 1e-8f) s_raw = vec_cross(f, ABS(f.y) < 0.99f ? vec3f(0, 1, 0) : vec3f(1, 0, 0));

    vec3f s = vec_norm(s_raw);
    vec3f u = vec_cross(s, f);

    return mat4f(
        vec4f( s.x,              u.x,            -f.x,             0),
        vec4f( s.y,              u.y,            -f.y,             0),
        vec4f( s.z,              u.z,            -f.z,             0),
        vec4f(-vec_dot(s, eye), -vec_dot(u, eye), vec_dot(f, eye), 1)
    );
}

mat4f render_cam3d_vp(const render_cam3d_t *cam, vec2i viewport) {
    render_cam3d_t c = cam ? *cam : render_cam3d_def();

    float aspect = (viewport.x > 0 && viewport.y > 0) ? (float)viewport.x / (float)viewport.y : RENDER_DEF_ASPECT;
    float fov    = c.fov_rad > 1e-4f                  ? c.fov_rad                             : RENDER_DEF_FOV_RAD;
    float near_z = c.near_z  > 1e-4f                  ? c.near_z                              : RENDER_DEF_NEAR_Z;
    float far_z  = c.far_z   > near_z + 1e-3f         ? c.far_z                               : MAX(RENDER_DEF_FAR_Z, near_z + 1.0f);

    return mat4f_mul(mat4f_persp(fov, aspect, near_z, far_z), render_mat4f_lookat(c.pos, c.target, c.up));
}

mat4f render_cam2d_vp(const render_cam2d_t *cam, vec2i viewport) {
    render_cam2d_t c = cam ? *cam : render_cam2d_def();

    float w    = viewport.x > 0 ? (float)viewport.x : RENDER_DEF_VIEWPORT_W;
    float h    = viewport.y > 0 ? (float)viewport.y : RENDER_DEF_VIEWPORT_H;
    float zoom = c.zoom > 1e-5f ? c.zoom            : 1.0f;

    mat4f proj;
    if (c.centered) {
        float hw = (w * 0.5f) / zoom, hh = (h * 0.5f) / zoom;
        proj     = mat4f_ortho(-hw,  hw,       hh,       -hh,  -1.0f, 1.0f);
    } else proj  = mat4f_ortho(0.0f, w / zoom, h / zoom, 0.0f, -1.0f, 1.0f);

    /* view = R(-rot) * T(-pos) */
    float cs = cosf(-c.rot), sn = sinf(-c.rot);
    vec2f t = vec2f(
        -(c.pos.x * cs - c.pos.y * sn),
        -(c.pos.x * sn + c.pos.y * cs)
    );

    return mat4f_mul(proj, mat4f(
        vec4f( cs, sn,  0,  0),
        vec4f(-sn, cs,  0,  0),
        vec4f( 0,  0,   1,  0),
        vec4f( t.x, t.y, 0, 1)
    ));
}

mat4f render_trans3d_mat4(const trans3d *wt) {
    if (!wt) return mat4f_id();

    quatf q = (wt->qx == 0.0f && wt->qy == 0.0f && wt->qz == 0.0f && wt->qw == 0.0f)
              ? quatf_id()
              : quatf_norm(quatf(wt->qx, wt->qy, wt->qz, wt->qw));
    mat3f r = quatf_to_mat3f(q);

    vec3f c0 = vec_scale(r.col[0], wt->scale_x);
    vec3f c1 = vec_scale(r.col[1], wt->scale_y);
    vec3f c2 = vec_scale(r.col[2], wt->scale_z);

    return mat4f(
        vec4f(c0.x,  c0.y,  c0.z,  0),
        vec4f(c1.x,  c1.y,  c1.z,  0),
        vec4f(c2.x,  c2.y,  c2.z,  0),
        vec4f(wt->x, wt->y, wt->z, 1)
    );
}

/* ============================================================================================== */
/* >BATCH EXTRACTION                                                                              */
/* ============================================================================================== */

res_t render_build_batch(ecs_world_t *world, const asset_atlas_t *atlas, const mesh_registry_t *meshes,
                         const render_view_t *view, render_batch_t *out_batch) {
    ASSERT(out_batch, "render_build_batch called with NULL out_batch (world: %p, view: %p)", (void *)world, (void *)view);
    ASSERT(view,      "render_build_batch called with NULL view (world: %p, out_batch: %p)", (void *)world, (void *)out_batch);
    if   (!out_batch) return EINVAL;
    *out_batch = (render_batch_t){0};
    if   (!view || view->viewport.x <= 0 || view->viewport.y <= 0) return EINVAL;

    ecs_world_t *w = world ? world : ecs_get_world();
    if (!w)          return ENOTFND;
    if (!w->scratch) return EINVAL;

    ecs_world_t *prev = ecs_get_world();
    if (world) ecs_set_world(world);

    res_t res = OK;
    render_batch_t b = {
        .view_proj3d = render_cam3d_vp(&view->cam3d, view->viewport),
        .view_proj2d = render_cam2d_vp(&view->cam2d, view->viewport),
        .ambient     = view->ambient,
        .viewport    = view->viewport,
    };

    /* pass 1 lightning, extract up to RENDER_MAX_LIGHTS, directional first, then closest to eye */
    ecs_view_t vl = ecs_query_comp(trans3d, light3d);
    if (vl.cnt) {
        typedef struct _light_cand {
            render_light_gpu_t gpu;
            float              dist_sq;
        } _light_cand_t;

        arena_temp_t   mark  = arena_temp_begin(w->scratch);
        _light_cand_t *cands = arena_alloc(w->scratch, (size_t)vl.cnt * sizeof(_light_cand_t));
        if (!cands) { arena_temp_end(mark); res = EOOM; goto fail; }

        uint32_t cand_cnt = 0;
        for (uint32_t i = 0; i < vl.cnt; ++i) {
            entity_handle_t ent = vl.ents[i];
            const light3d  *lc  = ecs_get_fast(w, ent, light3d);

            trans3d wt;
            if (!ecs_get_world_trans(ent, &wt)) {
                const trans3d *t = ecs_get_fast(w, ent, trans3d);
                if (t) wt = *t; else continue;
            }

            quatf q   = (wt.qx == 0.0f && wt.qy == 0.0f && wt.qz == 0.0f && wt.qw == 0.0f)
                        ? quatf_id()
                        : quatf_norm(quatf(wt.qx, wt.qy, wt.qz, wt.qw));
            vec3f dir = quatf_mul_vec3(q, vec3f(0, 0, -1));
            dir       = vec_len_sq(dir) > 1e-8f ? vec_norm(dir) : vec3f(0, 0, -1);

            vec3f pos = vec3f(wt.x, wt.y, wt.z);

            float r_col = ((lc->color >> 24) & 0xFFU) / 255.0f;
            float g_col = ((lc->color >> 16) & 0xFFU) / 255.0f;
            float b_col = ((lc->color >>  8) & 0xFFU) / 255.0f;

            render_light_gpu_t gpu = {
                .pos_range = vec4f(pos.x, pos.y, pos.z, lc->type == LIGHT_DIR ? 0.0f : MAX(0.0f, lc->range)),
                .dir_type  = vec4f(dir.x, dir.y, dir.z, (float)lc->type),
                .color     = vec_scale(vec4f(r_col, g_col, b_col, 1), lc->intensity),
                .cone      = vec4f(cosf(lc->cone_in), cosf(lc->cone_out), 0, 0),
            };

            float d2 = vec_len_sq(vec_sub(pos, view->cam3d.pos));

            cands[cand_cnt++] = (_light_cand_t){
                .gpu     = gpu,
                .dist_sq = lc->type == LIGHT_DIR ? -1.0f : d2,
            };
        }

        /* insert sort - dir lights first, then closest to cam */
        for (uint32_t i = 1; i < cand_cnt; ++i) {
            _light_cand_t key = cands[i];
            int32_t       j   = (int32_t)i - 1;
            while (j >= 0 && (cands[j].dist_sq > key.dist_sq)) { cands[j + 1] = cands[j]; --j; }
            cands[j + 1] = key;
        }

        uint32_t take = MIN(cand_cnt, RENDER_MAX_LIGHTS);
        for (uint32_t i = 0; i < take; ++i) b.lights[i] = cands[i].gpu;
        b.light_cnt = take;

        arena_temp_end(mark);
    }

    /* pass 1 (3D opaque meshes) */
    ecs_view_t v3 = ecs_query_comp(trans3d, mesh3d);
    if (v3.cnt) {
        if (!meshes || !mesh_get(meshes, MESH_ID_CUBE)) { res = EINVAL; goto fail; }

        b.cmds3d = arena_alloc(w->scratch, (size_t)v3.cnt * sizeof(render_cmd3d_t));
        if (!b.cmds3d) { res = EOOM; goto fail; }

        for (uint32_t i = 0; i < v3.cnt; ++i) {
            entity_handle_t ent = v3.ents[i];
            const mesh3d   *mc  = ecs_get_fast(w, ent, mesh3d);

            /* TODO: replace per-entity parent walk with cached top-down world transforms */
            trans3d wt;
            if (!ecs_get_world_trans(ent, &wt)) {
                const trans3d *t = ecs_get_fast(w, ent, trans3d);
                if (t) wt = *t; else continue;
            }

            const mesh_t *m     = mesh_get(meshes, mc->mesh_id);
            mat4f         model = render_trans3d_mat4(&wt);
            aabb2f        uv    = aabb2f(0, 0, 1, 1);
            if (atlas)    uv    = asset_get_uv(atlas, mc->tex_id);

            b.cmds3d[b.cmd3d_cnt++] = (render_cmd3d_t){
                .model    = model,
                .norm_mat = mat3f_normal(model),
                .uv_rect  = uv,
                .mesh     = m,
                .ent      = ent,
                .sort_key = ((uint32_t)m->id << 16) | (uint32_t)mc->tex_id,
                .color    = mc->color,
            };
        }

        if (b.cmd3d_cnt > 1U) {
            arena_temp_t    tmp_mark = arena_temp_begin(w->scratch);
            render_cmd3d_t *tmp      = arena_alloc(w->scratch, (size_t)b.cmd3d_cnt * sizeof(render_cmd3d_t));
            if (!tmp) { arena_temp_end(tmp_mark); res = EOOM; goto fail; }

            _render_sort_cmds3d(b.cmds3d, tmp, b.cmd3d_cnt);
            arena_temp_end(tmp_mark);
        }
    }

    /* pass 2 (2D overlay sprites, stable counting sort by layer) */
    ecs_view_t v2 = ecs_query_comp(trans2d, sprite2d);
    if (v2.cnt) {
        if (!atlas || !asset_get_sprite(atlas, ASSET_SPRITE_CHECKER)) { res = EINVAL; goto fail; }

        b.sprites2d   = arena_alloc(w->scratch, (size_t)v2.cnt * sizeof(render_sprite_inst_t));
        b.sprite_ents = arena_alloc(w->scratch, (size_t)v2.cnt * sizeof(entity_handle_t));
        if (!b.sprites2d || !b.sprite_ents) { res = EOOM; goto fail; }

        uint32_t offs[RENDER_LAYER_CNT + 1U] = {0};
        for (uint32_t i = 0; i < v2.cnt; ++i)
            offs[(uint32_t)ecs_get_fast(w, v2.ents[i], sprite2d)->layer + 1U]++;
        for (uint32_t l = 1; l <= RENDER_LAYER_CNT; ++l) offs[l] += offs[l - 1U];

        for (uint32_t i = 0; i < v2.cnt; ++i) {
            entity_handle_t ent = v2.ents[i];
            const sprite2d *sc  = ecs_get_fast(w, ent, sprite2d);

            trans2d wt;
            if (!ecs_get_world_trans(ent, &wt)) {
                const trans2d *t = ecs_get_fast(w, ent, trans2d);
                if (t) wt = *t; else continue;
            }

            const asset_sprite_t *sp  = _render_resolve_sprite(atlas, sc->id);
            vec2i                 psz = aabb_size(sp->px);
            aabb2f                uv  = sp->uv;
            if (sc->flip_mask & RENDER_FLIP_H) SWAP(uv.min.x, uv.max.x);
            if (sc->flip_mask & RENDER_FLIP_V) SWAP(uv.min.y, uv.max.y);

            uint32_t dst = offs[sc->layer]++;
            b.sprite_ents[dst] = ent;
            b.sprites2d[dst] = (render_sprite_inst_t){
                .pos   = vec2f(wt.x, wt.y),
                .size  = vec_scale(vec2f(psz), wt.scale),
                .pivot = sp->pivot,
                .uv    = uv,
                .rot   = wt.rot,
                .color = sc->color,
            };
        }
        b.sprite2d_cnt = v2.cnt;
    }

    if (world) ecs_set_world(prev);
    *out_batch = b;
    return OK;

fail:
    if (world) ecs_set_world(prev);
    return res;
}

void render_sprite_quad_verts(const render_sprite_inst_t *inst, vec2f out_corners[4]) {
    ASSERT(inst,        "render_sprite_quad_verts called with NULL inst");
    ASSERT(out_corners, "render_sprite_quad_verts called with NULL out_corners");

    float x0 = -inst->pivot.x * inst->size.x, y0 = -inst->pivot.y * inst->size.y;
    float x1 = x0 + inst->size.x,             y1 = y0 + inst->size.y;

    vec2f local[4] = { vec2f(x0, y0), vec2f(x1, y0), vec2f(x1, y1), vec2f(x0, y1) };

    float c = cosf(inst->rot), s = sinf(inst->rot);
    for (uint8_t i = 0; i < 4; ++i)
        out_corners[i] = vec_add(inst->pos, vec2f(local[i].x * c - local[i].y * s,
                                                  local[i].x * s + local[i].y * c));
}
