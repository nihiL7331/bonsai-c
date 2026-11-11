/*
====================
mesh.c

3D mesh registry, Wavefront .obj parser,
and procedural primitive implementation.
====================
*/

#include "mesh.h"
#include "core.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================================================== */
/* >INTERNAL HELPERS                                                                              */
/* ============================================================================================== */

static inline aabb3f _mesh_calc_bounds(const mesh_vert_t *verts, uint32_t vert_cnt) {
    if (!verts || !vert_cnt) return aabb3f(vec3f_splat(0), vec3f_splat(0));

    vec3f min_p = verts[0].pos, max_p = verts[0].pos;
    for (uint32_t i = 1; i < vert_cnt; ++i) {
        min_p = vec_min(min_p, verts[i].pos);
        max_p = vec_max(max_p, verts[i].pos);
    }

    return aabb3f(min_p, max_p);
}

static inline res_t _mesh_alloc_slot(mesh_registry_t *reg, uint32_t vert_cnt, uint32_t idx_cnt, mesh_t **out_mesh) {
    if (reg->mesh_cnt >= reg->max_meshes) return EFULL;

    size_t       mark  = reg->arena->off;
    mesh_vert_t *verts = arena_alloc(reg->arena, (size_t)vert_cnt * sizeof(mesh_vert_t));
    uint32_t    *idxs  = arena_alloc(reg->arena, (size_t)idx_cnt  * sizeof(uint32_t));
    if (!verts || !idxs) { reg->arena->off = mark; return EOOM; }

    if (mark != reg->tail_off) reg->base_arena_off = mark;
    reg->tail_off = reg->arena->off;

    mesh_id_t id = (mesh_id_t)reg->mesh_cnt++;
    mesh_t   *m  = &reg->meshes[id];
    *m = (mesh_t){
        .verts    = verts,
        .indices  = idxs,
        .vert_cnt = vert_cnt,
        .idx_cnt  = idx_cnt,
        .id       = id,
    };

    *out_mesh = m;
    return OK;
}

/* ============================================================================================== */
/* >OBJ PARSER HELPERS                                                                            */
/* ============================================================================================== */

/* locale-independent equivalent of strtof used to parse float values in .obj files */
static inline float _obj_strtof(const char *s, char **end) {
    if (end) *end = (char *)s;
    while (*s == ' ' || *s == '\t') s++;
    float sgn = (*s == '-') ? -1.0f : 1.0f;
    if (*s == '-' || *s == '+') s++;

    uint32_t m = 0;
    int32_t  e = 0, n = 0;
    for (; *s >= '0' && *s <= '9'; s++, n++)
        if (m < 100000000U) m = m * 10U + (uint32_t)(*s - '0'); else e++;

    if (*s == '.')
        for (s++; *s >= '0' && *s <= '9'; ++s, ++n)
            if (m < 100000000U) { m = m * 10U + (uint32_t)(*s - '0'); e--; }
    if (!n) return 0.0f;

    if ((*s | 32) == 'e') { /* exponent */
        const char *q  = s + 1;
        int32_t     en = (*q == '-'), x = 0, k = 0;
        if (*q == '-' || *q == '+') q++;

        for (; *q >= '0' && *q <= '9'; ++q, ++k)
            if (x < 1000) x = x * 10 + (*q - '0');

        if (k) {e += en ? -x : x; s = q; }
    }

    float v = (float)m;
    if (m) {
        float d = 1.0f;
        for (int32_t i = ABS(e); i > 0; --i) d *= 10.0f;
        v = e < 0 ? v / d : v * d;
    }

    if (end) *end = (char *)s;
    return sgn * v;
}

static inline const char *_obj_skip_ws(const char *p, const char *end) {
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\r')) p++;
    return p;
}

static inline const char *_obj_next_line(const char *p, const char *end) {
    while (p < end && *p != '\n') p++;
    if (p < end && *p == '\n')    p++;
    return p;
}

static inline bool _obj_parse_float(const char **io_p, const char *end, float *out_val) {
    const char *p = _obj_skip_ws(*io_p, end);
    if (p >= end || *p == '\n' || *p == '#') return false;

    char buf[64];
    size_t len = 0;

    while (p + len < end && len < sizeof(buf) - 1U) {
        char c = p[len];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '#') break;

        buf[len++] = c;
    }

    if (!len) return false;
    buf[len] = '\0';

    char *endptr = NULL;
    *out_val     = _obj_strtof(buf, &endptr);
    if (endptr == buf) return false;

    *io_p = p + len;
    return true;
}

static inline int32_t _obj_parse_int(const char **io_p, const char *end) {
    const char *p   = *io_p;
    int32_t     sgn = 1;
    if      (p < end && *p == '-') { sgn = -1; p++; }
    else if (p < end && *p == '+') { sgn = 1;  p++; }

    int32_t val = 0;
    while (p < end && *p >= '0' && *p <= '9') {
        if (val > (INT32_MAX - 9) / 10) val = INT32_MAX;
        else                            val = val * 10 + (int32_t)(*p - '0');
        p++;
    }

    *io_p = p;
    return val * sgn;
}

typedef struct obj_face_vert {
    int32_t v;
    int32_t vt;
    int32_t vn;
} obj_face_vert_t;

static inline bool _obj_parse_face_token(const char **io_p, const char *end, obj_face_vert_t *out_fv) {
    const char *p = _obj_skip_ws(*io_p, end);
    if (p >= end || *p == '\n' || *p == '#') return false;
    if (*p != '-' && (*p < '0' || *p > '9')) return false;

    *out_fv   = (obj_face_vert_t){0};
    out_fv->v = _obj_parse_int(&p, end);
    if (out_fv->v == 0) return false;

    if (p < end && *p == '/') {
        p++;
        if (p < end && (*p == '-' || (*p >= '0' && *p <= '9')))
            out_fv->vt = _obj_parse_int(&p, end);
        if (p < end && *p == '/') {
            p++;
            if (p < end && (*p == '-' || (*p >= '0' && *p <= '9')))
                out_fv->vn = _obj_parse_int(&p, end);
        }
    }

    *io_p = p;
    return true;
}

static inline bool _obj_resolve_idx(int32_t raw_idx, uint32_t cur_cnt, uint32_t *out_idx) {
    int32_t idx = (raw_idx > 0) ? (raw_idx - 1) : ((int32_t)cur_cnt + raw_idx);
    if (idx < 0 || (uint32_t)idx >= cur_cnt) return false;

    *out_idx = (uint32_t)idx;
    return true;
}

/* ============================================================================================== */
/* >LIFECYCLE                                                                                     */
/* ============================================================================================== */

res_t mesh_registry_init(mesh_registry_t *reg, arena_t *perm, uint32_t max_meshes) {
    ASSERT(reg,  "mesh_registry_init called with NULL reg (perm: %p, max_meshes: %" PRIu32 ")", (void *)perm, max_meshes);
    ASSERT(perm, "mesh_registry_init called with NULL perm (reg: %p, max_meshes: %" PRIu32 ")", (void *)reg, max_meshes);
    if (!reg || !perm) return EINVAL;

    uint32_t cap  = max_meshes ? MIN(max_meshes, (uint32_t)MESH_ID_NULL) : MESH_DEF_MAX;
    size_t   mark = perm->off;

    mesh_t *meshes = arena_alloc(perm, (size_t)cap * sizeof(mesh_t));
    if (!meshes) return EOOM;
    memset(meshes, 0, (size_t)cap * sizeof(mesh_t));

    *reg = (mesh_registry_t){
        .arena          = perm,
        .meshes         = meshes,
        .max_meshes     = cap,
        .mesh_cnt       = 0,
    };

    mesh_id_t cube_id = MESH_ID_NULL;
    res_t res = mesh_gen_box(reg, vec3f_splat(1), &cube_id);
    if (res != OK || cube_id != MESH_ID_CUBE) {
        perm->off = mark;
        *reg      = (mesh_registry_t){0};
        return (res != OK) ? res : EINVAL;
    }

    reg->base_arena_off = reg->tail_off = perm->off;
    return OK;
}

res_t mesh_registry_reset(mesh_registry_t *reg) {
    ASSERT(reg, "mesh_registry_reset called with NULL reg");
    if (!reg || !reg->arena || !reg->meshes || !reg->mesh_cnt) return EINVAL;

    if (reg->max_meshes > 1U)
        memset(&reg->meshes[1], 0, (size_t)(reg->max_meshes - 1U) * sizeof(mesh_t));

    if (reg->arena->off == reg->tail_off) reg->arena->off     = reg->base_arena_off;
    else                                  reg->base_arena_off = reg->arena->off;
    reg->tail_off = reg->base_arena_off;
    reg->mesh_cnt = 1U;
    return OK;
}

res_t mesh_register(mesh_registry_t *reg, const mesh_vert_t *verts, uint32_t vert_cnt, const uint32_t *indices, uint32_t idx_cnt, mesh_id_t *out_id) {
    ASSERT(reg,     "mesh_register called with NULL reg (verts: %p, vert_cnt: %" PRIu32 ", indices: %p, idx_cnt: %" PRIu32 ", out_id: %p)", (void *)verts, vert_cnt, (void *)indices, idx_cnt, (void *)out_id);
    ASSERT(verts,   "mesh_register called with NULL verts (reg: %p, vert_cnt: %" PRIu32 ", indices: %p, idx_cnt: %" PRIu32 ", out_id: %p)", (void *)reg, vert_cnt, (void *)indices, idx_cnt, (void *)out_id);
    ASSERT(indices, "mesh_register called with NULL indices (reg: %p, verts: %p, vert_cnt: %" PRIu32 ", idx_cnt: %" PRIu32 ", out_id: %p)", (void *)reg, (void *)verts, vert_cnt, idx_cnt, (void *)out_id);
    ASSERT(out_id,  "mesh_register called with NULL out_id (reg: %p, verts: %p, vert_cnt: %" PRIu32 ", indices: %p, idx_cnt: %" PRIu32 ")", (void *)reg, (void *)verts, vert_cnt, (void *)indices, idx_cnt);
    if   (!reg || !reg->arena || !reg->meshes || !verts || !indices || !out_id) return EINVAL;
    if   (!vert_cnt || !idx_cnt || (idx_cnt % 3U))                              return EINVAL;
    for (uint32_t i = 0; i < idx_cnt; ++i) if (indices[i] >= vert_cnt)          return EINVAL;

    mesh_t *m   = NULL;
    res_t   res = _mesh_alloc_slot(reg, vert_cnt, idx_cnt, &m);
    if (res != OK) return res;

    memcpy(m->verts,   verts,   (size_t)vert_cnt * sizeof(mesh_vert_t));
    memcpy(m->indices, indices, (size_t)idx_cnt  * sizeof(uint32_t));
    m->bounds = _mesh_calc_bounds(verts, vert_cnt);
    *out_id   = m->id;

    return OK;
}

/* ============================================================================================== */
/* >WAVEFRONT .OBJ                                                                                */
/* ============================================================================================== */

res_t mesh_load_obj_mem(mesh_registry_t *reg, const char *obj_str, size_t len, arena_t *scratch, mesh_id_t *out_id) {
    ASSERT(reg,                   "mesh_load_obj_mem called with NULL reg (obj_str: %p, len: %zu, scratch: %p, out_id: %p)", (void *)obj_str, len, (void *)scratch, (void *)out_id);
    ASSERT(obj_str,               "mesh_load_obj_mem called with NULL obj_str (reg: %p, len: %zu, scratch: %p, out_id: %p)", (void *)reg, len, (void *)scratch, (void *)out_id);
    ASSERT(scratch,               "mesh_load_obj_mem called with NULL scratch (reg: %p, obj_str: %p, len: %zu, out_id: %p)", (void *)reg, (void *)obj_str, len, (void *)out_id);
    ASSERT(out_id,                "mesh_load_obj_mem called with NULL out_id (reg: %p, obj_str: %p, len: %zu, scratch: %p)", (void *)reg, (void *)obj_str, len, (void *)scratch);
    ASSERT(scratch != reg->arena, "mesh_load_obj_mem called with scratch == reg->arena");
    if   (!reg || !reg->arena || !reg->meshes || !obj_str || !scratch || !out_id || !len || scratch == reg->arena) return EINVAL;
    if   (reg->mesh_cnt >= reg->max_meshes)                                                                        return EFULL;

    const char *end   = obj_str + len;
    uint32_t    v_cap = 0, vt_cap = 0, vn_cap = 0, tri_cnt = 0;

    /* count attributes and triangles */
    for (const char *p = obj_str; p < end; p = _obj_next_line(p, end)) {
        const char *line = _obj_skip_ws(p, end);
        if (line >= end || *line == '\n' || *line == '#') continue;

        if (line[0] == 'v' && line + 1 < end) {
            char c1 = line[1];
            if      (c1 == ' ' || c1 == '\t')                                             v_cap++;
            else if (c1 == 't' && line + 2 < end && (line[2] == ' ' || line[2] == '\t')) vt_cap++;
            else if (c1 == 'n' && line + 2 < end && (line[2] == ' ' || line[2] == '\t')) vn_cap++;
        } else if (line[0] == 'f' && line + 1 < end && (line[1] == ' ' || line[1] == '\t')) {
            const char      *fp  = line + 2;
            uint32_t        fcnt = 0;
            obj_face_vert_t fv;
            while (_obj_parse_face_token(&fp, end, &fv)) fcnt++;
            if (fcnt >= 3U) tri_cnt += (fcnt - 2U);
        }
    }

    if (!v_cap || !tri_cnt || tri_cnt > UINT32_MAX / 3U) return ENOTFND;

    arena_temp_t tmp    = arena_temp_begin(scratch);
    vec3f       *v_buf  =          arena_alloc(scratch, (size_t)v_cap  * sizeof(vec3f));
    vec2f       *vt_buf = vt_cap ? arena_alloc(scratch, (size_t)vt_cap * sizeof(vec2f)) : NULL;
    vec3f       *vn_buf = vn_cap ? arena_alloc(scratch, (size_t)vn_cap * sizeof(vec3f)) : NULL;
    if (!v_buf || (vt_cap && !vt_buf) || (vn_cap && !vn_buf)) { arena_temp_end(tmp); return EOOM; }

    uint32_t out_vert_cnt = tri_cnt * 3U;
    uint32_t out_idx_cnt  = tri_cnt * 3U;
    size_t   perm_mark    = reg->arena->off;
    mesh_t  *m            = NULL;
    res_t    res          = _mesh_alloc_slot(reg, out_vert_cnt, out_idx_cnt, &m);
    if (res != OK) { arena_temp_end(tmp); return res; }

    uint32_t v_cnt = 0, vt_cnt = 0, vn_cnt = 0, emit_idx = 0;

    /* parse attributes and triangulate faces */
    for (const char *p = obj_str; p < end; p = _obj_next_line(p, end)) {
        const char *line = _obj_skip_ws(p, end);
        if (line >= end || *line == '\n' || *line == '#') continue;

        if (line[0] == 'v' && line + 1 < end) {
            char c1 = line[1];
            if (c1 == ' ' || c1 == '\t') {
                const char *vp = line + 2;
                vec3f v = vec3f_splat(0);
                if (!_obj_parse_float(&vp, end, &v.x) || !_obj_parse_float(&vp, end, &v.y) || !_obj_parse_float(&vp, end, &v.z)) goto fail_parse;

                v_buf[v_cnt++] = v;
            } else if (c1 == 't' && line + 2 < end && (line[2] == ' ' || line[2] == '\t')) {
                const char *vp = line + 3;
                vec2f vt = vec2f_splat(0);
                if (!_obj_parse_float(&vp, end, &vt.x) || !_obj_parse_float(&vp, end, &vt.y)) goto fail_parse;

                vt.y = 1.0f - vt.y; /* OBJ puts v = 0 at the bottom, we assume v = 0 at the top */
                vt_buf[vt_cnt++] = vt;
            } else if (c1 == 'n' && line + 2 < end && (line[2] == ' ' || line[2] == '\t')) {
                const char *vp = line + 3;
                vec3f vn = vec3f_splat(0);
                if (!_obj_parse_float(&vp, end, &vn.x) || !_obj_parse_float(&vp, end, &vn.y) || !_obj_parse_float(&vp, end, &vn.z)) goto fail_parse;

                vn_buf[vn_cnt++] = vec_norm(vn);
            }
        } else if (line[0] == 'f' && line + 1 < end && (line[1] == ' ' || line[1] == '\t')) {
            const char     *fp   = line + 2;
            uint32_t        fcnt = 0;
            obj_face_vert_t fvs[64];

            while (fcnt  < 64U && _obj_parse_face_token(&fp, end, &fvs[fcnt])) fcnt++;
            if    (fcnt  < 3U)                                                            goto fail_parse;
            if    (fcnt == 64U && _obj_parse_face_token(&fp, end, &(obj_face_vert_t){0})) goto fail_parse;

            for (uint32_t k = 1U; k + 1U < fcnt; ++k) {
                obj_face_vert_t tri[3] = { fvs[0], fvs[k], fvs[k + 1U] };
                vec3f           pos[3];
                vec2f           uv[3];
                vec3f           norm[3];
                bool            has_norm = true;

                for (uint32_t c = 0; c < 3U; ++c) {
                    uint32_t vi = 0;
                    if (!_obj_resolve_idx(tri[c].v, v_cnt, &vi)) goto fail_parse;

                    pos[c] = v_buf[vi];

                    if (tri[c].vt != 0) {
                        uint32_t vti = 0;
                        if (!_obj_resolve_idx(tri[c].vt, vt_cnt, &vti)) goto fail_parse;

                        uv[c] = vt_buf[vti];
                    } else uv[c] = vec2f_splat(0);

                    if (tri[c].vn != 0) {
                        uint32_t vni = 0;
                        if (!_obj_resolve_idx(tri[c].vn, vn_cnt, &vni)) goto fail_parse;

                        norm[c] = vn_buf[vni];
                    } else has_norm = false;
                }

                if (!has_norm) {
                    vec3f e1 = vec_sub(pos[1], pos[0]);
                    vec3f e2 = vec_sub(pos[2], pos[0]);
                    vec3f fn = vec_norm(vec_cross(e1, e2));
                    norm[0]  = norm[1] = norm[2] = fn;
                }

                for (uint32_t c = 0; c < 3U; ++c) {
                    m->verts[emit_idx] = (mesh_vert_t){
                        .pos  = pos[c],
                        .norm = norm[c],
                        .uv   = uv[c],
                    };
                    m->indices[emit_idx] = emit_idx;
                    emit_idx++;
                }
            }
        }
    }

    if (!emit_idx) goto fail_parse;

    m->vert_cnt = emit_idx;
    m->idx_cnt  = emit_idx;
    m->bounds   = _mesh_calc_bounds(m->verts, m->vert_cnt);
    *out_id     = m->id;

    arena_temp_end(tmp);
    return OK;

fail_parse:
    reg->mesh_cnt--;
    reg->arena->off = perm_mark;
    arena_temp_end(tmp);

    return ENOTFND;
}

res_t mesh_load_obj_file(mesh_registry_t *reg, const char *path, arena_t *scratch, mesh_id_t *out_id) {
    ASSERT(reg,                   "mesh_load_obj_file called with NULL reg (path: %p, scratch: %p, out_id: %p)", (void *)path, (void *)scratch, (void *)out_id);
    ASSERT(path,                  "mesh_load_obj_file called with NULL path (reg: %p, scratch: %p, out_id: %p)", (void *)reg, (void *)scratch, (void *)out_id);
    ASSERT(scratch,               "mesh_load_obj_file called with NULL scratch (reg: %p, path: %p, out_id: %p)", (void *)reg, (void *)path, (void *)out_id);
    ASSERT(out_id,                "mesh_load_obj_file called with NULL out_id (reg: %p, path: %p, scratch: %p)", (void *)reg, (void *)path, (void *)scratch);
    ASSERT(scratch != reg->arena, "mesh_load_obj_file called with scratch == reg->arena");
    if   (!reg || !path || !scratch || !out_id || scratch == reg->arena) return EINVAL;

    FILE *fp = fopen(path, "rb");
    if (!fp) return ENOTFND;

    if (fseek(fp, 0, SEEK_END) != 0) { fclose(fp); return ENOTFND; }
    long fsz = ftell(fp);
    if (fsz <= 0 || fseek(fp, 0, SEEK_SET) != 0) { fclose(fp); return ENOTFND; }

    arena_temp_t tmp = arena_temp_begin(scratch);
    char        *buf = arena_alloc(scratch, (size_t)fsz + 1U);
    if (!buf) { arena_temp_end(tmp); fclose(fp); return EOOM; }

    size_t read_sz = fread(buf, 1U, (size_t)fsz, fp);
    fclose(fp);
    if (read_sz != (size_t)fsz) { arena_temp_end(tmp); return ENOTFND; }
    buf[read_sz] = '\0';

    res_t res = mesh_load_obj_mem(reg, buf, read_sz, scratch, out_id);
    arena_temp_end(tmp);

    return res;
}

/* ============================================================================================== */
/* >PROCEDURAL 3D PRIMITIVES                                                                      */
/* ============================================================================================== */

res_t mesh_gen_box(mesh_registry_t *reg, vec3f size, mesh_id_t *out_id) {
    ASSERT(reg,    "mesh_gen_box called with NULL reg (size: " FMT_VEC3F ", out_id: %p)", ARG_VEC3F(size), (void *)out_id);
    ASSERT(out_id, "mesh_gen_box called with NULL out_id (reg: %p, size: " FMT_VEC3F ")", (void *)reg, ARG_VEC3F(size));
    ASSERT(size.x > 0.0f && size.y > 0.0f && size.z > 0.0f, "mesh_gen_box requires positive size, got " FMT_VEC3F, ARG_VEC3F(size));
    if   (!reg || !reg->arena || !reg->meshes || !out_id)     return EINVAL;
    if   (size.x <= 0.0f || size.y <= 0.0f || size.z <= 0.0f) return EINVAL;

    mesh_t *m   = NULL;
    res_t   res = _mesh_alloc_slot(reg, 24U, 36U, &m);
    if     (res != OK) return res;

    vec3f h = vec_scale(size, 0.5f);

    /* 6 faces, outward normal N and face-right axis U (with V = N x U so U x V = N) */
    const vec3f norms[6] = {
        vec3f( 0,  0,  1), /* +Z front  */
        vec3f( 0,  0, -1), /* -Z back   */
        vec3f( 1,  0,  0), /* +X right  */
        vec3f(-1,  0,  0), /* -X left   */
        vec3f( 0,  1,  0), /* +Y top    */
        vec3f( 0, -1,  0), /* -Y bottom */
    };
    const vec3f axis_u[6] = {
        vec3f( 1, 0,  0),
        vec3f(-1, 0,  0),
        vec3f( 0, 0, -1),
        vec3f( 0, 0,  1),
        vec3f( 1, 0,  0),
        vec3f( 1, 0,  0),
    };

    for (uint32_t f = 0; f < 6U; ++f) {
        vec3f n  = norms[f];
        vec3f u  = axis_u[f];
        vec3f v  = vec_cross(n, u);
        vec3f c  = vec_mul(n, h);
        vec3f hu = vec_mul(u, h);
        vec3f hv = vec_mul(v, h);

        uint32_t base_v = f * 4U;
        uint32_t base_i = f * 6U;

        m->verts[base_v + 0U] = (mesh_vert_t){ .pos = vec_sub(vec_sub(c, hu), hv), .norm = n, .uv = vec2f(0, 1) };
        m->verts[base_v + 1U] = (mesh_vert_t){ .pos = vec_sub(vec_add(c, hu), hv), .norm = n, .uv = vec2f(1, 1) };
        m->verts[base_v + 2U] = (mesh_vert_t){ .pos = vec_add(c, hu, hv),          .norm = n, .uv = vec2f(1, 0) };
        m->verts[base_v + 3U] = (mesh_vert_t){ .pos = vec_add(vec_sub(c, hu), hv), .norm = n, .uv = vec2f(0, 0) };

        m->indices[base_i + 0U] = base_v + 0U;
        m->indices[base_i + 1U] = base_v + 1U;
        m->indices[base_i + 2U] = base_v + 2U;
        m->indices[base_i + 3U] = base_v + 0U;
        m->indices[base_i + 4U] = base_v + 2U;
        m->indices[base_i + 5U] = base_v + 3U;
    }

    m->bounds = aabb_center_half(vec3f_splat(0), h);
    *out_id   = m->id;

    return OK;
}

res_t mesh_gen_plane(mesh_registry_t *reg, vec2f size, uint32_t subdiv_x, uint32_t subdiv_z, mesh_id_t *out_id) {
    ASSERT(reg,                            "mesh_gen_plane called with NULL reg (size: " FMT_VEC2F ", subdiv_x: %" PRIu32 ", subdiv_z: %" PRIu32 ", out_id: %p)", ARG_VEC2F(size), subdiv_x, subdiv_z, (void *)out_id);
    ASSERT(out_id,                         "mesh_gen_plane called with NULL out_id (reg: %p, size: " FMT_VEC2F ", subdiv_x: %" PRIu32 ", subdiv_z: %" PRIu32 ")", (void *)reg, ARG_VEC2F(size), subdiv_x, subdiv_z);
    ASSERT(size.x > 0.0f && size.y > 0.0f, "mesh_gen_plane requires positive size, got " FMT_VEC2F, ARG_VEC2F(size));
    if (!reg || !reg->arena || !reg->meshes || !out_id || size.x <= 0.0f || size.y <= 0.0f) return EINVAL;

    uint32_t sx = subdiv_x ? subdiv_x : 1U;
    uint32_t sz = subdiv_z ? subdiv_z : 1U;
    uint32_t vx = sx + 1U;
    uint32_t vz = sz + 1U;

    mesh_t *m   = NULL;
    res_t   res = _mesh_alloc_slot(reg, vx * vz, sx * sz * 6U, &m);
    if (res != OK) return res;

    float hx = size.x * 0.5f;
    float hz = size.y * 0.5f;
    vec3f up = vec3f(0, 1, 0);

    for (uint32_t iz = 0; iz < vz; ++iz) {
        float tz = (float)iz / (float)sz;
        float z  = -hz + tz * size.y;

        for (uint32_t ix = 0; ix < vx; ++ix) {
            float tx = (float)ix / (float)sx;
            float x  = -hx + tx * size.x;

            m->verts[iz * vx + ix] = (mesh_vert_t){
                .pos  = vec3f(x, 0, z),
                .norm = up,
                .uv   = vec2f(tx, tz),
            };
        }
    }

    uint32_t idx = 0;
    for (uint32_t iz = 0; iz < sz; ++iz)
        for (uint32_t ix = 0; ix < sx; ++ix) {
            uint32_t i00 = iz * vx + ix;
            uint32_t i10 = i00 + 1U;
            uint32_t i01 = (iz + 1U) * vx + ix;
            uint32_t i11 = i01 + 1U;

            m->indices[idx++] = i00;
            m->indices[idx++] = i01;
            m->indices[idx++] = i11;
            m->indices[idx++] = i00;
            m->indices[idx++] = i11;
            m->indices[idx++] = i10;
        }

    m->bounds = aabb3f(-hx, 0, -hz, hx, 0, hz);
    *out_id   = m->id;

    return OK;
}

res_t mesh_gen_sphere(mesh_registry_t *reg, float radius, uint32_t slices, uint32_t stacks, mesh_id_t *out_id) {
    ASSERT(reg,           "mesh_gen_sphere called with NULL reg (radius: %.2f, slices: %" PRIu32 ", stacks: %" PRIu32 ", out_id: %p)", radius, slices, stacks, (void *)out_id);
    ASSERT(out_id,        "mesh_gen_sphere called with NULL out_id (reg: %p, radius: %.2f, slices: %" PRIu32 ", stacks: %" PRIu32 ")", (void *)reg, radius, slices, stacks);
    ASSERT(radius > 0.0f, "mesh_gen_sphere requires radius > 0.0f (reg: %p, radius: %.2f, slices: %" PRIu32 ", stacks: %" PRIu32 ", out_id: %p)", (void *)reg, radius, slices, stacks, (void *)out_id);
    if (!reg || !reg->arena || !reg->meshes || !out_id || radius <= 0.0f) return EINVAL;

    uint32_t sl = slices ? MAX(slices, 3U) : MESH_DEF_SLICES;
    uint32_t st = stacks ? MAX(stacks, 2U) : MESH_DEF_STACKS;

    uint32_t vert_cnt = (st + 1U) * (sl + 1U);
    uint32_t idx_cnt  = st * sl * 6U;

    mesh_t *m   = NULL;
    res_t   res = _mesh_alloc_slot(reg, vert_cnt, idx_cnt, &m);
    if     (res != OK) return res;

    uint32_t v = 0;
    for (uint32_t i = 0; i <= st; ++i) {
        float tv    = (float)i / (float)st;
        float phi   = tv       * GMATH_PI;
        float sin_p = sinf(phi);
        float cos_p = cosf(phi);

        for (uint32_t j = 0; j <= sl; ++j) {
            float tu    = (float)j / (float)sl;
            float theta = tu       * GMATH_TAU;
            vec3f norm  = vec3f(sin_p * sinf(theta), cos_p, sin_p * cosf(theta));

            m->verts[v++] = (mesh_vert_t){
                .pos  = vec_scale(norm, radius),
                .norm = norm,
                .uv   = vec2f(tu, tv),
            };
        }
    }

    uint32_t idx = 0, row = sl + 1U;
    for (uint32_t i = 0; i < st; ++i)
        for (uint32_t j = 0; j < sl; ++j) {
            uint32_t i00 = i * row + j;
            uint32_t i10 = i00 + 1U;
            uint32_t i01 = (i + 1U) * row + j;
            uint32_t i11 = i01 + 1U;

            m->indices[idx++] = i00;
            m->indices[idx++] = i01;
            m->indices[idx++] = i11;
            m->indices[idx++] = i00;
            m->indices[idx++] = i11;
            m->indices[idx++] = i10;
        }

    m->bounds = _mesh_calc_bounds(m->verts, m->vert_cnt);
    *out_id   = m->id;

    return OK;
}

res_t mesh_gen_cylinder(mesh_registry_t *reg, float radius, float height, uint32_t slices, mesh_id_t *out_id) {
    ASSERT(reg,                            "mesh_gen_cylinder called with NULL reg (radius: %.2f, height: %.2f, slices: %" PRIu32 ", out_id: %p)", radius, height, slices, (void *)out_id);
    ASSERT(out_id,                         "mesh_gen_cylinder called with NULL out_id (reg: %p, radius: %.2f, height: %.2f, slices: %" PRIu32 ")", (void *)reg, radius, height, slices);
    ASSERT(radius > 0.0f && height > 0.0f, "mesh_gen_cylinder requires positive radius and height, got radius: %.2f, height: %.2f", radius, height);
    if (!reg || !reg->arena || !reg->meshes || !out_id || radius <= 0.0f || height <= 0.0f) return EINVAL;

    uint32_t sl = slices ? MAX(slices, 3U) : MESH_DEF_SLICES;

    /* side: 2 * (sl + 1) verts, 6 * sl indices
       top cap + bottom cap: 2 * (sl + 2) verts, 2 * 3 * sl indices */
    uint32_t vert_cnt = 2U * (sl + 1U) + 2U * (sl + 2U);
    uint32_t idx_cnt  = 12U * sl;

    mesh_t *m = NULL;
    res_t res = _mesh_alloc_slot(reg, vert_cnt, idx_cnt, &m);
    if (res != OK) return res;

    float    hy  = height * 0.5f;
    uint32_t v   = 0;
    uint32_t idx = 0;

    /* side barrel */
    for (uint32_t j = 0; j <= sl; ++j) {
        float tu    = (float)j / (float)sl;
        float theta = tu * GMATH_TAU;
        float sx    = sinf(theta);
        float cz    = cosf(theta);
        vec3f norm  = vec3f(sx, 0, cz);

        m->verts[v++] = (mesh_vert_t){ .pos = vec3f(sx * radius,  hy, cz * radius), .norm = norm, .uv = vec2f(tu, 0) };
        m->verts[v++] = (mesh_vert_t){ .pos = vec3f(sx * radius, -hy, cz * radius), .norm = norm, .uv = vec2f(tu, 1) };
    }

    for (uint32_t j = 0; j < sl; ++j) {
        uint32_t t0 = j * 2U;
        uint32_t b0 = t0 + 1U;
        uint32_t t1 = t0 + 2U;
        uint32_t b1 = t0 + 3U;

        m->indices[idx++] = t0;
        m->indices[idx++] = b0;
        m->indices[idx++] = b1;
        m->indices[idx++] = t0;
        m->indices[idx++] = b1;
        m->indices[idx++] = t1;
    }

    /* top cap (+Y) */
    uint32_t top_center = v;
    vec3f    up         = vec3f(0, 1, 0);
    m->verts[v++]       = (mesh_vert_t){ .pos = vec3f(0, hy, 0), .norm = up, .uv = vec2f(0.5f, 0.5f) };

    for (uint32_t j = 0; j <= sl; ++j) {
        float theta   = ((float)j / (float)sl) * GMATH_TAU;
        float sx      = sinf(theta);
        float cz      = cosf(theta);

        m->verts[v++] = (mesh_vert_t){
            .pos  = vec3f(sx * radius, hy, cz * radius),
            .norm = up,
            .uv   = vec2f(0.5f + sx * 0.5f, 0.5f + cz * 0.5f),
        };
    }

    for (uint32_t j = 0; j < sl; ++j) {
        m->indices[idx++] = top_center;
        m->indices[idx++] = top_center + 1U + j;
        m->indices[idx++] = top_center + 2U + j;
    }

    /* bottom cap (-Y) */
    uint32_t bot_center = v;
    vec3f    down       = vec3f(0, -1, 0);
    m->verts[v++]       = (mesh_vert_t){ .pos = vec3f(0, -hy, 0), .norm = down, .uv = vec2f(0.5f, 0.5f) };

    for (uint32_t j = 0; j <= sl; ++j) {
        float theta = ((float)j / (float)sl) * GMATH_TAU;
        float sx    = sinf(theta);
        float cz    = cosf(theta);

        m->verts[v++] = (mesh_vert_t){
            .pos  = vec3f(sx * radius, -hy, cz * radius),
            .norm = down,
            .uv   = vec2f(0.5f + sx * 0.5f, 0.5f - cz * 0.5f),
        };
    }

    for (uint32_t j = 0; j < sl; ++j) {
        m->indices[idx++] = bot_center;
        m->indices[idx++] = bot_center + 2U + j;
        m->indices[idx++] = bot_center + 1U + j;
    }

    m->bounds = _mesh_calc_bounds(m->verts, m->vert_cnt);
    *out_id   = m->id;
    return OK;
}

/* ============================================================================================== */
/* >QUERIES                                                                                       */
/* ============================================================================================== */

const mesh_t *mesh_get(const mesh_registry_t *reg, mesh_id_t id) {
    ASSERT(reg, "mesh_get called with NULL reg (id: %" PRIu16 ")", id);
    if   (!reg || !reg->meshes || !reg->mesh_cnt) return NULL;
    if   (id >= reg->mesh_cnt)                    return &reg->meshes[MESH_ID_CUBE];
    return &reg->meshes[id];
}
