/*
====================
assets.c

Texture atlas management, procedural canvas rasterization, and asset ingestion.
Implements:
- 2D skyline bin packing into a contiguous RGBA32 atlas bitmap
- Alpha over-compositing and anti-aliased procedural geometry
- Text pixel art DSL and multi-frame animation baking
- In-place asset hot reloading and atlas-wide palette remapping
====================
*/

#include "assets.h"
#include "core.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_BMP
#define STBI_ONLY_TGA
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

/* ============================================================================================== */
/* >PALETTE / COLOR MATH                                                                          */
/* ============================================================================================== */

/* built-in PICO-8 palette */
const uint32_t g_palette_pico8[16] = {
    0x000000FFU, /* black       */
    0x1D2B53FFU, /* dark blue   */
    0x7E2553FFU, /* dark purple */
    0x008751FFU, /* dark green  */
    0xAB5236FFU, /* brown       */
    0x5F574FFFU, /* dark gray   */
    0xC2C3C7FFU, /* gray        */
    0xFFF1E8FFU, /* white       */
    0xFF004DFFU, /* red         */
    0xFFA300FFU, /* orange      */
    0xFFEC27FFU, /* yellow      */
    0x00E436FFU, /* green       */
    0x29ADFFFFU, /* blue        */
    0x83769CFFU, /* indigo      */
    0xFF77A8FFU, /* pink        */
    0xFFCCAAFFU, /* peach       */
};

/* Porter-Duff source-over alpha compositing */
static inline uint32_t _asset_col_over(uint32_t dst, uint32_t src) {
    uint32_t sa = COLOR_A(src);
    if (sa == COLOR_CHAN_MAX) return src;
    if (sa == COLOR_CHAN_MIN) return dst;

    uint32_t da     = COLOR_A(dst);
    uint32_t inv_sa = COLOR_CHAN_MAX - sa;

    uint32_t sr = COLOR_R(src), sg = COLOR_G(src), sb = COLOR_B(src);
    uint32_t dr = COLOR_R(dst), dg = COLOR_G(dst), db = COLOR_B(dst);

    uint32_t out_a = sa + ((da * inv_sa + COLOR_CHAN_MAX / 2) / COLOR_CHAN_MAX);
    if (out_a >= COLOR_CHAN_MAX) out_a = COLOR_CHAN_MAX;
    if (out_a == COLOR_CHAN_MIN) return  COLOR_CLEAR;

    uint32_t out_r = (sr * sa + ((dr * da * inv_sa + COLOR_CHAN_MAX / 2) / COLOR_CHAN_MAX)) / out_a;
    uint32_t out_g = (sg * sa + ((dg * da * inv_sa + COLOR_CHAN_MAX / 2) / COLOR_CHAN_MAX)) / out_a;
    uint32_t out_b = (sb * sa + ((db * da * inv_sa + COLOR_CHAN_MAX / 2) / COLOR_CHAN_MAX)) / out_a;

    return COLOR_RGBA(
        MIN(out_r, COLOR_CHAN_MAX),
        MIN(out_g, COLOR_CHAN_MAX),
        MIN(out_b, COLOR_CHAN_MAX),
        MIN(out_a, COLOR_CHAN_MAX)
    );
}

/* projects an integer pixel rect into UV coords */
static inline aabb2f _asset_calc_uv(aabb2i px, vec2i atlas_sz) {
    float inv_w = 1.0f / (float)atlas_sz.x;
    float inv_h = 1.0f / (float)atlas_sz.y;
    return aabb2f(
        vec2f((float)px.min_x * inv_w, (float)px.min_y * inv_h),
        vec2f((float)px.max_x * inv_w, (float)px.max_y * inv_h)
    );
}

/* returns pointer to pixel at local px=(x, y) */
static inline uint32_t *_asset_slot_px(const asset_slot_t *slot, vec2i px) {
    return &slot->pixels[px.y * slot->stride + px.x];
}

/* returns pointer to start of row y */
static inline uint32_t *_asset_slot_row(const asset_slot_t *slot, int32_t y) {
    return &slot->pixels[y * slot->stride];
}

/* ============================================================================================== */
/* >SKYLINE 2D BIN PACKER                                                                         */
/* ============================================================================================== */

typedef struct asset_skyline_node {
    vec2i   px;
    int32_t wid;
} asset_skyline_node_t;

typedef struct asset_skyline_packer {
    asset_skyline_node_t *nodes;
    int32_t               node_cnt;
    int32_t               max_nodes;
    vec2i                 atlas_sz;
} asset_skyline_packer_t;

static void _asset_packer_init(asset_skyline_packer_t *p, asset_skyline_node_t *nodes, int32_t max_nodes, vec2i sz) {
    p->nodes        = nodes;
    p->node_cnt     = 1;
    p->max_nodes    = max_nodes;
    p->atlas_sz     = sz;
    p->nodes[0].px  = vec2i(0, 0);
    p->nodes[0].wid = sz.x;
}

static bool _asset_packer_alloc(asset_skyline_packer_t *p, vec2i sz, aabb2i *out_px) {
    vec2i   best_sz  = vec2i(INT32_MAX, INT32_MAX);
    int32_t best_idx = -1;
    int32_t best_y   = 0;

    for (int32_t i = 0; i < p->node_cnt; ++i) {
        if (p->nodes[i].px.x + sz.x > p->atlas_sz.x) break;

        int32_t y       = 0;
        int32_t wid_rem = sz.x;
        int32_t j       = i;

        while (wid_rem > 0 && j < p->node_cnt) {
            y = MAX(y, p->nodes[j].px.y);
            wid_rem -= p->nodes[j].wid;
            j++;
        }

        if (wid_rem <= 0 && y + sz.y <= p->atlas_sz.y) {
            if (y < best_sz.y || (y == best_sz.y && p->nodes[i].wid < best_sz.x)) {
                best_sz.y = y;
                best_sz.x = p->nodes[i].wid;
                best_idx  = i;
                best_y    = y;
            }
        }
    }

    if (best_idx == -1)              return false;
    if (p->node_cnt >= p->max_nodes) return false;

    asset_skyline_node_t new_node = {
        .px  = vec2i(p->nodes[best_idx].px.x, best_y + sz.y),
        .wid = sz.x,
    };

    /* insert new node */
    for (int32_t i = p->node_cnt; i > best_idx; --i) p->nodes[i] = p->nodes[i - 1];
    p->nodes[best_idx] = new_node;
    p->node_cnt++;

    /* shrink/merge overlapping nodes */
    for (int32_t i = best_idx + 1; i < p->node_cnt; ++i) {
        int32_t prev_x = p->nodes[i - 1].px.x + p->nodes[i - 1].wid;

        if (p->nodes[i].px.x < prev_x) {
            int32_t shrink    = prev_x - p->nodes[i].px.x;
            p->nodes[i].px.x += shrink;
            p->nodes[i].wid  -= shrink;

            if (p->nodes[i].wid <= 0) {
                for (int32_t k = i; k < p->node_cnt - 1; ++k) p->nodes[k] = p->nodes[k + 1];
                p->node_cnt--;
                i--;
            }
        } else break;
    }

    out_px->min_x = new_node.px.x;
    out_px->min_y = best_y;
    out_px->max_x = new_node.px.x + sz.x;
    out_px->max_y = best_y        + sz.y;

    return true;
}

/* ============================================================================================== */
/* >LIFECYCLE                                                                                     */
/* ============================================================================================== */

static inline res_t _asset_atlas_bake_defaults(asset_atlas_t *atlas) {
    /* bake ID0 */
    asset_slot_t slot_white;
    res_t r_white = asset_atlas_alloc_slot(atlas, vec2i(1, 1), &slot_white);
    if   (r_white != OK) return r_white;
    slot_white.pixels[0] = COLOR_WHITE;

    /* bake ID1 */
    asset_slot_t slot_checker;
    res_t r_checker = asset_atlas_alloc_slot(atlas, vec2i(8, 8), &slot_checker);
    if   (r_checker != OK) return r_checker;
    for (int32_t y = 0; y < 8; ++y) {
        uint32_t *row = _asset_slot_row(&slot_checker, y);
        for (int32_t x = 0; x < 8; ++x)
            row[x] = (!((x / 4 + y / 4) % 2))? COLOR_MAGENTA : COLOR_BLACK;
    }

    atlas->dirty = true;
    return OK;
}

res_t asset_atlas_init(asset_atlas_t *atlas, arena_t *perm, vec2i size, uint32_t max_sprites) {
    ASSERT(atlas,                      "Attempted to initialize NULL asset_atlas (perm: %p, size: " FMT_VEC2I ", max_sprites: %" PRIu32 ")",                  (void *)perm,  ARG_VEC2I(size), max_sprites);
    ASSERT(perm,                       "Attempted to initialize asset_atlas with NULL perm arena (atlas: %p, size: " FMT_VEC2I ", max_sprites: %" PRIu32 ")", (void *)atlas, ARG_VEC2I(size), max_sprites);
    ASSERT(size.x >= 0 && size.y >= 0, "Atlas dimensions must be non-negative (0 for default), got " FMT_VEC2I, ARG_VEC2I(size));
    if   (!atlas || !perm || size.x < 0 || size.y < 0) return EINVAL;

    memset(atlas, 0, sizeof(*atlas));
    atlas->size        = (size.x && size.y) ? size        : vec2i(ASSET_ATLAS_DEF_WID, ASSET_ATLAS_DEF_HEI);
    atlas->max_sprites = max_sprites        ? max_sprites : ASSET_ATLAS_MAX_SPRITES;

    /* alloc pixel bitmap */
    size_t pixels_sz = (size_t)atlas->size.x * (size_t)atlas->size.y * sizeof(uint32_t);
    atlas->pixels    = arena_alloc(perm, pixels_sz);
    if   (!atlas->pixels) return EOOM;
    memset(atlas->pixels, 0, pixels_sz);

    /* alloc sprite metadata tab */
    size_t sprites_sz = (size_t)atlas->max_sprites * sizeof(asset_sprite_t);
    atlas->sprites    = arena_alloc(perm, sprites_sz);
    if   (!atlas->sprites) return EOOM;
    memset(atlas->sprites, 0, sprites_sz);

    /* alloc skyline packer state */
    size_t packer_mem_sz = sizeof(asset_skyline_packer_t) + (size_t)atlas->size.x * sizeof(asset_skyline_node_t);
    atlas->packer_mem = arena_alloc(perm, packer_mem_sz);
    if   (!atlas->packer_mem) return EOOM;
    memset(atlas->packer_mem, 0, packer_mem_sz);

    asset_skyline_packer_t *packer = (asset_skyline_packer_t *)atlas->packer_mem;
    asset_skyline_node_t   *nodes  = (asset_skyline_node_t   *)(packer + 1);
    _asset_packer_init(packer, nodes, atlas->size.x, atlas->size);

    return _asset_atlas_bake_defaults(atlas);
}

res_t asset_atlas_reset(asset_atlas_t *atlas) {
    ASSERT(atlas, "Attempted to reset NULL asset_atlas");
    if   (!atlas || !atlas->pixels || !atlas->sprites || !atlas->packer_mem) return EINVAL;

    size_t pixels_sz = (size_t)atlas->size.x * (size_t)atlas->size.y * sizeof(uint32_t);
    memset(atlas->pixels, 0, pixels_sz);

    size_t sprites_sz = (size_t)atlas->max_sprites * sizeof(asset_sprite_t);
    memset(atlas->sprites, 0, sprites_sz);
    atlas->sprite_cnt = 0;

    asset_skyline_packer_t *packer = (asset_skyline_packer_t *)atlas->packer_mem;
    asset_skyline_node_t   *nodes  = (asset_skyline_node_t   *)(packer + 1);
    _asset_packer_init(packer, nodes, atlas->size.x, atlas->size);

    return _asset_atlas_bake_defaults(atlas);
}

res_t asset_atlas_alloc_slot(asset_atlas_t *atlas, vec2i size, asset_slot_t *out_slot) {
    ASSERT(atlas,                    "Attempted to allocate slot in NULL asset_atlas (size: " FMT_VEC2I ", out_slot: %p)",                    ARG_VEC2I(size), (void *)out_slot);
    ASSERT(out_slot,                 "Attempted to allocate slot with NULL out_slot pointer (atlas: %p, size: " FMT_VEC2I ")", (void *)atlas, ARG_VEC2I(size));
    ASSERT(size.x > 0 && size.y > 0, "Atlas dimensions must be positive, got " FMT_VEC2I, ARG_VEC2I(size));
    if   (!atlas || size.x <= 0 || size.y <= 0 || !out_slot) return EINVAL;
    if   (atlas->sprite_cnt >= atlas->max_sprites)           return EFULL;

    asset_skyline_packer_t *packer = (asset_skyline_packer_t *)atlas->packer_mem;
    aabb2i px;
    if (!_asset_packer_alloc(packer, size, &px)) return EFULL;

    asset_sprite_id_t id = (asset_sprite_id_t)atlas->sprite_cnt++;

    /* record sprite metadata */
    atlas->sprites[id] = (asset_sprite_t){
        .uv    = _asset_calc_uv(px, atlas->size),
        .px    = px,
        .pivot = PIVOT_CENTER,
        .id    = id,
    };

    /* fill out_slot */
    *out_slot = (asset_slot_t){
        .atlas       = atlas,
        .pixels      = &atlas->pixels[IDX_2D(px.min_x, px.min_y, atlas->size.x)],
        .stride      = (uint32_t)atlas->size.x,
        .px          = px,
        .id          = id,
        .antialiased = true,
    };

    atlas->dirty = true;
    return OK;
}

/* ============================================================================================== */
/* >PROCEDURAL VECTOR CANVAS                                                                      */
/* ============================================================================================== */

/* blends col modulated by coverage factor [0.0, 1.0] onto slot pixel */
static inline void _asset_slot_blend_cov(asset_slot_t *slot, vec2i px, uint32_t col, float cov) {
    if (cov <= 0.0f) return;

    vec2i sz = aabb_size(slot->px);
    if (px.x < 0 || px.x >= sz.x || px.y < 0 || px.y >= sz.y) return;

    if (!slot->antialiased) {
        if (cov < 0.5f) return;
        cov = 1.0f;
    }

    uint32_t *dst = _asset_slot_px(slot, px);
    if (cov >= 1.0f) *dst = _asset_col_over(*dst, col);
    else {
        uint32_t a     = COLOR_A(col);
        uint32_t eff_a = (uint32_t)((float)a * cov + 0.5f);
        if (eff_a == COLOR_CHAN_MIN) return;

        uint32_t eff_col = COLOR_RGBA(COLOR_R(col), COLOR_G(col), COLOR_B(col), eff_a);
        *dst             = _asset_col_over(*dst, eff_col);
    }
}

/* linearly interpolates two colors */
static inline uint32_t _asset_col_lerp(uint32_t c0, uint32_t c1, float t) {
    t = CLAMP(t, 0.0f, 1.0f);
    vec2f vt = vec2f(1.0f - t, t);

    return COLOR_RGBA(
        (uint32_t)(vec_dot(vt, vec2f(COLOR_R(c0), COLOR_R(c1))) + 0.5f),
        (uint32_t)(vec_dot(vt, vec2f(COLOR_G(c0), COLOR_G(c1))) + 0.5f),
        (uint32_t)(vec_dot(vt, vec2f(COLOR_B(c0), COLOR_B(c1))) + 0.5f),
        (uint32_t)(vec_dot(vt, vec2f(COLOR_A(c0), COLOR_A(c1))) + 0.5f)
    );
}

static inline aabb2i _asset_slot_scan_bounds(vec2i slot_sz, aabb2f b, float pad) {
    return aabb_clamp(
        aabb2i(aabb_expand(b, vec2f_splat(pad))),
        aabb_pos_size(vec2i_splat(0), slot_sz)
    );
}

void asset_slot_clear(asset_slot_t *slot, uint32_t col) {
    if (!slot || !slot->pixels) return;

    vec2i sz = aabb_size(slot->px);
    if (sz.x <= 0 || sz.y <= 0) return;

    for (int32_t y = 0; y < sz.y; ++y) {
        uint32_t *row = _asset_slot_row(slot, y);
        for (int32_t x = 0; x < sz.x; ++x) row[x] = col;
    }
}

void asset_slot_set_pixel(asset_slot_t *slot, vec2i px, uint32_t col) {
    if (!slot) return;

    vec2i sz = aabb_size(slot->px);
    if (px.x < 0 || px.x >= sz.x || px.y < 0 || px.y >= sz.y) return;

    uint32_t *p = _asset_slot_px(slot, px);
    *p          = _asset_col_over(*p, col);
}

void asset_slot_draw_rect(asset_slot_t *slot, aabb2i rect, uint32_t col) {
    if (!slot || !aabb_valid(rect) || COLOR_A(col) == COLOR_CHAN_MIN) return;

    vec2i sz = aabb_size(slot->px);
    if (sz.x <= 0 || sz.y <= 0) return;

    aabb2i clipped = aabb_clamp(rect, aabb2i(vec2i_splat(0), sz));
    if (!aabb_valid(clipped)) return;

    for (int32_t y = clipped.min_y; y < clipped.max_y; ++y) {
        uint32_t *row = _asset_slot_row(slot, y);
        for (int32_t x = clipped.min_x; x < clipped.max_x; ++x)
            row[x] = _asset_col_over(row[x], col);
    }
}

void asset_slot_draw_rect_outline(asset_slot_t *slot, aabb2i rect, float thickness, uint32_t col) {
    if (!slot || !aabb_valid(rect) || thickness <= 0.0f || COLOR_A(col) == COLOR_CHAN_MIN) return;

    vec2i sz = aabb_size(slot->px);
    if (sz.x <= 0 || sz.y <= 0) return;

    vec2f  center = aabb_center(aabb2f(rect));
    vec2f  ext    = vec_scale(aabb_size(aabb2f(rect)), 0.5f);
    vec2f  in_ext = vec_sub(ext, vec2f_splat(thickness));

    aabb2i scan_bounds = _asset_slot_scan_bounds(sz, aabb2f(rect), 1.0f);
    if (!aabb_valid(scan_bounds)) return;

    vec2f q = {0};
    for (int32_t y = scan_bounds.min_y; y < scan_bounds.max_y; ++y) {
        q.y = ABS((float)y + 0.5f - center.y);

        for (int32_t x = scan_bounds.min_x; x < scan_bounds.max_x; ++x) {
            q.x = ABS((float)x + 0.5f - center.x);

            vec2f d    = vec_sub(q, ext);
            float d_out = vec_len(vec_max(d, vec2f_splat(0))) + MIN(MAX(d.x, d.y), 0.0f);

            float cov = CLAMP(0.5f - d_out, 0.0f, 1.0f);
            if (cov == 0.0f) continue;

            if (in_ext.x > 0.0f && in_ext.y > 0.0f) {
                vec2f in_d = vec_sub(q, in_ext);
                float d_in = vec_len(vec_max(in_d, vec2f_splat(0))) + MIN(MAX(in_d.x, in_d.y), 0.0f);
                cov = MIN(cov, CLAMP(d_in + 0.5f, 0.0f, 1.0f));
            }

            _asset_slot_blend_cov(slot, vec2i(x, y), col, cov);
        }
    }
}

void asset_slot_draw_rect_rounded(asset_slot_t *slot, aabb2i rect, float radius, uint32_t col) {
    if (!slot || !aabb_valid(rect) || COLOR_A(col) == COLOR_CHAN_MIN) return;

    vec2i sz = aabb_size(slot->px);
    if (sz.x <= 0 || sz.y <= 0) return;

    vec2f  center = aabb_center(aabb2f(rect));
    vec2f  ext    = vec_scale(aabb_size(aabb2f(rect)), 0.5f);

    float r = CLAMP(radius, 0.0f, MIN(ext.x, ext.y));
    vec2f b = vec_sub(ext, vec2f_splat(r));

    aabb2i scan_bounds = _asset_slot_scan_bounds(sz, aabb2f(rect), 1.0f);
    if (!aabb_valid(scan_bounds)) return;

    vec2f q = {0};
    for (int32_t y = scan_bounds.min_y; y < scan_bounds.max_y; ++y) {
        q.y = ABS((float)y + 0.5f - center.y) - b.y;

        for (int32_t x = scan_bounds.min_x; x < scan_bounds.max_x; ++x) {
            q.x = ABS((float)x + 0.5f - center.x) - b.x;

            float dist = vec_len(vec_max(q, vec2f_splat(0))) + MIN(MAX(q.x, q.y), 0.0f) - r;
            float cov  = CLAMP(0.5f - dist, 0.0f, 1.0f);

            _asset_slot_blend_cov(slot, vec2i(x, y), col, cov);
        }
    }
}

void asset_slot_draw_circle(asset_slot_t *slot, vec2f center, float radius, uint32_t col) {
    if (!slot || radius < 0.0f || COLOR_A(col) == COLOR_CHAN_MIN) return;

    vec2i sz = aabb_size(slot->px);
    if (sz.x <= 0 || sz.y <= 0) return;

    aabb2i scan_bounds = _asset_slot_scan_bounds(sz, aabb_center_half(center, vec2f_splat(radius)), 1.0f);
    if (!aabb_valid(scan_bounds)) return;

    float r_inner    = MAX(radius - 0.5f, 0.0f);
    float r_outer    = radius + 0.5f;
    float r_inner_sq = r_inner * r_inner;
    float r_outer_sq = r_outer * r_outer;

    for (int32_t y = scan_bounds.min_y; y < scan_bounds.max_y; ++y) {
        float dy  = (float)y + 0.5f - center.y;
        float dy2 = dy * dy;

        for (int32_t x = scan_bounds.min_x; x < scan_bounds.max_x; ++x) {
            float dx  = (float)x + 0.5f - center.x;
            float d2 = dx * dx + dy2;
            if (d2 > r_outer_sq) continue;

            if (d2 <= r_inner_sq) {
                uint32_t *p = _asset_slot_px(slot, vec2i(x, y));
                *p          = _asset_col_over(*p, col);
            } else {
                float dist = sqrtf(d2) - radius;
                float cov  = CLAMP(0.5f - dist, 0.0f, 1.0f);
                _asset_slot_blend_cov(slot, vec2i(x, y), col, cov);
            }
        }
    }
}

void asset_slot_draw_circle_outline(asset_slot_t *slot, vec2f center, float radius, float thickness, uint32_t col) {
    if (!slot || radius <= 0.0f || thickness <= 0.0f || COLOR_A(col) == COLOR_CHAN_MIN) return;

    vec2i sz = aabb_size(slot->px);
    if (sz.x <= 0 || sz.y <= 0) return;

    float half_t = thickness * 0.5f;
    float bound  = radius + half_t + 1.0f;

    aabb2i scan_bounds = _asset_slot_scan_bounds(sz, aabb_center_half(center, vec2f_splat(bound)), 0.0f);
    if (!aabb_valid(scan_bounds)) return;

    float inner_r  = MAX(radius - half_t - 0.5f, 0.0f);
    float outer_r  = radius + half_t + 0.5f;
    float inner_sq = inner_r * inner_r;
    float outer_sq = outer_r * outer_r;

    for (int32_t y = scan_bounds.min_y; y < scan_bounds.max_y; ++y) {
        float dy  = (float)y + 0.5f - center.y;
        float dy2 = dy * dy;

        for (int32_t x = scan_bounds.min_x; x < scan_bounds.max_x; ++x) {
            float dx = (float)x + 0.5f - center.x;
            float d2 = dx * dx + dy2;
            if (d2 < inner_sq || d2 > outer_sq) continue;

            float dist = ABS(sqrtf(d2) - radius) - half_t;
            float cov  = CLAMP(0.5f - dist, 0.0f, 1.0f);
            _asset_slot_blend_cov(slot, vec2i(x, y), col, cov);
        }
    }
}

void asset_slot_draw_triangle(asset_slot_t *slot, vec2f p0, vec2f p1, vec2f p2, uint32_t col) {
    if (!slot || COLOR_A(col) == COLOR_CHAN_MIN) return;

    vec2i sz = aabb_size(slot->px);
    if (sz.x <= 0 || sz.y <= 0) return;

    vec2f min_p = vec_min(p0, p1, p2), max_p = vec_max(p0, p1, p2);

    aabb2i scan_bounds = _asset_slot_scan_bounds(sz, aabb2f(min_p, max_p), 1.0f);
    if (!aabb_valid(scan_bounds)) return;

    vec2f e0  = vec_sub(p1, p0), e1 = vec_sub(p2, p1), e2 = vec_sub(p0, p2);
    float det = vec_cross(e0, e2);
    if (ABS(det) < 1e-6f) return;

    for (int32_t y = scan_bounds.min_y; y < scan_bounds.max_y; ++y)
        for (int32_t x = scan_bounds.min_x; x < scan_bounds.max_x; ++x) {
            vec2f p  = vec_add(vec2f(x, y), vec2f_splat(0.5f));
            vec2f v0 = vec_sub(p, p0), v1 = vec_sub(p, p1), v2 = vec_sub(p, p2);

            vec2f pq0 = vec_sub(v0, vec_scale(e0, CLAMP(vec_dot(v0, e0) / vec_dot(e0, e0), 0.0f, 1.0f)));
            vec2f pq1 = vec_sub(v1, vec_scale(e1, CLAMP(vec_dot(v1, e1) / vec_dot(e1, e1), 0.0f, 1.0f)));
            vec2f pq2 = vec_sub(v2, vec_scale(e2, CLAMP(vec_dot(v2, e2) / vec_dot(e2, e2), 0.0f, 1.0f)));

            float s  = (det < 0.0f) ? -1.0f : 1.0f;
            float c0 = s * vec_cross(v0, e0), c1 = s * vec_cross(v1, e1), c2 = s * vec_cross(v2, e2);
            bool inside = (c0 >= 0.0f && c1 >= 0.0f && c2 >= 0.0f);

            float min_d = MIN(vec_dot(pq0, pq0), vec_dot(pq1, pq1), vec_dot(pq2, pq2));
            float dist  = (inside ? -1.0f : 1.0f) * sqrtf(min_d);
            float cov   = CLAMP(0.5f - dist, 0.0f, 1.0f);

            _asset_slot_blend_cov(slot, vec2i(x, y), col, cov);
        }
}

void asset_slot_draw_line(asset_slot_t *slot, vec2f p0, vec2f p1, float thickness, uint32_t col) {
    if (!slot || thickness <= 0.0f || COLOR_A(col) == COLOR_CHAN_MIN) return;

    vec2i sz = aabb_size(slot->px);
    if (sz.x <= 0 || sz.y <= 0) return;

    float half_t = thickness * 0.5f;
    vec2f min_p  = vec_min(p0, p1);
    vec2f max_p  = vec_max(p0, p1);

    aabb2i scan_bounds = _asset_slot_scan_bounds(sz, aabb2f(min_p, max_p), half_t + 1.0f);
    if (!aabb_valid(scan_bounds)) return;

    vec2f d  = vec_sub(p1, p0);
    float l2 = vec_dot(d, d);

    for (int32_t y = scan_bounds.min_y; y < scan_bounds.max_y; ++y)
        for (int32_t x = scan_bounds.min_x; x < scan_bounds.max_x; ++x) {
            vec2f p = vec_add(vec2f(x, y), vec2f_splat(0.5f));
            vec2f v = vec_sub(p, p0);

            float t    = (l2 > 0.0f) ? CLAMP(vec_dot(v, d) / l2, 0.0f, 1.0f) : 0.0f;
            vec2f proj = vec_add(p0, vec_scale(d, t));
            float dist = vec_len(vec_sub(p, proj)) - half_t;
            float cov  = CLAMP(0.5f - dist, 0.0f, 1.0f);

            _asset_slot_blend_cov(slot, vec2i(x, y), col, cov);
        }
}

void asset_slot_draw_bezier(asset_slot_t *slot, vec2f p0, vec2f p1, vec2f p2, float thickness, uint32_t col) {
    if (!slot || thickness <= 0.0f || COLOR_A(col) == COLOR_CHAN_MIN) return;

    const int32_t steps = 16;
    vec2f         prev  = p0;

    for (int32_t i = 1; i <= steps; ++i) {
        float t = (float)i / (float)steps;
        float inv_t = 1.0f - t;
        vec2f curr = vec_add(
            vec_scale(p0, inv_t * inv_t),
            vec_scale(p1, 2.0f * inv_t *t),
            vec_scale(p2, t * t)
        );

        asset_slot_draw_line(slot, prev, curr, thickness, col);
        prev = curr;
    }
}

void asset_slot_draw_gradient_linear(asset_slot_t *slot, aabb2i rect, uint32_t col_top, uint32_t col_bot) {
    if (!slot || !aabb_valid(rect)) return;

    vec2i sz = aabb_size(slot->px);
    if (sz.x <= 0 || sz.y <= 0) return;

    aabb2i clipped = aabb_clamp(rect, aabb2i(vec2i_splat(0), sz));
    if (!aabb_valid(clipped)) return;

    float inv_h = (rect.max_y > rect.min_y + 1) ? 1.0f / (float)(rect.max_y - rect.min_y - 1) : 1.0f;

    for (int32_t y = clipped.min_y; y < clipped.max_y; ++y) {
        float t       = (float)(y - rect.min_y) * inv_h;
        uint32_t  col = _asset_col_lerp(col_top, col_bot, t);
        uint32_t *row = _asset_slot_row(slot, y);

        for (int32_t x = clipped.min_x; x < clipped.max_x; ++x)
            row[x] = _asset_col_over(row[x], col);
    }
}

void asset_slot_draw_gradient_radial(asset_slot_t *slot, vec2f center, float radius, uint32_t col_in, uint32_t col_out) {
    if (!slot || radius <= 0.0f) return;

    vec2i sz = aabb_size(slot->px);
    if (sz.x <= 0 || sz.y <= 0) return;

    aabb2i scan_bounds = _asset_slot_scan_bounds(sz, aabb_center_half(center, vec2f_splat(radius)), 1.0f);
    if (!aabb_valid(scan_bounds)) return;

    float inv_r = 1.0f / radius;
    float outer_sq = (radius + 0.5f) * (radius + 0.5f);

    for (int32_t y = scan_bounds.min_y; y < scan_bounds.max_y; ++y) {
        float dy  = (float)y + 0.5f - center.y;
        float dy2 = dy * dy;

        for (int32_t x = scan_bounds.min_x; x < scan_bounds.max_x; ++x) {
            float dx = (float)x + 0.5f - center.x;
            float d2 = dx * dx + dy2;
            if (d2 > outer_sq) continue;

            float d = sqrtf(d2);
            float t = CLAMP(d * inv_r, 0.0f, 1.0f);
            uint32_t col = _asset_col_lerp(col_in, col_out, t);

            float cov = CLAMP(radius + 0.5f - d, 0.0f, 1.0f);
            _asset_slot_blend_cov(slot, vec2i(x, y), col, cov);
        }
    }
}

/* ============================================================================================== */
/* >TEXT PIXEL ART DSL                                                                            */
/* ============================================================================================== */

/* decodes hex chars '0'-'9', 'a'-'f', 'A'-'F' to 0-15 */
static inline bool _asset_hex_val(char c, uint32_t *out_idx) {
    if (c >= '0' && c <= '9') { *out_idx = (uint32_t)(c - '0');        return true; }
    if (c >= 'a' && c <= 'f') { *out_idx = (uint32_t)(10 + (c - 'a')); return true; }
    if (c >= 'A' && c <= 'F') { *out_idx = (uint32_t)(10 + (c - 'A')); return true; }
    return false;
}

res_t asset_bake_text_sprite(asset_atlas_t *atlas, const char *ascii, vec2i size, const uint32_t *palette_16, asset_sprite_id_t *out_id) {
    ASSERT(atlas,                    "Attempted to bake text sprite in NULL atlas (ascii: %p, size: " FMT_VEC2I ", palette_16: %p, out_id: %p)",                          (void *)ascii, ARG_VEC2I(size), (void *)palette_16, (void *)out_id);
    ASSERT(ascii,                    "Attempted to bake text sprite with NULL ascii string (atlas: %p, size: " FMT_VEC2I ", palette_16: %p, out_id: %p)", (void *)atlas,                 ARG_VEC2I(size), (void *)palette_16, (void *)out_id);
    ASSERT(out_id,                   "Attempted to bake text sprite with NULL out_id pointer (atlas: %p, ascii: %p, size: " FMT_VEC2I ", palette_16: %p)", (void *)atlas, (void *)ascii, ARG_VEC2I(size), (void *)palette_16);
    ASSERT(size.x > 0 && size.y > 0, "Text sprite dimensions must be positive, got " FMT_VEC2I, ARG_VEC2I(size));
    if   (!atlas || !ascii || !out_id || size.x <= 0 || size.y <= 0) return EINVAL;

    asset_slot_t slot;
    res_t r = asset_atlas_alloc_slot(atlas, size, &slot);
    if (r != OK) return r;

    const uint32_t *pal = palette_16 ? palette_16 : g_palette_pico8;

    vec2i       px = {0};
    const char *p = ascii;

    while (*p && px.y < size.y) {
        char c = *p++;
        if (c == '\n' || c == '\r' || c == '\t') continue;

        uint32_t col = COLOR_CLEAR;
        uint32_t idx = 0;
        if (_asset_hex_val(c, &idx)) col = pal[idx];

        uint32_t *dst = _asset_slot_px(&slot, px);
        *dst = col;
        px.x++;

        if (px.x >= size.x) {
            px.x = 0;
            px.y++;
        }
    }

    /* clear any remaining unfilled pixels */
    while (px.y < size.y) {
        uint32_t *dst = _asset_slot_px(&slot, px);
        *dst = COLOR_CLEAR;
        px.x++;
        if (px.x >= size.x) {
            px.x = 0;
            px.y++;
        }
    }

    *out_id      = slot.id;
    atlas->dirty = true;

    return OK;
}

res_t asset_bake_text_animation(asset_atlas_t *atlas, const char **frames, uint32_t frame_cnt, vec2i size, const uint32_t *palette_16, asset_sprite_id_t *out_base_id) {
    ASSERT(atlas,                    "Attempted to bake text animation in NULL atlas (frames: %p, frame_cnt: %" PRIu32 ", size: " FMT_VEC2I ", palette_16: %p, out_base_id: %p)",                          (void *)frames, frame_cnt, ARG_VEC2I(size), (void *)palette_16, (void *)out_base_id);
    ASSERT(frames,                   "Attempted to bake text animation with NULL frames array (atlas: %p, frame_cnt: %" PRIu32 ", size: " FMT_VEC2I ", palette_16: %p, out_base_id: %p)",   (void *)atlas,                 frame_cnt, ARG_VEC2I(size), (void *)palette_16, (void *)out_base_id);
    ASSERT(out_base_id,              "Attempted to bake text animation with NULL out_base_id pointer (atlas: %p, frames: %p, frame_cnt: %" PRIu32 ", size: " FMT_VEC2I ", palette_16: %p)", (void *)atlas, (void *)frames, frame_cnt, ARG_VEC2I(size), (void *)palette_16);
    ASSERT(frame_cnt,                "Animation frame count must be greater than zero, got %" PRIu32, frame_cnt);
    ASSERT(size.x > 0 && size.y > 0, "Animation frame dimensions must be positive, got " FMT_VEC2I, ARG_VEC2I(size));
    if    (!atlas || !frames || !out_base_id || !frame_cnt || size.x <= 0 || size.y <= 0) return EINVAL;
    if    (atlas->sprite_cnt + frame_cnt > atlas->max_sprites)                            return EFULL;

    for (uint32_t i = 0; i < frame_cnt; ++i) {
        if (!frames[i]) return EINVAL;

        asset_sprite_id_t id;
        res_t r = asset_bake_text_sprite(atlas, frames[i], size, palette_16, &id);
        if (r != OK) return r;

        if (i == 0) *out_base_id = id;
        ASSERT(id == *out_base_id + i, "Baked animation frame IDs must be contiguous");
    }

    return OK;
}

/* ============================================================================================== */
/* >IMAGE LOADERS / SPRITESHEET LOGIC                                                             */
/* ============================================================================================== */

static inline res_t _asset_ingest_decoded(asset_atlas_t *atlas, stbi_uc *pxs, vec2i sz, asset_sprite_id_t *out_id) {
    if (!pxs || sz.x <= 0 || sz.y <= 0) return ENOTFND;

    asset_slot_t slot;
    res_t r = asset_atlas_alloc_slot(atlas, sz, &slot);
    if (r != OK) { stbi_image_free(pxs); return r; }

    for (int32_t y = 0; y < sz.y; ++y) {
        const stbi_uc *src = pxs + (size_t)y * (size_t)sz.x * sizeof(uint32_t);
        uint32_t      *dst = _asset_slot_row(&slot, y);

        for (int32_t x = 0; x < sz.x; ++x) {
            size_t off = (size_t)x * sizeof(uint32_t);
            dst[x]     = COLOR_RGBA(src[off], src[off + 1U], src[off + 2U], src[off + 3U]);
        }
    }

    stbi_image_free(pxs);
    *out_id = slot.id;

    return OK;
}

res_t asset_load_image_mem(asset_atlas_t *atlas, const void *buf, size_t size, asset_sprite_id_t *out_id) {
    ASSERT(atlas,  "Attempted to load image into NULL asset_atlas (buf: %p, size: %zu, out_id: %p)",                  buf, size,  (void *)out_id);
    ASSERT(buf,    "Attempted to load image from NULL buffer (atlas: %p, size: %zu, out_id: %p)",      (void *)atlas,      size,  (void *)out_id);
    ASSERT(out_id, "Attempted to load image with NULL out_id pointer (atlas: %p, buf: %p, size: %zu)", (void *)atlas, buf, size);
    ASSERT(size,   "Attempted to load image with size == 0 (atlas: %p, buf: %p, out_id: %p)",          (void *)atlas, buf,        (void *)out_id);
    if   (!atlas || !buf || !out_id || !size) return EINVAL;

    vec2i    sz   = {0};
    int      comp = 0;
    stbi_uc *pxs  = stbi_load_from_memory((const stbi_uc *)buf, (int)size, &sz.x, &sz.y, &comp, 4);

    return _asset_ingest_decoded(atlas, pxs, sz, out_id);
}

res_t asset_load_image_file(asset_atlas_t *atlas, const char *path, asset_sprite_id_t *out_id) {
    ASSERT(atlas,  "Attempted to load image file into NULL asset_atlas (path: %s, out_id: %p)",          path ? path : "NULL", (void *)out_id);
    ASSERT(path,   "Attempted to load image file with NULL path (atlas: %p, out_id: %p)", (void *)atlas,                       (void *)out_id);
    ASSERT(out_id, "Attempted to load image file with NULL out_id (atlas: %p, path: %s)", (void *)atlas, path ? path : "NULL");
    if   (!atlas || !path || !out_id) return EINVAL;

    vec2i    sz   = {0};
    int      comp = 0;
    stbi_uc *pxs  = stbi_load(path, &sz.x, &sz.y, &comp, 4);

    return _asset_ingest_decoded(atlas, pxs, sz, out_id);
}

res_t asset_register_grid(asset_atlas_t *atlas, asset_sprite_id_t src_id, vec2i cell_size, asset_sprite_id_t *out_ids, uint32_t max_out, uint32_t *out_cnt) {
    ASSERT(atlas, "Attempted to register grid on NULL asset_atlas (src_id: %" PRIu32 ", cell_size: " FMT_VEC2I ")", src_id, ARG_VEC2I(cell_size));
    ASSERT(cell_size.x > 0 && cell_size.y > 0, "Grid cell dimensions must be positive, got " FMT_VEC2I, ARG_VEC2I(cell_size));
    ASSERT(out_ids, "Attempted to register grid with NULL out_ids pointer (atlas: %p, src_id: %" PRIu32 ")", (void *)atlas, src_id);
    ASSERT(out_cnt, "Attempted to register grid with NULL out_cnt pointer (atlas: %p, src_id: %" PRIu32 ")", (void *)atlas, src_id);
    if   (!atlas || cell_size.x <= 0 || cell_size.y <= 0 || !out_ids || !out_cnt || src_id >= atlas->sprite_cnt) return EINVAL;

    const asset_sprite_t *src_sprite = &atlas->sprites[src_id];
    vec2i                 src_sz     = aabb_size(src_sprite->px);
    if (src_sz.x < cell_size.x || src_sz.y < cell_size.y) return EINVAL;

    vec2i    grid_sz = vec_div(src_sz, cell_size);
    uint32_t total   = grid_sz.x * grid_sz.y;
    if (atlas->sprite_cnt + total > atlas->max_sprites) return EFULL;

    uint32_t cnt = 0;
    for (int32_t r = 0; r < grid_sz.y; ++r)
        for (int32_t c = 0; c < grid_sz.x; ++c) {
            if (cnt >= max_out) break;

            aabb2i cell_px = aabb_pos_size(
                vec_add(src_sprite->px.min, vec_mul(vec2i(c, r), cell_size)),
                cell_size
            );

            asset_sprite_id_t id = (asset_sprite_id_t)atlas->sprite_cnt++;
            atlas->sprites[id]   = (asset_sprite_t){
                .uv = _asset_calc_uv(cell_px, atlas->size),
                .px = cell_px,
                .pivot = PIVOT_CENTER,
                .id = id,
            };

            out_ids[cnt++] = id;
        }

    *out_cnt = cnt;

    return OK;
}

/* ============================================================================================== */
/* >HOT RELOADING                                                                                 */
/* ============================================================================================== */

static inline res_t _asset_reload_decoded(asset_atlas_t *atlas, asset_sprite_id_t id, stbi_uc *pxs, vec2i sz) {
    if (!pxs || sz.x <= 0 || sz.y <= 0) return ENOTFND;

    aabb2i spr_px = atlas->sprites[id].px;
    vec2i  spr_sz = aabb_size(spr_px);
    if (sz.x != spr_sz.x || sz.y != spr_sz.y) { stbi_image_free(pxs); return EINVAL; }

    for (int32_t y = 0; y < sz.y; ++y) {
        const stbi_uc *src = pxs + (size_t)y * (size_t)sz.x * sizeof(uint32_t);
        uint32_t      *dst = &atlas->pixels[IDX_2D(spr_px.min_x, spr_px.min_y + y, atlas->size.x)];

        for (int32_t x = 0; x < sz.x; ++x) {
            size_t off = (size_t)x * sizeof(uint32_t);
            dst[x]     = COLOR_RGBA(src[off], src[off + 1U], src[off + 2U], src[off + 3U]);
        }
    }

    stbi_image_free(pxs);
    atlas->dirty = true;

    return OK;
}

res_t asset_reload_image_file(asset_atlas_t *atlas, asset_sprite_id_t id, const char *path) {
    ASSERT(atlas, "Attempted to reload image file in NULL asset_atlas (id: %" PRIu32 ", path: %s)",            id, path ? path : "NULL");
    ASSERT(path,  "Attempted to reload image file with NULL path (atlas: %p, id: %" PRIu32 ")", (void *)atlas, id);
    if   (!atlas || id >= atlas->sprite_cnt || !path) return EINVAL;

    vec2i    sz   = {0};
    int      comp = 0;
    stbi_uc *pxs  = stbi_load(path, &sz.x, &sz.y, &comp, 4);

    return _asset_reload_decoded(atlas, id, pxs, sz);
}

res_t asset_reload_image_mem(asset_atlas_t *atlas, asset_sprite_id_t id, const void *buf, size_t size) {
    ASSERT(atlas, "Attempted to reload image in NULL asset_atlas (id: %" PRIu32 ", buf: %p, size: %zu)",               id, buf, size);
    ASSERT(buf,   "Attempted to reload image from NULL buffer (atlas: %p, id: %" PRIu32 ", size: %zu)", (void *)atlas, id,      size);
    ASSERT(size,  "Attempted to reload image with size == 0 (atlas: %p, id: %" PRIu32 ", buf: %p)",     (void *)atlas, id, buf);
    if (!atlas || id >= atlas->sprite_cnt || !buf || !size) return EINVAL;

    vec2i    sz   = {0};
    int      comp = 0;
    stbi_uc *pxs  = stbi_load_from_memory((const stbi_uc *)buf, (int)size, &sz.x, &sz.y, &comp, 4);

    return _asset_reload_decoded(atlas, id, pxs, sz);
}

/* ============================================================================================== */
/* >PALETTE SWAPPING                                                                              */
/* ============================================================================================== */

res_t asset_atlas_remap_palette(asset_atlas_t *atlas, const uint32_t *src_palette, const uint32_t *dst_palette, uint32_t count) {
    ASSERT(atlas,       "Attempted to remap palette on NULL asset_atlas (src_palette: %p, dst_palette: %p, count: %" PRIu32 ")",                    (void *)src_palette, (void *)dst_palette, count);
    ASSERT(src_palette, "Attempted to remap palette with NULL src_palette pointer (atlas: %p, dst_palette: %p, count: %" PRIu32 ")", (void *)atlas,                      (void *)dst_palette, count);
    ASSERT(dst_palette, "Attempted to remap palette with NULL dst_palette pointer (atlas: %p, src_palette: %p, count: %" PRIu32 ")", (void *)atlas, (void *)src_palette,                      count);
    ASSERT(count,       "Attempted to remap palette with count == 0 (atlas: %p, src_palette: %p, dst_palette: %p)",                  (void *)atlas, (void *)src_palette, (void *)dst_palette);
    if   (!atlas || !src_palette || !dst_palette || !count) return EINVAL;

    size_t total_px = (size_t)atlas->size.x * (size_t)atlas->size.y;
    for (size_t i = 0; i < total_px; ++i) {
        uint32_t col = atlas->pixels[i];

        for (uint32_t k = 0; k < count; ++k) {
            if (col == src_palette[k]) {
                atlas->pixels[i] = dst_palette[k];
                break;
            }
        }
    }

    atlas->dirty = true;

    return OK;
}

res_t asset_atlas_remap_color(asset_atlas_t *atlas, uint32_t src_col, uint32_t dst_col) {
    ASSERT(atlas, "Attempted to remap color on NULL asset_atlas (src_col: 0x%08X, dst_col: 0x%08X)", src_col, dst_col);
    if   (!atlas) return EINVAL;

    size_t total_px = (size_t)atlas->size.x * (size_t)atlas->size.y;
    for (size_t i = 0; i < total_px; ++i)
        if (atlas->pixels[i] == src_col) atlas->pixels[i] = dst_col;

    atlas->dirty = true;

    return OK;
}

/* ============================================================================================== */
/* >DISK EXPORT                                                                                   */
/* ============================================================================================== */

static inline bool _asset_file_exists(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    fclose(f);
    return true;
}

/* converts numeric atlas colors to RGBA bytes in borrowed scratch storage */
static inline res_t _asset_export_rect_png(const asset_atlas_t *atlas, aabb2i rect, const char *path, arena_t *scratch) {
    vec2i size = aabb_size(rect);
    if (size.x <= 0 || size.y <= 0 || (size_t)size.x > INT_MAX / sizeof(uint32_t)) return EINVAL;

    size_t stride = (size_t)size.x * sizeof(uint32_t);
    if ((size_t)size.y > SIZE_MAX / stride) return EINVAL;
    size_t bytes = stride * (size_t)size.y;

    arena_temp_t mark = arena_temp_begin(scratch);
    uint8_t     *rgba = arena_alloc(scratch, bytes);
    if (!rgba) { arena_temp_end(mark); return EOOM; }

    for (int32_t y = 0; y < size.y; ++y) {
        size_t   src_row = (size_t)(rect.min.y + y) * (size_t)atlas->size.x;
        uint8_t *dst     = rgba + (size_t)y * stride;

        for (int32_t x = 0; x < size.x; ++x) {
            uint32_t color = atlas->pixels[src_row + (size_t)(rect.min_x + x)];
            size_t   off   = (size_t)x * sizeof(uint32_t);

            dst[off]      = (uint8_t)COLOR_R(color);
            dst[off + 1U] = (uint8_t)COLOR_G(color);
            dst[off + 2U] = (uint8_t)COLOR_B(color);
            dst[off + 3U] = (uint8_t)COLOR_A(color);
        }
    }

    int ok = stbi_write_png(path, size.x, size.y, 4, rgba, (int)stride);
    arena_temp_end(mark);
    return ok ? OK : EIO;
}

res_t asset_export_sprite_png(const asset_atlas_t *atlas, asset_sprite_id_t id, const char *path, bool overwrite, arena_t *scratch) {
    ASSERT(atlas,   "Attempted to export sprite to PNG from NULL asset_atlas (id: %" PRIu32 ", path: %s)",                       id, path ? path : "NULL");
    ASSERT(path,    "Attempted to export sprite to PNG with NULL path (atlas: %p, id: %" PRIu32 ")",              (void *)atlas, id);
    ASSERT(scratch, "Attempted to export sprite to PNG with NULL scratch (atlas: %p, id: %" PRIu32 ", path: %s)", (void *)atlas, id, path ? path : "NULL");
    if   (!atlas || id >= atlas->sprite_cnt || !path || !scratch) return EINVAL;
    if   (!overwrite && _asset_file_exists(path))                 return EIO;

    return _asset_export_rect_png(atlas, atlas->sprites[id].px, path, scratch);
}

res_t asset_export_atlas_png(const asset_atlas_t *atlas, const char *path, bool overwrite, arena_t *scratch) {
    ASSERT(atlas,   "Attempted to export atlas to PNG from NULL asset_atlas (path: %s)",                       path ? path : "NULL");
    ASSERT(path,    "Attempted to export atlas to PNG with NULL path (atlas: %p)",              (void *)atlas);
    ASSERT(scratch, "Attempted to export atlas to PNG with NULL scratch (atlas: %p, path: %s)", (void *)atlas, path ? path : "NULL");
    if   (!atlas || !path || !scratch)            return EINVAL;
    if   (!overwrite && _asset_file_exists(path)) return EIO;

    aabb2i rect = aabb_pos_size(vec2i_splat(0), atlas->size);
    return _asset_export_rect_png(atlas, rect, path, scratch);
}

/* ============================================================================================== */
/* >MUTATIONS                                                                                     */
/* ============================================================================================== */

res_t asset_sprite_set_pivot(asset_atlas_t *atlas, asset_sprite_id_t id, vec2f norm_pivot) {
    ASSERT(atlas,                   "Attempted to set pivot on NULL asset_atlas (id: %" PRIu32 ", norm_pivot: " FMT_VEC2F ")", id, ARG_VEC2F(norm_pivot));
    ASSERT(id < atlas->max_sprites, "Sprite ID %" PRIu32 " exceeds atlas max_sprites %" PRIu32, id, atlas->max_sprites);
    if   (!atlas || id >= atlas->sprite_cnt) return EINVAL;

    atlas->sprites[id].pivot = norm_pivot;
    return OK;
}

res_t asset_set_sprite_pivot_px(asset_atlas_t *atlas, asset_sprite_id_t id, vec2f px_off) {
    ASSERT(atlas,                   "Attempted to set pixel pivot on NULL asset_atlas (id: %" PRIu32 ")", id);
    ASSERT(id < atlas->max_sprites, "Sprite ID %" PRIu32 " exceeds atlas max_sprites %" PRIu32, id, atlas->max_sprites);
    if   (!atlas || id >= atlas->sprite_cnt) return EINVAL;

    vec2i sz = aabb_size(atlas->sprites[id].px);
    if (sz.x <= 0 || sz.y <= 0) return EINVAL;

    atlas->sprites[id].pivot = vec_div(px_off, vec2f(sz));
    return OK;
}

/* ============================================================================================== */
/* >QUERIES                                                                                       */
/* ============================================================================================== */

const asset_sprite_t *asset_get_sprite(const asset_atlas_t *atlas, asset_sprite_id_t id) {
    ASSERT(atlas, "Attempted to get sprite from NULL asset_atlas (id: %" PRIu32 ")", id);
    if   (!atlas || id >= atlas->sprite_cnt) return NULL;

    return &atlas->sprites[id];
}

aabb2f asset_get_uv(const asset_atlas_t *atlas, asset_sprite_id_t id) {
    ASSERT(atlas, "Attempted to get UV from NULL asset_atlas (id: %" PRIu32 ")", id);
    if    (!atlas)                  return aabb2f(0.0f, 0.0f, 1.0f, 1.0f);
    if    (id >= atlas->sprite_cnt) return atlas->sprite_cnt > ASSET_SPRITE_CHECKER ?
                                           atlas->sprites[ASSET_SPRITE_CHECKER].uv :
                                           aabb2f(0.0f, 0.0f, 1.0f, 1.0f);

    return atlas->sprites[id].uv;
}

uint32_t asset_get_pixel(const asset_atlas_t *atlas, vec2i pos) {
    ASSERT(atlas,                                                                      "Attempted to get pixel from NULL asset_atlas (pos: " FMT_VEC2I ")", ARG_VEC2I(pos));
    ASSERT(pos.x >= 0 && pos.x < atlas->size.x && pos.y >= 0 && pos.y < atlas->size.y, "Attempted to get pixel out of atlas bounds (atlas: %p, pos: " FMT_VEC2I ")", (void *)atlas, ARG_VEC2I(pos));
    if    (!atlas || pos.x < 0 || pos.x >= atlas->size.x || pos.y < 0 || pos.y >= atlas->size.y) return COLOR_CLEAR;
    return atlas->pixels[IDX_2D(pos.x, pos.y, atlas->size.x)];
}

/* ============================================================================================== */
/* >GPU SYNCHRONIZATION                                                                           */
/* ============================================================================================== */

res_t asset_atlas_sync_gpu(asset_atlas_t *atlas) {
    ASSERT(atlas, "Attempted to sync NULL asset_atlas to GPU");
    if   (!atlas) return EINVAL;

    atlas->dirty = false;
    return OK;
}
