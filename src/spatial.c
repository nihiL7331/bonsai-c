/*
====================
spatial.c

2D and 3D uniform spatial hashing broadphase implementation.
Features:
- deterministic 2-pass counting sort for linear bucket construction
- zero heap allocation during ticks via frame scratch arena allocations
- bitset for deduplication (for <=2048 entities stack-allocated)
- Teschner prime spatial hashing for coordinate-to-bucket mapping
- Jim Arvo's clamped distance algorithm for exact circular and spherical queries
====================
*/

#include "spatial.h"
#include "core.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

/* ============================================================================================== */
/* >BITSET HELPERS                                                                                */
/* ============================================================================================== */

typedef struct spatial_bitset {
    uint64_t    *words;
    uint32_t     word_cnt;
    arena_temp_t tmp;
} spatial_bitset_t;

static inline spatial_bitset_t _spatial_bitset_begin(uint32_t ent_cnt, uint64_t *stack_buf, arena_t *scratch) {
    spatial_bitset_t bs = {0};
    bs.word_cnt         = (ent_cnt + 63) / 64;
    if (bs.word_cnt <= SPATIAL_STACK_WORDS) bs.words = stack_buf;
    else if (scratch) {
        ASSERT(scratch, "Spatial query requires scratch arena when entity count exceeds stack buffer (cnt: %" PRIu32 ")", ent_cnt);
        bs.tmp   = arena_temp_begin(scratch);
        bs.words = arena_alloc(scratch, bs.word_cnt * sizeof(uint64_t));
        if (!bs.words) arena_temp_end(bs.tmp);
    }
    if (bs.words) memset(bs.words, 0, bs.word_cnt * sizeof(uint64_t));
    return bs;
}

static inline void _spatial_bitset_end(spatial_bitset_t *bs) {
    if (bs->word_cnt > SPATIAL_STACK_WORDS && bs->tmp.arena) arena_temp_end(bs->tmp);
}

static inline bool _spatial_bitset_test_set(spatial_bitset_t *bs, uint32_t idx) {
    uint32_t word = idx / 64;
    uint64_t bit  = BIT(idx % 64);
    if (bs->words[word] & bit) return true;
    bs->words[word] |= bit;
    return false;
}

/* ============================================================================================== */
/* >CONVERSION HELPERS                                                                            */
/* ============================================================================================== */

static inline int32_t _world_to_cell(float coord, float inv_cell_sz) {
    return (int32_t)floorf(coord * inv_cell_sz);
}

static inline aabb2i _spatial_bounds_to_cells2d(aabb2f b, float inv_cell_sz) {
    int32_t min_cx = _world_to_cell(b.min_x, inv_cell_sz);
    int32_t min_cy = _world_to_cell(b.min_y, inv_cell_sz);
    int32_t max_cx = CLAMP(_world_to_cell(b.max_x, inv_cell_sz), min_cx, min_cx + (int32_t)SPATIAL_MAX_CELL_SPAN - 1);
    int32_t max_cy = CLAMP(_world_to_cell(b.max_y, inv_cell_sz), min_cy, min_cy + (int32_t)SPATIAL_MAX_CELL_SPAN - 1);
    return (aabb2i){ .min_x = min_cx, .min_y = min_cy, .max_x = max_cx, .max_y = max_cy };
}

static inline aabb3i _spatial_bounds_to_cells3d(aabb3f b, float inv_cell_sz) {
    int32_t min_cx = _world_to_cell(b.min_x, inv_cell_sz);
    int32_t min_cy = _world_to_cell(b.min_y, inv_cell_sz);
    int32_t min_cz = _world_to_cell(b.min_z, inv_cell_sz);
    int32_t max_cx = CLAMP(_world_to_cell(b.max_x, inv_cell_sz), min_cx, min_cx + (int32_t)SPATIAL_MAX_CELL_SPAN - 1);
    int32_t max_cy = CLAMP(_world_to_cell(b.max_y, inv_cell_sz), min_cy, min_cy + (int32_t)SPATIAL_MAX_CELL_SPAN - 1);
    int32_t max_cz = CLAMP(_world_to_cell(b.max_z, inv_cell_sz), min_cz, min_cz + (int32_t)SPATIAL_MAX_CELL_SPAN - 1);
    return (aabb3i){ .min_x = min_cx, .min_y = min_cy, .min_z = min_cz, .max_x = max_cx, .max_y = max_cy, .max_z = max_cz };
}

/* ============================================================================================== */
/* >HASHING LOGIC                                                                                 */
/* ============================================================================================== */

#define SPATIAL_HASH_PRIME_X      73856093U
#define SPATIAL_HASH_PRIME_Y      19349663U
#define SPATIAL_HASH_PRIME_Z      83492791U
#define SPATIAL_MURMUR_MULTIPLIER 0x85EBCA6BU

static inline uint32_t _spatial_murmurmix(uint32_t h) {
    h ^= h >> 16;
    h *= SPATIAL_MURMUR_MULTIPLIER;
    h ^= h >> 13;
    return h;
}

static inline uint32_t _spatial_hash2d(int32_t cx, int32_t cy, uint32_t mask) {
    uint32_t h = ((uint32_t)cx * SPATIAL_HASH_PRIME_X) ^
                 ((uint32_t)cy * SPATIAL_HASH_PRIME_Y);
    return _spatial_murmurmix(h) & mask;
}

static inline uint32_t _spatial_hash3d(int32_t cx, int32_t cy, int32_t cz, uint32_t mask) {
    uint32_t h = ((uint32_t)cx * SPATIAL_HASH_PRIME_X) ^
                 ((uint32_t)cy * SPATIAL_HASH_PRIME_Y) ^
                 ((uint32_t)cz * SPATIAL_HASH_PRIME_Z);
    return _spatial_murmurmix(h) & mask;
}

/* ============================================================================================== */
/* >2D LOGIC IMPL                                                                                 */
/* ============================================================================================== */

res_t spatial_grid2d_init(spatial_grid2d_t *grid, float cell_sz, uint32_t bucket_cnt) {
    ASSERT(grid,                                                "Attempted to initialize a NULL spatial_grid2d");
    ASSERT(bucket_cnt == 0 || IS_POW2(bucket_cnt), "bucket_cnt must be a power of two (or 0 for default)");
    ASSERT(cell_sz >= 0.0f,                                     "cell_sz must be >0.0f (or 0.0f for default)");
    if   (!grid || (bucket_cnt && !IS_POW2(bucket_cnt)) || cell_sz < 0.0f) return EINVAL;

    memset(grid, 0, sizeof(*grid));
    grid->inv_cell_sz = 1.0f / ((cell_sz > 0.0f) ? cell_sz : SPATIAL_DEF_CELL_SZ);
    grid->bucket_cnt  = bucket_cnt ? bucket_cnt : SPATIAL_DEF_BUCKETS;

    return OK;
}

res_t spatial_grid2d_build(spatial_grid2d_t *grid, const spatial_entity2d_t *ents, uint32_t ent_cnt, arena_t *scratch) {
    ASSERT(grid, "Attempted to build with NULL grid (ents: %p, ent_cnt: %" PRIu32 ", scratch: %p)",                  (void *)ents, ent_cnt, (void *)scratch);
    ASSERT(scratch, "Attempted to build with NULL scratch (grid: %p, ents: %p, ent_cnt: %" PRIu32 ")", (void *)grid, (void *)ents, ent_cnt);
    if   (!grid || !scratch) return EINVAL;

    if (ent_cnt == 0 || !ents) {
        grid->entities   = NULL;
        grid->cell_offs  = NULL;
        grid->entries    = NULL;
        grid->entity_cnt = 0;
        grid->entry_cnt  = 0;
        return WEMPTY;
    }

    /* scratch-alloc cell_offs */
    grid->cell_offs = arena_alloc(scratch, (grid->bucket_cnt + 1) * sizeof(uint32_t));
    if (!grid->cell_offs) return EOOM;
    memset(grid->cell_offs, 0, (grid->bucket_cnt + 1) * sizeof(uint32_t));

    /* count cell overlaps into cell_offs[b + 1] */
    uint32_t mask          = grid->bucket_cnt - 1;
    uint32_t total_entries = 0;

    for (uint32_t i = 0; i < ent_cnt; ++i) {
        aabb2i c = _spatial_bounds_to_cells2d(ents[i].bounds, grid->inv_cell_sz);
        for (int32_t cy = c.min_y; cy <= c.max_y; ++cy)
            for (int32_t cx = c.min_x; cx <= c.max_x; ++cx) {
                grid->cell_offs[_spatial_hash2d(cx, cy, mask) + 1]++;
                total_entries++;
            }
    }

    /* counts -> offsets */
    grid->cell_offs[0] = 0;
    for (uint32_t b = 1; b <= grid->bucket_cnt; ++b) grid->cell_offs[b] += grid->cell_offs[b - 1];

    /* scratch-alloc entries & cursor */
    grid->entries = arena_alloc(scratch, (size_t)total_entries * sizeof(uint32_t));
    if (total_entries && !grid->entries) return EOOM;

    /* temp to unwind cursor, it only lives for build process */
    arena_temp_t tmp = arena_temp_begin(scratch);
    uint32_t *cursor = arena_alloc(scratch, (size_t)grid->bucket_cnt * sizeof(uint32_t));
    if (!cursor) return EOOM;
    memcpy(cursor, grid->cell_offs, (size_t)grid->bucket_cnt * sizeof(uint32_t));

    /* scatter entity indices via _spatial_hash2d */
    for (uint32_t i = 0; i < ent_cnt; ++i) {
        aabb2i c = _spatial_bounds_to_cells2d(ents[i].bounds, grid->inv_cell_sz);
        for (int32_t cy = c.min_y; cy <= c.max_y; ++cy)
            for (int32_t cx = c.min_x; cx <= c.max_x; ++cx)
                grid->entries[cursor[_spatial_hash2d(cx, cy, mask)]++] = i;
    }

    arena_temp_end(tmp);

    grid->entities   = ents;
    grid->entity_cnt = ent_cnt;
    grid->entry_cnt  = total_entries;

    return OK;
}

res_t spatial_grid2d_build_from_ecs(spatial_grid2d_t *grid, ecs_world_t *world, arena_t *scratch) {
    ASSERT(grid,    "Attempted to build 2D grid from ECS with NULL grid (world: %p, scratch: %p)", (void *)world, (void *)scratch);
    ASSERT(scratch, "Attempted to build 2D grid from ECS with NULL scratch (grid: %p, world: %p)", (void *)grid,  (void *)world);
    if   (!grid || !scratch) return EINVAL;

    /* if world == NULL && s_active_world == NULL return to avoid assertion */
    ecs_world_t *w = world ? world : ecs_get_world();
    if (!w) return ENOTFND;

    /* set active world ctx if explicit world was passed */
    ecs_world_t *prev = ecs_get_world();
    if (world) ecs_set_world(world);

    ecs_view_t view = ecs_query_comp(trans2d, collider2d);
    if (!view.cnt || !view.ents) {
        if (world) ecs_set_world(prev); /* revert */
        return spatial_grid2d_build(grid, NULL, 0, scratch);
    }

    spatial_entity2d_t *ents = arena_alloc(scratch, view.cnt * sizeof(spatial_entity2d_t));
    if (!ents) {
        if (world) ecs_set_world(prev); /* revert */
        return EOOM;
    }

    for (uint32_t i = 0; i < view.cnt; ++i) {
        entity_handle_t ent   = view.ents[i];
        trans2d wt;
        ecs_get_world_trans(ent, &wt);
        const collider2d *col = ecs_get(ent, collider2d);

        vec2f pos  = vec2f(wt.x, wt.y);
        vec2f off  = vec2f_scale(vec2f(
            col->off_x,
            col->off_y
        ), wt.scale);
        vec2f half = vec2f_abs(vec2f_scale(vec2f(
            col->half_x,
            col->half_y
        ), wt.scale));

        vec2f center, extents;

        if (wt.rot != 0.0f) { /* if rotated, recalculate tight aabb */
            float c = cosf(wt.rot);
            float s = sinf(wt.rot);
            /* center_x = x + off_x * scale * cos(theta) - off_y * scale * sin(theta) */
            /* center_y = y + off_x * scale * sin(theta) + off_y * scale * cos(theta) */
            vec2f rot_off = vec2f(
                off.x * c - off.y * s,
                off.x * s + off.y * c
            );
            center = vec2f_add(pos, rot_off);
            /* extents_x = half_x * |cos(theta)| + half_y * |sin(theta)| */
            /* extents_y = half_x * |sin(theta)| + half_y * |cos(theta)| */
            extents = vec2f(
                half.x * ABS(c) + half.y * ABS(s),
                half.x * ABS(s) + half.y * ABS(c)
            );
        } else {
            center  = vec_add(pos, off);
            extents = half;
        }

        /* tight aabb = [center_x - extents_x, center_y - extents_y] -> */
        /*              [center_x + extents_x, center_y + extents_y]    */
        ents[i] = (spatial_entity2d_t){
            .ent    = ent,
            .bounds = aabb_center_half(center, extents),
            .mask   = col->mask,
        };
    }

    if (world) ecs_set_world(prev); /* revert */
    return spatial_grid2d_build(grid, ents, view.cnt, scratch);
}

uint32_t spatial_query_aabb2d(const spatial_grid2d_t *grid, aabb2f box, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out, arena_t *scratch) {
    ASSERT(grid,     "Attempted to query AABB with NULL grid (box: " FMT_AABB2F ", mask: %" PRIu32 ", out_ents: %p, max_out: %" PRIu32 ", scratch: %p)",               ARG_AABB2F(box), mask, (void *)out_ents, max_out,  (void *)scratch);
    ASSERT(out_ents, "Attempted to query with NULL out_ents (grid: %p, box: " FMT_AABB2F ", mask: %" PRIu32 ", max_out: %" PRIu32 ", scratch: %p)",      (void *)grid, ARG_AABB2F(box), mask,                   max_out,  (void *)scratch);
    if (!grid || !out_ents || !max_out || !grid->entity_cnt || !grid->cell_offs || !grid->entries) return 0;

    uint64_t stack_buf[SPATIAL_STACK_WORDS];
    spatial_bitset_t bs = _spatial_bitset_begin(grid->entity_cnt, stack_buf, scratch);
    if (!bs.words) return 0;

    aabb2i   c           = _spatial_bounds_to_cells2d(box, grid->inv_cell_sz);
    uint32_t bucket_mask = grid->bucket_cnt - 1;
    uint32_t found       = 0;

    /* traverse all cells touched by query box */
    for (int32_t cy = c.min_y; cy <= c.max_y; ++cy)
        for (int32_t cx = c.min_x; cx <= c.max_x; ++cx) {
            uint32_t hash = _spatial_hash2d(cx, cy, bucket_mask);

            /* iterate ents in slice [cell_offs[h], cell_offs[h + 1]] */
            for (uint32_t idx = grid->cell_offs[hash]; idx < grid->cell_offs[hash + 1]; ++idx) {
                uint32_t e_idx = grid->entries[idx];

                /* deduplication check, update visited */
                if (_spatial_bitset_test_set(&bs, e_idx)) continue;

                /* layer bitmask check, where 0=match any */
                if (mask && !(grid->entities[e_idx].mask & mask)) continue;

                /* aabb check on remains */
                if (!aabb_inter(box, grid->entities[e_idx].bounds)) continue;

                out_ents[found++] = grid->entities[e_idx].ent;
                if (found == max_out) goto done;
            }
        }

done:
    _spatial_bitset_end(&bs);
    return found;
}


uint32_t spatial_query_radius2d(const spatial_grid2d_t *grid, vec2f center, float radius, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out, arena_t *scratch) {
    ASSERT(grid,     "Attempted to query radius with NULL grid (center: " FMT_VEC2F ", radius: %.2f, mask: %" PRIu32 ", out_ents: %p, max_out: %" PRIu32 ", scratch: %p)",                            ARG_VEC2F(center), radius, mask, (void *)out_ents, max_out, (void *)scratch);
    ASSERT(out_ents, "Attempted to query radius with NULL out_ents (grid: %p, center: " FMT_VEC2F ", radius: %.2f, mask: %" PRIu32 ", max_out: %" PRIu32 ", scratch: %p)",              (void *)grid, ARG_VEC2F(center), radius, mask,                   max_out, (void *)scratch);
    ASSERT(radius >= 0.0f, "Attempted to query with radius < 0.0f (grid: %p, center: " FMT_VEC2F ", radius: %.2f, mask: %" PRIu32 ", out_ents: %p, max_out: %" PRIu32 ", scratch: %p)", (void *)grid, ARG_VEC2F(center), radius, mask, (void *)out_ents, max_out, (void *)scratch);
    if (!grid || !out_ents || !max_out || radius < 0.0f || !grid->entity_cnt || !grid->entries || !grid->cell_offs || !grid->entities) return 0;

    uint64_t stack_buf[SPATIAL_STACK_WORDS];
    spatial_bitset_t bs = _spatial_bitset_begin(grid->entity_cnt, stack_buf, scratch);
    if (!bs.words) return 0;

    /* narrow down to aabb, then check radius intersection */
    aabb2f   box         = aabb_center_half(center, vec2f(radius, radius));
    aabb2i   c           = _spatial_bounds_to_cells2d(box, grid->inv_cell_sz);
    uint32_t bucket_mask = grid->bucket_cnt - 1;
    uint32_t found       = 0;

    for (int32_t cy = c.min_y; cy <= c.max_y; ++cy)
        for (int32_t cx = c.min_x; cx <= c.max_x; ++cx) {
            uint32_t hash = _spatial_hash2d(cx, cy, bucket_mask);

            for (uint32_t idx = grid->cell_offs[hash]; idx < grid->cell_offs[hash + 1]; ++idx) {
                uint32_t e_idx = grid->entries[idx];

                /* deduplication check, update visited */
                if (_spatial_bitset_test_set(&bs, e_idx)) continue;

                /* layer bitmask check, where 0=match any */
                if (mask && !(grid->entities[e_idx].mask & mask)) continue;

                /* distance check on remains */
                if (!aabb_inter_radius(grid->entities[e_idx].bounds, center, radius)) continue;

                out_ents[found++] = grid->entities[e_idx].ent;
                if (found == max_out) goto done;
            }
        }

done:
    _spatial_bitset_end(&bs);
    return found;
}

uint32_t spatial_query_point2d(const spatial_grid2d_t *grid, vec2f point, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out) {
    ASSERT(grid, "Attempted to query point with NULL grid (point: " FMT_VEC2F ", mask: %" PRIu32 ", out_ents: %p, max_out: %" PRIu32 ")",     ARG_VEC2F(point), mask,             (void *)out_ents, max_out);
    ASSERT(out_ents, "Attempted to query point with NULL out_ents (grid: %p, point: " FMT_VEC2F ", mask: %" PRIu32 ", max_out: %" PRIu32 ")", (void *)grid,     ARG_VEC2F(point), mask,             max_out);
    if (!grid || !out_ents || !max_out || !grid->entity_cnt || !grid->entries || !grid->cell_offs || !grid->entities) return 0;

    int32_t  cx          = _world_to_cell(point.x, grid->inv_cell_sz);
    int32_t  cy          = _world_to_cell(point.y, grid->inv_cell_sz);
    uint32_t bucket_mask = grid->bucket_cnt - 1;
    uint32_t hash        = _spatial_hash2d(cx, cy, bucket_mask);
    uint32_t found       = 0;

    /* iterate ents in slice [cell_offs[h], cell_offs[h + 1]] */
    /* to dedupe if point is on two cells that hash collide   */
    for (uint32_t idx = grid->cell_offs[hash]; idx < grid->cell_offs[hash + 1]; ++idx) {
        uint32_t e_idx = grid->entries[idx];

        /* layer bitmask check */
        if (mask && !(grid->entities[e_idx].mask & mask)) continue;

        /* point inside aabb check */
        if (!aabb2f_contains(grid->entities[e_idx].bounds, point)) continue;

        /* dedupe against col-aliased entries in same bucket */
        entity_handle_t ent = grid->entities[e_idx].ent;
        bool dup = false;
        for (uint32_t j = 0; j < found; ++j) if (out_ents[j] == ent) { dup = true; break; }
        if (dup) continue;

        out_ents[found++] = ent;
        if (found == max_out) return max_out;
    }

    return found;
}

/* ============================================================================================== */
/* >3D LOGIC IMPL                                                                                 */
/* ============================================================================================== */

res_t spatial_grid3d_init(spatial_grid3d_t *grid, float cell_sz, uint32_t bucket_cnt) {
    ASSERT(grid,                                                "Attempted to initialize a NULL spatial_grid3d");
    ASSERT(bucket_cnt == 0 || IS_POW2(bucket_cnt), "bucket_cnt must be a power of two (or 0 for default)");
    ASSERT(cell_sz >= 0.0f,                                     "cell_sz must be >0.0f (or 0.0f for default)");
    if   (!grid || (bucket_cnt && !IS_POW2(bucket_cnt)) || cell_sz < 0.0f) return EINVAL;

    memset(grid, 0, sizeof(*grid));
    grid->inv_cell_sz = 1.0f / ((cell_sz > 0.0f) ? cell_sz : SPATIAL_DEF_CELL_SZ);
    grid->bucket_cnt  = bucket_cnt ? bucket_cnt : SPATIAL_DEF_BUCKETS;

    return OK;
}

res_t spatial_grid3d_build(spatial_grid3d_t *grid, const spatial_entity3d_t *ents, uint32_t ent_cnt, arena_t *scratch) {
    ASSERT(grid, "Attempted to build with NULL grid (ents: %p, ent_cnt: %" PRIu32 ", scratch: %p)",    (void *)ents, ent_cnt,      (void *)scratch);
    ASSERT(scratch, "Attempted to build with NULL scratch (grid: %p, ents: %p, ent_cnt: %" PRIu32 ")", (void *)grid, (void *)ents, ent_cnt);
    if   (!grid || !scratch) return EINVAL;

    if (ent_cnt == 0 || !ents) {
        grid->entities   = NULL;
        grid->cell_offs  = NULL;
        grid->entries    = NULL;
        grid->entity_cnt = 0;
        grid->entry_cnt  = 0;
        return WEMPTY;
    }

    /* scratch-alloc cell_offs */
    grid->cell_offs = arena_alloc(scratch, (grid->bucket_cnt + 1) * sizeof(uint32_t));
    if (!grid->cell_offs) return EOOM;
    memset(grid->cell_offs, 0, (grid->bucket_cnt + 1) * sizeof(uint32_t));

    /* count cell overlaps into cell_offs[b + 1] */
    uint32_t mask          = grid->bucket_cnt - 1;
    uint32_t total_entries = 0;

    for (uint32_t i = 0; i < ent_cnt; ++i) {
        aabb3i c = _spatial_bounds_to_cells3d(ents[i].bounds, grid->inv_cell_sz);
        for (int32_t cz = c.min_z; cz <= c.max_z; ++cz)
            for (int32_t cy = c.min_y; cy <= c.max_y; ++cy)
                for (int32_t cx = c.min_x; cx <= c.max_x; ++cx) {
                    grid->cell_offs[_spatial_hash3d(cx, cy, cz, mask) + 1]++;
                    total_entries++;
                }
    }

    /* counts -> offsets */
    grid->cell_offs[0] = 0;
    for (uint32_t b = 1; b <= grid->bucket_cnt; ++b) grid->cell_offs[b] += grid->cell_offs[b - 1];

    /* scratch-alloc entries & cursor */
    grid->entries = arena_alloc(scratch, (size_t)total_entries * sizeof(uint32_t));
    if (total_entries && !grid->entries) return EOOM;

    /* temp to unwind cursor, it only lives for build process */
    arena_temp_t tmp = arena_temp_begin(scratch);
    uint32_t *cursor = arena_alloc(scratch, (size_t)grid->bucket_cnt * sizeof(uint32_t));
    if (!cursor) return EOOM;
    memcpy(cursor, grid->cell_offs, (size_t)grid->bucket_cnt * sizeof(uint32_t));

    /* scatter entity indices via _spatial_hash3d */
    for (uint32_t i = 0; i < ent_cnt; ++i) {
        aabb3i c = _spatial_bounds_to_cells3d(ents[i].bounds, grid->inv_cell_sz);
        for (int32_t cz = c.min_z; cz <= c.max_z; ++cz)
            for (int32_t cy = c.min_y; cy <= c.max_y; ++cy)
                for (int32_t cx = c.min_x; cx <= c.max_x; ++cx)
                    grid->entries[cursor[_spatial_hash3d(cx, cy, cz, mask)]++] = i;
    }

    arena_temp_end(tmp);

    grid->entities   = ents;
    grid->entity_cnt = ent_cnt;
    grid->entry_cnt  = total_entries;

    return OK;
}

res_t spatial_grid3d_build_from_ecs(spatial_grid3d_t *grid, ecs_world_t *world, arena_t *scratch) {
    ASSERT(grid,    "Attempted to build 3D grid from ECS with NULL grid (world: %p, scratch: %p)", (void *)world, (void *)scratch);
    ASSERT(scratch, "Attempted to build 3D grid from ECS with NULL scratch (grid: %p, world: %p)", (void *)grid,  (void *)world);
    if   (!grid || !scratch) return EINVAL;

    /* if world == NULL && s_active_world == NULL return to avoid assertion */
    ecs_world_t *w = world ? world : ecs_get_world();
    if (!w) return ENOTFND;

    /* set active world ctx if explicit world was passed */
    ecs_world_t *prev = ecs_get_world();
    if (world) ecs_set_world(world);

    ecs_view_t view = ecs_query_comp(trans3d, collider3d);
    if (!view.cnt || !view.ents) {
        if (world) ecs_set_world(prev); /* revert */
        return spatial_grid3d_build(grid, NULL, 0, scratch);
    }

    spatial_entity3d_t *ents = arena_alloc(scratch, view.cnt * sizeof(spatial_entity3d_t));
    if (!ents) {
        if (world) ecs_set_world(prev); /* revert */
        return EOOM;
    }

    for (uint32_t i = 0; i < view.cnt; ++i) {
        entity_handle_t   ent = view.ents[i];
        trans3d wt;
        ecs_get_world_trans(ent, &wt);
        const collider3d *col = ecs_get(ent, collider3d);

        vec3f pos   = vec3f(wt.x, wt.y, wt.z);
        vec3f scale = vec3f(wt.scale_x, wt.scale_y, wt.scale_z);
        vec3f off   = vec3f_mul(vec3f(col->off_x, col->off_y, col->off_z), scale);

        quatf q = quatf(wt.qx, wt.qy, wt.qz, wt.qw);
        mat3f r = quatf_to_mat3f(q);

        vec3f rot_off = vec3f(
            r.cols[0][0] * off.x + r.cols[1][0] * off.y + r.cols[2][0] * off.z,
            r.cols[0][1] * off.x + r.cols[1][1] * off.y + r.cols[2][1] * off.z,
            r.cols[0][2] * off.x + r.cols[1][2] * off.y + r.cols[2][2] * off.z
        );

        vec3f center = vec3f_add(pos, rot_off);

        vec3f half = vec3f_abs(vec_mul(vec3f(col->half_x, col->half_y, col->half_z), scale));

        /*    ex = |r00| * hx + |r01| * hy + |r02| * hz */
        /*    ey = |r10| * hx + |r11| * hy + |r12| * hz */
        /*    ez = |r20| * hx + |r21| * hy + |r22| * hz */
        vec3f extents = vec3f(
            ABS(r.cols[0][0]) * half.x + ABS(r.cols[1][0]) * half.y + ABS(r.cols[2][0]) * half.z,
            ABS(r.cols[0][1]) * half.x + ABS(r.cols[1][1]) * half.y + ABS(r.cols[2][1]) * half.z,
            ABS(r.cols[0][2]) * half.x + ABS(r.cols[1][2]) * half.y + ABS(r.cols[2][2]) * half.z
        );

        /* tight aabb = [cx - ex, cy - ey, cz - ez] -> [cx + ex, cy + ey, cz + ez] */
        ents[i] = (spatial_entity3d_t){
            .ent    = ent,
            .bounds = aabb_center_half(center, extents),
            .mask   = col->mask,
        };
    }

    if (world) ecs_set_world(prev); /* revert */
    return spatial_grid3d_build(grid, ents, view.cnt, scratch);
}

uint32_t spatial_query_aabb3d(const spatial_grid3d_t *grid, aabb3f box, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out, arena_t *scratch) {
    ASSERT(grid,     "Attempted to query AABB with NULL grid (box: " FMT_AABB3F ", mask: %" PRIu32 ", out_ents: %p, max_out: %" PRIu32 ", scratch: %p)", ARG_AABB3F(box), mask,            (void *)out_ents, max_out,  (void *)scratch);
    ASSERT(out_ents, "Attempted to query with NULL out_ents (grid: %p, box: " FMT_AABB3F ", mask: %" PRIu32 ", max_out: %" PRIu32 ", scratch: %p)",      (void *)grid,    ARG_AABB3F(box), mask,             max_out,  (void *)scratch);
    if (!grid || !out_ents || !max_out || !grid->entity_cnt || !grid->cell_offs || !grid->entries) return 0;

    uint64_t stack_buf[SPATIAL_STACK_WORDS];
    spatial_bitset_t bs = _spatial_bitset_begin(grid->entity_cnt, stack_buf, scratch);
    if (!bs.words) return 0;

    aabb3i   c           = _spatial_bounds_to_cells3d(box, grid->inv_cell_sz);
    uint32_t bucket_mask = grid->bucket_cnt - 1;
    uint32_t found       = 0;

    /* traverse all cells touched by query box */
    for (int32_t cz = c.min_z; cz <= c.max_z; ++cz)
        for (int32_t cy = c.min_y; cy <= c.max_y; ++cy)
            for (int32_t cx = c.min_x; cx <= c.max_x; ++cx) {
                uint32_t hash = _spatial_hash3d(cx, cy, cz, bucket_mask);

                /* iterate ents in slice [cell_offs[h], cell_offs[h + 1]] */
                for (uint32_t idx = grid->cell_offs[hash]; idx < grid->cell_offs[hash + 1]; ++idx) {
                    uint32_t e_idx = grid->entries[idx];

                    /* deduplication check, update visited */
                    if (_spatial_bitset_test_set(&bs, e_idx)) continue;

                    /* layer bitmask check, where 0=match any */
                    if (mask && !(grid->entities[e_idx].mask & mask)) continue;

                    /* aabb check on remains */
                    if (!aabb_inter(box, grid->entities[e_idx].bounds)) continue;

                    out_ents[found++] = grid->entities[e_idx].ent;
                    if (found == max_out) goto done;
                }
            }

done:
    _spatial_bitset_end(&bs);
    return found;
}


uint32_t spatial_query_radius3d(const spatial_grid3d_t *grid, vec3f center, float radius, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out, arena_t *scratch) {
    ASSERT(grid,     "Attempted to query radius with NULL grid (center: " FMT_VEC3F ", radius: %.2f, mask: %" PRIu32 ", out_ents: %p, max_out: %" PRIu32 ", scratch: %p)",                            ARG_VEC3F(center), radius, mask, (void *)out_ents, max_out, (void *)scratch);
    ASSERT(out_ents, "Attempted to query radius with NULL out_ents (grid: %p, center: " FMT_VEC3F ", radius: %.2f, mask: %" PRIu32 ", max_out: %" PRIu32 ", scratch: %p)",              (void *)grid, ARG_VEC3F(center), radius, mask,                   max_out, (void *)scratch);
    ASSERT(radius >= 0.0f, "Attempted to query with radius < 0.0f (grid: %p, center: " FMT_VEC3F ", radius: %.2f, mask: %" PRIu32 ", out_ents: %p, max_out: %" PRIu32 ", scratch: %p)", (void *)grid, ARG_VEC3F(center), radius, mask, (void *)out_ents, max_out, (void *)scratch);
    if (!grid || !out_ents || !max_out || radius < 0.0f || !grid->entity_cnt || !grid->entries || !grid->cell_offs || !grid->entities) return 0;

    uint64_t stack_buf[SPATIAL_STACK_WORDS];
    spatial_bitset_t bs = _spatial_bitset_begin(grid->entity_cnt, stack_buf, scratch);
    if (!bs.words) return 0;

    /* narrow down to aabb, then check radius intersection */
    aabb3f   box         = aabb_center_half(center, vec3f(radius, radius, radius));
    aabb3i   c           = _spatial_bounds_to_cells3d(box, grid->inv_cell_sz);
    uint32_t bucket_mask = grid->bucket_cnt - 1;
    uint32_t found       = 0;

    for (int32_t cz = c.min_z; cz <= c.max_z; ++cz)
        for (int32_t cy = c.min_y; cy <= c.max_y; ++cy)
            for (int32_t cx = c.min_x; cx <= c.max_x; ++cx) {
                uint32_t hash = _spatial_hash3d(cx, cy, cz, bucket_mask);

                for (uint32_t idx = grid->cell_offs[hash]; idx < grid->cell_offs[hash + 1]; ++idx) {
                    uint32_t e_idx = grid->entries[idx];

                    /* deduplication check, update visited */
                    if (_spatial_bitset_test_set(&bs, e_idx)) continue;

                    /* layer bitmask check, where 0=match any */
                    if (mask && !(grid->entities[e_idx].mask & mask)) continue;

                    /* distance check on remains */
                    if (!aabb_inter_radius(grid->entities[e_idx].bounds, center, radius)) continue;

                    out_ents[found++] = grid->entities[e_idx].ent;
                    if (found == max_out) goto done;
                }
            }

done:
    _spatial_bitset_end(&bs);
    return found;
}

uint32_t spatial_query_point3d(const spatial_grid3d_t *grid, vec3f point, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out) {
    ASSERT(grid, "Attempted to query point with NULL grid (point: " FMT_VEC3F ", mask: %" PRIu32 ", out_ents: %p, max_out: %" PRIu32 ")",     ARG_VEC3F(point), mask,             (void *)out_ents, max_out);
    ASSERT(out_ents, "Attempted to query point with NULL out_ents (grid: %p, point: " FMT_VEC3F ", mask: %" PRIu32 ", max_out: %" PRIu32 ")", (void *)grid,     ARG_VEC3F(point), mask,             max_out);
    if (!grid || !out_ents || !max_out || !grid->entity_cnt || !grid->entries || !grid->cell_offs || !grid->entities) return 0;

    int32_t  cx          = _world_to_cell(point.x, grid->inv_cell_sz);
    int32_t  cy          = _world_to_cell(point.y, grid->inv_cell_sz);
    int32_t  cz          = _world_to_cell(point.z, grid->inv_cell_sz);
    uint32_t bucket_mask = grid->bucket_cnt - 1;
    uint32_t hash        = _spatial_hash3d(cx, cy, cz, bucket_mask);
    uint32_t found       = 0;

    /* iterate ents in slice [cell_offs[h], cell_offs[h + 1]] */
    /* to dedupe if point is on two cells that hash collide   */
    for (uint32_t idx = grid->cell_offs[hash]; idx < grid->cell_offs[hash + 1]; ++idx) {
        uint32_t e_idx = grid->entries[idx];

        /* layer bitmask check */
        if (mask && !(grid->entities[e_idx].mask & mask)) continue;

        /* point inside aabb check */
        if (!aabb3f_contains(grid->entities[e_idx].bounds, point)) continue;

        /* dedupe against col-aliased entries in same bucket */
        entity_handle_t ent = grid->entities[e_idx].ent;
        bool dup = false;
        for (uint32_t j = 0; j < found; ++j) if (out_ents[j] == ent) { dup = true; break; }
        if (dup) continue;

        out_ents[found++] = ent;
        if (found == max_out) return max_out;
    }

    return found;
}

