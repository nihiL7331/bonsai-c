#pragma once

#include "arena.h"
#include "core.h"
#include "ecs.h"
#include "gmath.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SPATIAL_DEF_CELL_SZ   64.0f
#define SPATIAL_DEF_BUCKETS   1024U
#define SPATIAL_MAX_CELL_SPAN 64U
#define SPATIAL_STACK_WORDS   32U /* 2048 entities */

/* ============================================================================================== */
/* >2D SPATIAL GRID                                                                               */
/* ============================================================================================== */

typedef struct spatial_entity2d {
    entity_handle_t ent;
    aabb2f          bounds;
    uint32_t        mask; /* layer bitmask, 0=match all */
} spatial_entity2d_t;

typedef struct spatial_grid2d {
    const spatial_entity2d_t *entities;
    uint32_t                 *cell_offs; /* sz: bucket_cnt + 1, scratch-alloced */
    uint32_t                 *entries;   /* sz: entry_cnt,      scratch-alloced */
    uint32_t                  entity_cnt;
    uint32_t                  entry_cnt;
    float                     inv_cell_sz;
    uint32_t                  bucket_cnt;  /* invariant: exists n in NN . bucket_cnt = 2^n */
} spatial_grid2d_t;

res_t    spatial_grid2d_init(spatial_grid2d_t *grid, float cell_sz, uint32_t bucket_cnt);
res_t    spatial_grid2d_build(spatial_grid2d_t *grid, const spatial_entity2d_t *ents, uint32_t ent_cnt, arena_t *scratch);
/* if world==NULL uses s_active_world */
res_t    spatial_grid2d_build_from_ecs(spatial_grid2d_t *grid, ecs_world_t *world, arena_t *scratch);
uint32_t spatial_query_aabb2d(const spatial_grid2d_t *grid, aabb2f box, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out, arena_t *scratch);
uint32_t spatial_query_radius2d(const spatial_grid2d_t *grid, vec2f center, float radius, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out, arena_t *scratch);
uint32_t spatial_query_point2d(const spatial_grid2d_t *grid, vec2f point, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out);

/* ============================================================================================== */
/* >3D SPATIAL GRID                                                                               */
/* ============================================================================================== */

typedef struct spatial_entity3d {
    entity_handle_t ent;
    aabb3f          bounds;
    uint32_t        mask;
} spatial_entity3d_t;

typedef struct spatial_grid3d {
    const spatial_entity3d_t *entities;
    uint32_t                 *cell_offs; /* sz: bucket_cnt + 1, scratch-alloced */
    uint32_t                 *entries;   /* sz: entry_cnt,      scratch-alloced */
    uint32_t                  entity_cnt;
    uint32_t                  entry_cnt;
    float                     inv_cell_sz;
    uint32_t                  bucket_cnt;  /* invariant: exists n in NN . bucket_cnt = 2^n */
} spatial_grid3d_t;

res_t    spatial_grid3d_init(spatial_grid3d_t *grid, float cell_sz, uint32_t bucket_cnt);
res_t    spatial_grid3d_build(spatial_grid3d_t *grid, const spatial_entity3d_t *ents, uint32_t ent_cnt, arena_t *scratch);
/* if world==NULL uses s_active_world */
res_t    spatial_grid3d_build_from_ecs(spatial_grid3d_t *grid, ecs_world_t *world, arena_t *scratch);
uint32_t spatial_query_aabb3d(const spatial_grid3d_t *grid, aabb3f box, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out, arena_t *scratch);
uint32_t spatial_query_radius3d(const spatial_grid3d_t *grid, vec3f center, float radius, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out, arena_t *scratch);
uint32_t spatial_query_point3d(const spatial_grid3d_t *grid, vec3f  point, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out);

/* ============================================================================================== */
/* >GENERIC WRAPPERS                                                                              */
/* ============================================================================================== */

#define spatial_grid_init(grid, cell_sz, bucket_cnt)      _Generic((grid), spatial_grid2d_t *: spatial_grid2d_init,           spatial_grid3d_t *: spatial_grid3d_init)((grid), (cell_sz), (bucket_cnt))
#define spatial_grid_build(grid, ents, ent_cnt, scratch)  _Generic((grid), spatial_grid2d_t *: spatial_grid2d_build,          spatial_grid3d_t *: spatial_grid3d_build)((grid), (ents), (ent_cnt), (scratch))
#define spatial_grid_build_from_ecs(grid, world, scratch) _Generic((grid), spatial_grid2d_t *: spatial_grid2d_build_from_ecs, spatial_grid3d_t *: spatial_grid3d_build_from_ecs)((grid), (world), (scratch))
#define spatial_query_aabb(grid, box, mask, out_ents, max_out, scratch)                            \
    _Generic((grid),                                                                               \
    spatial_grid2d_t *: spatial_query_aabb2d, const spatial_grid2d_t *: spatial_query_aabb2d,      \
    spatial_grid3d_t *: spatial_query_aabb3d, const spatial_grid3d_t *: spatial_query_aabb3d)      \
    ((grid), (box), (mask), (out_ents), (max_out), (scratch))
#define spatial_query_radius(grid, center, radius, mask, out_ents, max_out, scratch)               \
    _Generic((grid),                                                                               \
    spatial_grid2d_t *: spatial_query_radius2d, const spatial_grid2d_t *: spatial_query_radius2d,  \
    spatial_grid3d_t *: spatial_query_radius3d, const spatial_grid3d_t *: spatial_query_radius3d)  \
    ((grid), (center), (radius), (mask), (out_ents), (max_out), (scratch))
#define spatial_query_point(grid, point, mask, out_ents, max_out)                                  \
    _Generic((grid),                                                                               \
    spatial_grid2d_t *: spatial_query_point2d, const spatial_grid2d_t *: spatial_query_point2d,    \
    spatial_grid3d_t *: spatial_query_point3d, const spatial_grid3d_t *: spatial_query_point3d)    \
    ((grid), (point), (mask), (out_ents), (max_out))
