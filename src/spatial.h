#pragma once

/*
====================
spatial.h

2D and 3D spatial hashing broadphase subsystem.
Partitions unbounded world space into regular grid cells
mapped to a fixed-size hash table with power-of-two bucket count.
Features:
- deterministic 2-pass counting sort for contiguous bucket storage
- zero allocation during ticks via frame scratch arena allocations
====================
*/

#include "arena.h"
#include "core.h"
#include "ecs.h"
#include "gmath.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================================================== */
/* >CONSTANTS                                                                                     */
/* ============================================================================================== */

#define SPATIAL_DEF_CELL_SZ   64.0f
#define SPATIAL_DEF_BUCKETS   1024U
#define SPATIAL_MAX_CELL_SPAN 64U
#define SPATIAL_STACK_WORDS   32U /* 2048 entities */

/* ============================================================================================== */
/* >2D SPATIAL GRID                                                                               */
/* ============================================================================================== */

/*
====================
spatial_entity2d_t

Public 2D spatial entity proxy submitted for broadphase indexing.

Fields:
- `ent`:    ECS entity handle associated with this bounding volume
- `bounds`: axis-aligned bounding box in 2D world space
- `mask`:   layer bitmask for query filtering (0 matches all query masks)
====================
*/
typedef struct spatial_entity2d {
    entity_handle_t ent;
    aabb2f          bounds;
    uint32_t        mask; /* layer bitmask, 0=match all */
} spatial_entity2d_t;

/*
====================
spatial_grid2d_t

Managed 2D uniform spatial hash grid subsystem.
Partitions world space into uniform square cells
of size `1.0f / inv_cell_sz` and maps cell coords
`(cx, cy)` to `bucket_cnt` buckets via hash multiplication.
Bucket offsets and entity index entries are built
per-frame via a two-pass counting sort allocated
from scratch memory.

Invariants:
- `bucket_cnt` must be a power of two.
- Internal arrays (`cell_offs`, `entries`) are scratch-allocated.
- Borrows external entity arrays, zero persistent heap allocations.

Lifecycle:
- Initialized once via `spatial_grid2d_init`.
- Rebuilt per-frame via `spatial_grid2d_build` or `spatial_grid2d_build_from_ecs`.
====================
*/
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
res_t    spatial_grid2d_build_from_ecs(spatial_grid2d_t *grid, ecs_world_t *world, arena_t *scratch);
uint32_t spatial_query_aabb2d(const spatial_grid2d_t *grid, aabb2f box, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out, arena_t *scratch);
uint32_t spatial_query_radius2d(const spatial_grid2d_t *grid, vec2f center, float radius, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out, arena_t *scratch);
uint32_t spatial_query_point2d(const spatial_grid2d_t *grid, vec2f point, uint32_t mask, entity_handle_t *out_ents, uint32_t max_out);

/* ============================================================================================== */
/* >3D SPATIAL GRID                                                                               */
/* ============================================================================================== */

/*
====================
spatial_entity3d_t

Public 3D spatial entity proxy submitted for broadphase indexing.

Fields:
- `ent`:    ECS entity handle associated with this bounding volume.
- `bounds`: Axis-aligned bounding box in 3D world space.
- `mask`:   Layer bitmask for query filtering, where 0 matches all query masks.
====================
*/
typedef struct spatial_entity3d {
    entity_handle_t ent;
    aabb3f          bounds;
    uint32_t        mask;
} spatial_entity3d_t;

/*
====================
spatial_grid3d_t

Managed 3D uniform spatial hash grid subsystem.
Partitions world space into uniform cubic cells
of size `1.0f / inv_cell_sz` and maps cell
coordinates `(cx, cy, cz)` to `bucket_cnt`
buckets via 3D hash multiplication.
Bucket offsets and entity index entries are built
per-frame via a two-pass counting sort allocated
from scratch memory.

Invariants:
- `bucket_cnt` must be a power of two (asserted in debug, validated in `init`).
- Internal arrays (`cell_offs`, `entries`) are borrowed from scratch arena
  allocations and remain valid only until the next scratch unwind or frame reset.
- Borrows external entity arrays, zero persistent heap allocations.

Lifecycle:
- Initialized once via `spatial_grid_init`.
- Rebuilt per-frame via `spatial_grid_build` or `spatial_grid_build_from_ecs`.
====================
*/
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
/* >GENERIC WRAPPERS (PRIMARY PUBLIC API)                                                         */
/* ============================================================================================== */

/*
====================
spatial_grid_init

Initializes 2D or 3D spatial grid parameters based on grid pointer type.
If `cell_sz    == 0.0f`, defaults to `SPATIAL_DEF_CELL_SZ` (64.0f).
If `bucket_cnt == 0`,    defaults to `SPATIAL_DEF_BUCKETS` (1024).
Asserts valid non-null pointer, power-of-two bucket count, and
non-negative cell size in debug, returns error code in release.

Returns:
- `OK`     on successful initialization.
- `EINVAL` if `grid` is NULL, `cell_sz < 0.0f` or `bucket_cnt` is not a power of two.
====================
*/
#define spatial_grid_init(grid, cell_sz, bucket_cnt) _Generic((grid),                              \
    spatial_grid2d_t *: spatial_grid2d_init,                                                       \
    spatial_grid3d_t *: spatial_grid3d_init)                                                       \
    ((grid), (cell_sz), (bucket_cnt))

/*
====================
spatial_grid_build

Constructs 2D or 3D spatial hash grid over
proxy array `ents` via a 2-pass counting sort.
Pass 1 computes cell overlap counts per bucket.
Pass 2 scatters entity indices into the
contiguous `entries` array.
Allocates `cell_offs`, `entries` and temporary cursor from `scratch`.
Asserts non-null pointers in debug.
Borrows `ents` slice without copying.

Returns:
- `OK`     on successful grid construction.
- `EINVAL` if `grid` or `scratch` is `NULL`.
- `WEMPTY` if `ent_cnt == 0` or `ents` is `NULL` (clears internal grid pointers).
- `EOOM`   if `scratch` lacks memory for cell offsets, entry indices, or sort cursor.
====================
*/
#define spatial_grid_build(grid, ents, ent_cnt, scratch) _Generic((grid),                          \
    spatial_grid2d_t *: spatial_grid2d_build,                                                      \
    spatial_grid3d_t *: spatial_grid3d_build)                                                      \
    ((grid), (ents), (ent_cnt), (scratch))

/*
====================
spatial_grid_build_from_ecs

Extracts all active ECS entities owning position and collision components:
- 2D: Queries entities with both `trans2d` and `collider2d`.
- 3D: Queries entities with both `trans3d` and `collider3d`.
Computes tight world-space AABBs accounting for rotation
(trigonometric extents in 2D, quaternion rotation matrix in 3D),
scratch-allocates entity proxies, and builds the grid.
If `world` is `NULL`, uses the active world `ecs_get_world()`.
Asserts non-null grid and scratch in debug.

Returns:
- `OK`      on successful grid construction.
- `EINVAL`  if `grid` or `scratch` is `NULL`.
- `ENOTFND` if `world` is `NULL` and no active ECS world is bound.
- `WEMPTY`  if no matching entities exist in the queried world.
- `EOOM`    if `scratch` lacks capacity for entity proxies or grid storage.
====================
*/
#define spatial_grid_build_from_ecs(grid, world, scratch) _Generic((grid),                         \
    spatial_grid2d_t *: spatial_grid2d_build_from_ecs,                                             \
    spatial_grid3d_t *: spatial_grid3d_build_from_ecs)                                             \
    ((grid), (world), (scratch))

/*
====================
spatial_query_aabb

Queries spatial grid for all entities overlapping `box`
(`aabb2f` or `aabb3f`). Traverses all grid cells spanned
by `box` (clamped to `SPATIAL_MAX_CELL_SPAN`), filters by
layer `mask` (0 matches all), checks exact AABB intersection,
and deduplicates multi-cell entity overlaps using:
- for  <2048 entities 2048-bit stack-allocated buffer,
- for >=2048 entities scratch-allocated        buffer.
Asserts non-null grid and `out_ents` in debug,
returns `0` in release.

Returns count of unique matching entity handles
written to `out_ents` (up to `max_out`).
====================
*/
#define spatial_query_aabb(grid, box, mask, out_ents, max_out, scratch) _Generic((grid),           \
    spatial_grid2d_t *:       spatial_query_aabb2d,                                                \
    const spatial_grid2d_t *: spatial_query_aabb2d,                                                \
    spatial_grid3d_t *:       spatial_query_aabb3d,                                                \
    const spatial_grid3d_t *: spatial_query_aabb3d)                                                \
    ((grid), (box), (mask), (out_ents), (max_out), (scratch))

/*
====================
spatial_query_radius

Queries spatial grid for all entities overlapping
a sphere or circle defined by `center` and `radius`.
Computes the bounding box of the sphere/circle to
hash candidate cells, filters by layer `mask`
(0 matches all), tests exact distance using Jim Arvo's
algorithm (`aabb_inter_radius`), and deduplicates
multi-cell entity overlaps using a bitset.
Asserts non-null pointers and `radius >= 0.0f` in debug,
returns 0 in release.

Returns count of unique matching entity handles
written to `out_ents` (up to `max_out`).
====================
*/
#define spatial_query_radius(grid, center, radius, mask, out_ents, max_out, scratch) _Generic((grid),\
    spatial_grid2d_t *:       spatial_query_radius2d,                                              \
    const spatial_grid2d_t *: spatial_query_radius2d,                                              \
    spatial_grid3d_t *:       spatial_query_radius3d,                                              \
    const spatial_grid3d_t *: spatial_query_radius3d)                                              \
    ((grid), (center), (radius), (mask), (out_ents), (max_out), (scratch))

/*
====================
spatial_query_point

Queries spatial grid for all entities containing
world point `point` (`vec2f` or `vec3f`).
Hashes the single cell containing `point`,
filters by layer `mask` (0 matches all), checks exact
point containment (`aabb2f_contains` or `aabb3f_contains`),
and deduplicates linear results against bucket hash
collisions without requiring bitset or scratch allocations.
Asserts non-null pointers in debug, returns 0 in release.

Returns count of unique matching entity handles
written to `out_ents` (up to `max_out`).
====================
*/
#define spatial_query_point(grid, point, mask, out_ents, max_out) _Generic((grid),                 \
    spatial_grid2d_t *: spatial_query_point2d,                                                     \
    const spatial_grid2d_t *: spatial_query_point2d,                                               \
    spatial_grid3d_t *: spatial_query_point3d,                                                     \
    const spatial_grid3d_t *: spatial_query_point3d)                                               \
    ((grid), (point), (mask), (out_ents), (max_out))
