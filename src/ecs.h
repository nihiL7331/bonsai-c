#pragma once

/*
====================
ecs.h

64K-capped generational sparse-set ECS.
Features:
- 64-bit generational entity handles [32b gen | 16b flags | 16b idx]
- O(1) component access and dense contiguous memory for cache efficiency
- Deferred structural mutations flushed at frame boundaries via `ecs_sync`
- Fast bitmask filtering across component sparse sets
====================
*/

#include "arena.h"
#include "core.h"
#include "reflect.h"

#include <stdbool.h>
#include <stdint.h>

/*
====================
entity_handle_t

64-bit generational entity handle.
Layout:
- bits 32..63: 32-bit generation counter (`entity_gen`)
- bits 16..31: 16-bit status flags       (`entity_flags`), e.g. `ENTITY_FLAG_DEAD`
- bits  0..15: 16-bit entity index       (`entity_idx`),   max  `ECS_MAX_ENTITIES` (65535)
====================
*/
typedef uint64_t entity_handle_t;

#define ECS_MAX_ENTITIES  65535U

#define ECS_NULL_IDX      0xFFFFU
#define ENTITY_NULL       ((entity_handle_t)0ULL)
#define ENTITY_FLAG_DEAD  (1U << 0)

#define ENTITY_GEN_MASK   0xFFFFFFFF00000000ULL
#define ENTITY_FLAGS_MASK 0x00000000FFFF0000ULL
#define ENTITY_IDX_MASK   0x000000000000FFFFULL

#define entity_gen(h)   (uint32_t)(((h) & ENTITY_GEN_MASK)   >> 32)
#define entity_flags(h) (uint16_t)(((h) & ENTITY_FLAGS_MASK) >> 16)
#define entity_idx(h)   (uint16_t)( (h) & ENTITY_IDX_MASK)

#define entity_handle(gen, flags, idx) ((entity_handle_t)                                          \
    ((((entity_handle_t)(gen)   << 32) & ENTITY_GEN_MASK)   |                                      \
     (((entity_handle_t)(flags) << 16) & ENTITY_FLAGS_MASK) |                                      \
     (((entity_handle_t)(idx)        ) & ENTITY_IDX_MASK)))

#define ECS_CMD_INIT_CAP 256UL

/* ============================================================================================== */
/* >STRUCTS                                                                                       */
/* ============================================================================================== */

typedef enum ecs_cmd_type {
    ECS_CMD_DESTROY_ENT,
    ECS_CMD_DELETE_COMP,
} ecs_cmd_type_t;

/*
====================
ecs_cmd_t

Deferred structural mutation command.
Queued during ticks and flushed at `ecs_sync()`
to guarantee pointer stability and prevent
pool invalidation during iterations.
Stores:
- command type `type` (destroy entity or remove component)
- target       `ent`  (entity handle to modify)
- component ID `comp` (component ID for `ECS_CMD_DELETE_COMP`)
====================
*/
typedef struct ecs_cmd {
    ecs_cmd_type_t  type;
    entity_handle_t ent;
    comp_id_t       comp; /* for ECS_CMD_DELETE_COMP */
} ecs_cmd_t;

/*
====================
ecs_pool_t

Sparse-set component storage.
Maintains contiguous POD memory (`dense`) for packed
iteration, mapped bidirectionally via a 64K lookup
table (`sparse`) and backpointers (`entitites`).
====================
*/
typedef struct ecs_pool {
    void    *dense;
    uint32_t cnt;
    uint16_t elem_sz;
    uint16_t sparse[ECS_MAX_ENTITIES + 1];   /* ent_idx -> dense_idx, ECS_NULL_IDX inc. */
    uint16_t entities[ECS_MAX_ENTITIES + 1]; /* dense_idx -> ent_idx                    */
    bool     dirty;
} ecs_pool_t;

/*
====================
entity_mgr_t

Flat entity lifecycle and generational index allocator.
Manages up to `ECS_MAX_ENTITIES` (65535) simultaneous
entities without heap allocations.
Stores:
- component masks `comp_mask` (64-bit component ownership bitmask per entity index)
- generations     `gens`      (32-bit generation counters to detect stale/recycled handles)
- free queue      `free_list` (LIFO queue of recyclable entity indices)
- free head       `free_hd`   (index of the top element in `free_list`)
- living count    `alive_cnt` (count of currently living entities)
- entity flags    `flags`     (per-entity status bitmask, e.g. `ENTITY_FLAG_DEAD`)
====================
*/
typedef struct entity_mgr {
    uint64_t comp_mask[ECS_MAX_ENTITIES + 1];
    uint32_t gens[ECS_MAX_ENTITIES + 1];
    uint32_t free_hd;
    uint32_t alive_cnt;
    uint16_t free_list[ECS_MAX_ENTITIES + 1];
    uint16_t flags[ECS_MAX_ENTITIES + 1];
} entity_mgr_t;

/*
====================
ecs_world_t

Central ECS state container.
Holds entity generation counters, component masks,
dense/sparse pools, and deferred mutation command
buffers. Backed by three arenas:
- `perm`:    Persistent entity tables and component dense memory
- `backup`:  Transactional mirror for rollback / state snapshots
- `scratch`: Per-frame command buffers and query view allocations
====================
*/
typedef struct ecs_world {
    arena_t      *perm;    /* permanent lifetime */
    arena_t      *backup;  /* mirror of 'perm'   */
    arena_t      *scratch; /* per-frame          */
    entity_mgr_t mgr;
    ecs_pool_t   pools[COMP_CNT];
    ecs_cmd_t   *cmd_buf; /* scratch-alloced */
    uint32_t     cmd_cnt;
    uint32_t     cmd_cap;
} ecs_world_t;

/*
====================
ecs_view_row_t

Single entity match row in an `ecs_view_t`.
Stores:
- entity handle `ent`        (generational handle of matched entity)
- dense indices `dense_idxs` (dense array indices for matched components, up to 4)
====================
*/
typedef struct ecs_view_row {
    entity_handle_t ent;
    uint16_t        dense_idxs[4];
} ecs_view_row_t;

/*
====================
ecs_view_t

Scratch-allocated query match result.
Stores matched entity handles and dense component pool indices.
Allocated from `world->scratch` and lives until the next frame scratch reset.
====================
*/
typedef struct ecs_view {
    ecs_view_row_t *rows;
    uint32_t        cnt;
} ecs_view_t;

/* ============================================================================================== */
/* >API                                                                                           */
/* ============================================================================================== */

/*
====================
ecs_world_create

Allocates and initializes an ECS world within the provided permanent `perm` arena.
Sets active world context if no active world exists.

Returns:
- pointer to the initialized `ecs_world_t` on success.
- `NULL` on allocation failure.
====================
*/
ecs_world_t *ecs_world_create(arena_t *perm, arena_t *backup, arena_t *scratch);

/*
====================
ecs_set_world

Sets the global active ECS world context (`s_active_world`).
Asserts `world->perm` and `world->scratch` are non-null in debug.
Allows `ecs_set_world(NULL)` to set `s_active_world` to `NULL`.
====================
*/
void ecs_set_world(ecs_world_t *world);

/*
====================
ecs_get_world

Retrieves the currently active global ECS world context (`s_active_world`).

Returns:
- pointer to active `ecs_world_t` on success.
- `NULL` if `s_active_world` is uninitialized.
====================
*/
ecs_world_t *ecs_get_world(void);

/*
====================
ecs_spawn

Reserves a new entity handle from the active world's free list.
Increments generation count and marks entity alive.
Asserts active world and available capacity.

Returns:
- a valid living `entity_handle_t` on success.
- `ENTITY_NULL` on capacity exhaustion.
====================
*/
entity_handle_t ecs_spawn(void);

/*
====================
ecs_find_tag

Searches for the first living entity possessing
a `tag` component matching `tag`.
Checks `tag != NULL` and active world.

Returns:
- handle of matching living entity on success.
- `ENTITY_NULL` if not found.
====================
*/
entity_handle_t ecs_find_tag(const char *tag);

/*
====================
ecs_destroy

Marks entity as dead immediately (`ENTITY_FLAG_DEAD`)
and queues structural cleanup for the next `ecs_sync()` call.
Asserts entity is alive.
====================
*/
void ecs_destroy(entity_handle_t e);

/*
====================
ecs_is_alive

Validates whether entity handle `e` is living by
matching handle generation against `world->mgr.gens`
and verifying `ENTITY_FLAG_DEAD` is clear.

Returns:
- `true` if handle matches a living entity.
- `false` otherwise.
====================
*/
bool ecs_is_alive(entity_handle_t e);

/*
====================
ecs_sync

Flushes all deferred deletion commands from the scratch
command buffer. Executes swap-and-pop component removals,
recycles dead entity indices into the free list, increments
generation counters, and resets command buffers.
====================
*/
void ecs_sync(void);

/*
====================
ecs_query_filter

Queries entities matching all components in `with_mask` while
excluding entities matching any component in `without_mask`.
Up to 4 components per query.
Iterates over the smallest matching component pool to minimize traversals.
Results are allocated in `world->scratch`.

Returns an `ecs_view_t` containing matched entity handles and dense row indices.
====================
*/
ecs_view_t ecs_query_filter(uint64_t with_mask, uint64_t without_mask);

#define    _ecs_query_mask1(with)                            ecs_query_filter((with), 0ULL)
#define    _ecs_query_mask2(with, without)                   ecs_query_filter((with), (without))
#define    _ecs_query_mask_select(_1, _2, NAME, ...)         NAME
/* variadic 1-2 param query-by-mask:                           */
/* - if 1 arg  provided acts like ecs_query_filter(arg1, 0),   */
/* - if 2 args provided acts like ecs_query_filter(arg1, arg2) */
#define    ecs_query_mask(...)                               _ecs_query_mask_select(__VA_ARGS__, _ecs_query_mask2, _ecs_query_mask1, 0)(__VA_ARGS__)

#define    _ecs_query_comp1(t1)                              ecs_query_mask(BIT(COMP_##t1))
#define    _ecs_query_comp2(t1, t2)                          ecs_query_mask(BIT(COMP_##t1) | BIT(COMP_##t2))
#define    _ecs_query_comp3(t1, t2, t3)                      ecs_query_mask(BIT(COMP_##t1) | BIT(COMP_##t2) | BIT(COMP_##t3))
#define    _ecs_query_comp4(t1, t2, t3, t4)                  ecs_query_mask(BIT(COMP_##t1) | BIT(COMP_##t2) | BIT(COMP_##t3) | BIT(COMP_##t4))
#define    _ecs_query_comp_select(_1, _2, _3, _4, NAME, ...) NAME
/* variadic 1-4 param query-by-component, returns dense idxs for entities matching */
#define    ecs_query_comp(...)                               _ecs_query_comp_select(__VA_ARGS__, _ecs_query_comp4, _ecs_query_comp3, _ecs_query_comp2, _ecs_query_comp1, 0)(__VA_ARGS__)

/*
====================
ecs_get_raw

Retrieves direct pointer to component data in the
dense pool for entity `e`. Fails fast in release.

Returns:
- pointer to component memory on success.
- `NULL` if entity is dead or does not own component.
====================
*/
void *ecs_get_raw(entity_handle_t e, comp_id_t id);

/*
====================
ecs_has_raw

Checks whether entity `e` is currently alive and owns component `id`.

Returns:
- `true`  if entity is alive and owns component.
- `false` otherwise.
====================
*/
bool  ecs_has_raw(entity_handle_t e, comp_id_t id);

/*
====================
ecs_add_raw

Attaches component `id` to entity `e`.
If entity already owns the component, updates in-place.
If `val == NULL`, initializes memory from reflection defaults (`meta->def_val`).
Asserts entity is alive and pool capacity is sufficient.

Returns:
- pointer to attached component memory in the dense pool if succeeded.
- `NULL` on failure.
====================
*/
void *ecs_add_raw(entity_handle_t e, comp_id_t id, const void *val);

/*
====================
ecs_del_raw

Queues a deferred removal of component `id` from
entity `e` for the next `ecs_sync()`.
Asserts entity is alive and component ID is valid.
====================
*/
void  ecs_del_raw(entity_handle_t e, comp_id_t id);

#define ecs_get(entity_handle, type)      ((type *)ecs_get_raw((entity_handle), COMP_##type))
#define ecs_has(entity_handle, type)               ecs_has_raw((entity_handle), COMP_##type)
#define ecs_add(entity_handle, type, val) ((type *)ecs_add_raw((entity_handle), COMP_##type, (val)))
#define ecs_del(entity_handle, type)               ecs_del_raw((entity_handle), COMP_##type)
