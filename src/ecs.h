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
#include "gmath.h"
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

#define ECS_MAX_ENTITIES  65535U
#define ECS_NULL_IDX      0xFFFFU
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
table (`sparse`) and backpointers (`entities`).
====================
*/
typedef struct ecs_pool {
    void    *dense;
    uint32_t cnt;
    uint16_t elem_sz;
    uint16_t sparse[ECS_MAX_ENTITIES + 1];   /* ent_idx -> dense_idx, ECS_NULL_IDX inc. */
    uint16_t entities[ECS_MAX_ENTITIES + 1]; /* dense_idx -> ent_idx                    */
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
ecs_view_t

Scratch-allocated query match result.
Stores a contiguous array of matched entity handles (`ents`)
and match count (`cnt`). Allocated from `world->scratch` and
valid until the next frame scratch reset.
====================
*/
typedef struct ecs_view {
    entity_handle_t *ents;
    uint32_t         cnt;
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
ecs_world_snapshot

Captures a compact transactional snapshot
of the active world into `world->backup`.
Copies `world->mgr` and only the active
dense slice (`0, ..., cnt - 1`) of
non-empty component pools. Overwrites any
previous snapshot in `world->backup`.
Asserts active world and backup arena in
debug, returns result code in release.

Returns:
- `OK`     on successful snapshot capture.
- `EINVAL` if active world or `world->backup` is `NULL`.
- `EOOM`   if `world->backup` lacks capacity for the snapshot.
====================
*/
res_t ecs_world_snapshot(void);

/*
====================
ecs_world_rollback

Restores the active world from the
snapshot stored in `world->backup`.
Restores `world->mgr`, discards pending
deferred commands, restores active dense
component slices, and reconstructs
`sparse` lookup tables in O(cnt) time.
Preserves the snapshot in `world->backup`
so rollback can be called repeatedly.
Asserts active world and backup arena in debug,
returns result code in release.

Returns:
- `OK`      on successful state restoration.
- `EINVAL`  if active world or `world->backup` is `NULL`.
- `ENOTFND` if no valid snapshot exists in `world->backup`.
====================
*/
res_t ecs_world_rollback(void);

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

Queries living entities matching all components in `with_mask`
while excluding entities matching any component in `without_mask`
(1 to 64 components). Iterates over the smallest matching component
pool in `with_mask` to minimize traversals. Results are allocated
in `world->scratch`.

Returns an `ecs_view_t` containing matched entity handles.
====================
*/
ecs_view_t ecs_query_filter(uint64_t with_mask, uint64_t without_mask);

#define _ECS_MASK1(t1)        BIT(COMP_##t1)
#define _ECS_MASK2(t1, ...)  (BIT(COMP_##t1) | _ECS_MASK1(__VA_ARGS__))
#define _ECS_MASK3(t1, ...)  (BIT(COMP_##t1) | _ECS_MASK2(__VA_ARGS__))
#define _ECS_MASK4(t1, ...)  (BIT(COMP_##t1) | _ECS_MASK3(__VA_ARGS__))
#define _ECS_MASK5(t1, ...)  (BIT(COMP_##t1) | _ECS_MASK4(__VA_ARGS__))
#define _ECS_MASK6(t1, ...)  (BIT(COMP_##t1) | _ECS_MASK5(__VA_ARGS__))
#define _ECS_MASK7(t1, ...)  (BIT(COMP_##t1) | _ECS_MASK6(__VA_ARGS__))
#define _ECS_MASK8(t1, ...)  (BIT(COMP_##t1) | _ECS_MASK7(__VA_ARGS__))
#define _ECS_MASK9(t1, ...)  (BIT(COMP_##t1) | _ECS_MASK8(__VA_ARGS__))
#define _ECS_MASK10(t1, ...) (BIT(COMP_##t1) | _ECS_MASK9(__VA_ARGS__))
#define _ECS_MASK11(t1, ...) (BIT(COMP_##t1) | _ECS_MASK10(__VA_ARGS__))
#define _ECS_MASK12(t1, ...) (BIT(COMP_##t1) | _ECS_MASK11(__VA_ARGS__))
#define _ECS_MASK13(t1, ...) (BIT(COMP_##t1) | _ECS_MASK12(__VA_ARGS__))
#define _ECS_MASK14(t1, ...) (BIT(COMP_##t1) | _ECS_MASK13(__VA_ARGS__))
#define _ECS_MASK15(t1, ...) (BIT(COMP_##t1) | _ECS_MASK14(__VA_ARGS__))

/* variadic 1-16 component bitmask builder: ecs_mask(trans2d, sprite2d, ...) */
#define ecs_mask(...) VA_JOIN(_ECS_MASK, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

#define _ecs_query_mask1(with)                    ecs_query_filter((with), 0ULL)
#define _ecs_query_mask2(with, without)           ecs_query_filter((with), (without))
/* variadic 1-2 param query-by-mask:                           */
/* - if 1 arg  provided acts like ecs_query_filter(arg1, 0),   */
/* - if 2 args provided acts like ecs_query_filter(arg1, arg2) */
#define ecs_query_mask(...) VA_JOIN(_ecs_query_mask, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)

/* variadic 1-16 param query-by-component */
#define ecs_query_comp(...) ecs_query_mask(ecs_mask(__VA_ARGS__))

static inline void *ecs_get_fast_raw(const ecs_world_t *world, entity_handle_t e, comp_id_t id) {
    ASSERT(world,                                           "Attempted ecs_get_fast with NULL world (e: %" PRIu64 ", id: %d)", e, id);
    ASSERT((world->mgr.comp_mask[entity_idx(e)] & BIT(id)), "ecs_get_fast on unowned component (e: %" PRIu64 ", id: %d)", e, id);
    const ecs_pool_t *p = &world->pools[id];
    return (uint8_t *)p->dense + ((size_t)p->sparse[entity_idx(e)] * p->elem_sz);
}

/*
====================
ecs_get_fast

Unchecked O(1) component lookup in `world` for an entity `e`
already verified alive and owning component `id`.
Asserts valid world and component ownership in debug,
zero-branch in release.

Returns pointer to component memory in the dense pool.
====================
*/
#define ecs_get_fast(world, entity_handle, type) ((type *)ecs_get_fast_raw((world), (entity_handle), COMP_##type))

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

/* ============================================================================================== */
/* >HIERARCHY API                                                                                 */
/* ============================================================================================== */

/*
====================
ecs_set_parent

Attaches `child` as a direct child of `parent` in the scene forest.
If `parent == ENTITY_NULL`, detaches `child` via `ecs_remove_parent`.
Lazily attaches `hierarchy` components to `child` and `parent` if not
already owned. Walks `parent`s ancestor chain to reject cycles.
If `child` already has a parent, unlinks it from the previous parent first.
Asserts valid world and living handles in debug, returns result code in release.

Returns:
- `OK`     on successful parenting or no-op.
- `EINVAL` if active world is `NULL`, `child` or `parent` is dead, `child == parent`,
           or `child` is an ancestor of `parent` (cycle detected).
- `EOOM`   if the `hierarchy` component pool is exhausted.
====================
*/
res_t ecs_set_parent(entity_handle_t child, entity_handle_t parent);

/*
====================
ecs_remove_parent

Detaches `child` from its current parent so `child` becomes an
independent root of its own subtree (preserving all descendants
of `child`). No-op returning `OK` if `child` is already unparented.
Asserts valid world and living handle in debug, returns result
code in release.

Returns:
- `OK`     on successful detachment or if already unparented.
- `EINVAL` if active world is `NULL` or `child` is dead.
====================
*/
res_t ecs_remove_parent(entity_handle_t child);

bool ecs_get_world_trans2d(entity_handle_t e, trans2d *out);
bool ecs_get_world_trans3d(entity_handle_t e, trans3d *out);

/*
====================
ecs_get_world_trans

Resolves the world-space transform for entity `e` into `*out`.
If `e` is unparented, copies its `trans` directly.
If `e` is parented, walks up the ancestor chain and folds parent
transforms top-down (skipping organizational ancestor nodes that
do not own a transform).

Returns:
- `true`  if `e` is alive and owns a transform.
- `false` otherwise.
====================
*/
#define ecs_get_world_trans(e, out) _Generic((out),                                                \
    trans2d *: ecs_get_world_trans2d,                                                              \
    trans3d *: ecs_get_world_trans3d                                                               \
    )((e), (out))
