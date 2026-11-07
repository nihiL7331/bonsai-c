/*
====================
ecs.c

Sparse-set ECS implementation.
Implements generational entity recycling,
swap-and-pop dense array maintenance,
deferred command queueing, and
multi-component query evaluation.
====================
*/

#include "ecs.h"

#include <stdint.h>
#include <string.h>
#include <inttypes.h>

static ecs_world_t *s_active_world = NULL;

/* ============================================================================================== */
/* >WORLD API                                                                                     */
/* ============================================================================================== */

/* Seeds the entity free list and marks generation 1 for all usable indices */
static inline void _ecs_world_entity_mgr_init(entity_mgr_t *mgr) {
    mgr->gens[0] = 0UL;
    mgr->free_hd = ECS_MAX_ENTITIES;
    for (size_t i = 0; i < ECS_MAX_ENTITIES; ++i) {
        mgr->gens[i + 1]  = 1UL;                              /* ENTITY_NULL,     1,      1,     1,      ... */
        mgr->free_list[i] = (uint16_t)(ECS_MAX_ENTITIES - i); /* ECS_MAX_ENTITIES, ECS_MAX_ENTITIES - 1, ... */
    }
}

/* Allocates contiguous dense memory per component pool and clears sparse tables to 0xFFFF */
static inline void _ecs_world_pools_init(ecs_pool_t *pools, arena_t *perm) {
    for (size_t i = 0; i < COMP_CNT; ++i) {
        const comp_meta_t *meta = reflect_comp_by_id((comp_id_t)i);
        pools[i].elem_sz = (uint16_t)(meta ? meta->size : 0U);
        pools[i].dense = arena_alloc(perm, pools[i].elem_sz * (ECS_MAX_ENTITIES + 1U));
        memset(pools[i].sparse, 0xFFU, sizeof(pools[i].sparse));
    }
}

ecs_world_t *ecs_world_create(arena_t *perm, arena_t *backup, arena_t *scratch) {
    ecs_world_t *world = (ecs_world_t *)arena_alloc(perm, sizeof(ecs_world_t));
    ASSERT(world, "Failed to allocate world to perm arena (%p)", perm);
    if   (!world) return NULL;

    memset(world, 0, sizeof(ecs_world_t));

    world->perm    = perm;
    world->backup  = backup;
    world->scratch = scratch;
    _ecs_world_entity_mgr_init(&world->mgr);
    _ecs_world_pools_init(world->pools, perm);

    if (!s_active_world) s_active_world = world;
    return world;
}

void ecs_set_world(ecs_world_t *world) {
    if (world) ASSERT(world->perm && world->scratch, "world missing required arena pointers");
    s_active_world = world;
}

ecs_world_t *ecs_get_world(void) {
    return s_active_world;
}

/* ============================================================================================== */
/* >ECS LIFECYCLE API                                                                             */
/* ============================================================================================== */

/* Pushes deferred mutation commands into scratch arena, doubling capacity on full */
static inline void _ecs_cmd_push(ecs_cmd_t cmd) {
    ecs_world_t *world = ecs_get_world(); /* invariant held on _ecs_cmd_push call is world != NULL */

    if (world->cmd_cnt >= world->cmd_cap) {
        uint32_t   new_cap = (!world->cmd_cap) ? ECS_CMD_INIT_CAP : (world->cmd_cap * 2UL);
        ecs_cmd_t *new_buf = (ecs_cmd_t *)arena_alloc(world->scratch, sizeof(ecs_cmd_t) * new_cap);
        ASSERT(new_buf, "Scratch arena ran out of memory for ECS cmd buffer");
        if   (!new_buf) return;

        if (world->cmd_buf && world->cmd_cnt) memcpy(new_buf, world->cmd_buf, sizeof(ecs_cmd_t) * world->cmd_cnt);
        world->cmd_buf = new_buf;
        world->cmd_cap = new_cap;
    }

    world->cmd_buf[world->cmd_cnt++] = cmd;
}

/* O(1) swap and pop removal from dense pool, copies last element over deleted and repairs sparse tab */
static inline void _ecs_pool_remove(ecs_world_t *world, uint16_t e_idx, comp_id_t comp) {
    ecs_pool_t *p     = &world->pools[comp];
    uint16_t    d_idx = p->sparse[e_idx];
    if (d_idx == ECS_NULL_IDX || d_idx >= p->cnt) return;

    uint32_t last_d_idx = p->cnt - 1UL;

    /* if not last elem in dense, swap with last */
    if (d_idx != last_d_idx) {
        uint16_t last_e_idx = p->entities[last_d_idx];
        void *dst = (uint8_t *)p->dense + ((size_t)d_idx      * p->elem_sz);
        void *src = (uint8_t *)p->dense + ((size_t)last_d_idx * p->elem_sz);
        memcpy(dst, src, p->elem_sz);

        p->entities[d_idx]    = last_e_idx;
        p->sparse[last_e_idx] = d_idx;
    }

    world->mgr.comp_mask[e_idx] &= ~BIT(comp);
    p->sparse[e_idx]             = ECS_NULL_IDX;
    p->dirty                     = true;
    p->cnt--;
}

entity_handle_t ecs_spawn(void) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world, "Attempted to spawn with NULL active world");
    if   (!world) return ENTITY_NULL;

    entity_mgr_t *mgr = &world->mgr;
    ASSERT(mgr->free_hd, "ECS entity pool exhausted, max ECS_MAX_ENTITIES (%d) entities reached.", ECS_MAX_ENTITIES);
    if   (!mgr->free_hd) return ENTITY_NULL;

    uint16_t idx        = mgr->free_list[--mgr->free_hd];
    mgr->flags[idx]     = 0UL;
    mgr->comp_mask[idx] = 0ULL;
    mgr->alive_cnt++;

    return entity_handle(mgr->gens[idx], 0UL, idx);
}

entity_handle_t ecs_find_tag(const char *tag) {
    ASSERT(tag, "Attempted to find_tag with NULL tag");
    if (!tag) return ENTITY_NULL;

    ecs_world_t *world = ecs_get_world();
    ASSERT(world, "Attempted to find_tag with NULL active world (%s)", tag);
    if   (!world) return ENTITY_NULL;

    ecs_pool_t *p = &world->pools[COMP_tag];
    for (uint32_t i = 0; i < p->cnt; ++i) {
        const struct tag *t = (const struct tag *)((const uint8_t *)p->dense + (size_t)i * p->elem_sz);

        if (strncmp((const char *)t->name, tag, 32) == 0) {
            uint16_t idx = p->entities[i];
            if (!(world->mgr.flags[idx] & ENTITY_FLAG_DEAD))
                return entity_handle(world->mgr.gens[idx], world->mgr.flags[idx], idx);
        }
    }

    return ENTITY_NULL;
}

void ecs_destroy(entity_handle_t e) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world,           "Attempted to destroy with NULL active world (%" PRIu64 ")", e);
    ASSERT(ecs_is_alive(e), "Attempted to destroy a dead entity (%" PRIu64 ")",          e);
    if (!world || !ecs_is_alive(e)) return;

    uint16_t idx           = entity_idx(e);
    world->mgr.flags[idx] |= ENTITY_FLAG_DEAD;

    _ecs_cmd_push((ecs_cmd_t){ .type = ECS_CMD_DESTROY_ENT, .ent = e, .comp = 0 });
}

bool ecs_is_alive(entity_handle_t e) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world, "Attempted to check is_alive with NULL active world (%" PRIu64 ")", e);
    if   (!world || e == ENTITY_NULL) return false;

    uint16_t idx = entity_idx(e);
    if (idx == 0U) return false;

    entity_mgr_t *mgr = &world->mgr;

    return entity_gen(e) == mgr->gens[idx] && !(mgr->flags[idx] & ENTITY_FLAG_DEAD);
}

void ecs_sync(void) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world, "Attempted sync with NULL active world");
    if (!world || !world->cmd_cnt) return;

    entity_mgr_t *mgr = &world->mgr;

    for (uint32_t i = 0UL; i < world->cmd_cnt; ++i) {
        ecs_cmd_t cmd   = world->cmd_buf[i];
        uint16_t  e_idx = entity_idx(cmd.ent);

        /* guard against stale commands from prior gens */
        if (entity_gen(cmd.ent) != mgr->gens[e_idx]) continue;

        if      (cmd.type == ECS_CMD_DELETE_COMP) _ecs_pool_remove(world, e_idx, cmd.comp);
        else if (cmd.type == ECS_CMD_DESTROY_ENT) {
            /* remove all comps owned by this entity */
            uint64_t mask = mgr->comp_mask[e_idx];
            while (mask) {
                _ecs_pool_remove(world, e_idx, (comp_id_t)__builtin_ctzll(mask));
                mask &= mask - 1ULL;
            }

            mgr->flags[e_idx]     = 0U;
            mgr->comp_mask[e_idx] = 0ULL;

            mgr->gens[e_idx]++;
            if (mgr->gens[e_idx] == 0UL) mgr->gens[e_idx] = 1UL;

            mgr->free_list[mgr->free_hd++] = e_idx;
            mgr->alive_cnt--;
        }
    }

    world->cmd_cnt = 0UL;
    world->cmd_cap = 0UL;
    world->cmd_buf = NULL;
}

/* ============================================================================================== */
/* >QUERY API                                                                                     */
/* ============================================================================================== */

ecs_view_t ecs_query_filter(uint64_t with_mask, uint64_t without_mask) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world, "Attempted query with NULL active world (with_mask: %" PRIu64 ", without_mask: %" PRIu64 ")", with_mask, without_mask);
    if   (!world || !with_mask) return (ecs_view_t){ .rows = NULL, .cnt = 0UL };

    /* extract comp ids from bitmask */
    uint8_t  comp_ids[4] = {0};
    uint32_t comp_cnt    = 0UL;
    uint64_t tmp         = with_mask;

    while (tmp && comp_cnt < 4) {
        comp_ids[comp_cnt++] = (uint8_t)__builtin_ctzll(tmp);
        tmp                 &= tmp - 1ULL;
    }

    /* find smallest pool to minimize loop iters */
    uint32_t min_cnt  = UINT32_MAX;
    uint8_t  min_comp = comp_ids[0];

    for (uint32_t i = 0UL; i < comp_cnt; ++i) {
        uint8_t cid = comp_ids[i];
        if (cid >= COMP_CNT) {
            min_cnt = 0;
            break;
        }

        if (world->pools[cid].cnt < min_cnt) {
            min_cnt  = world->pools[cid].cnt;
            min_comp = cid;
        }
    }

    if (!min_cnt) return (ecs_view_t){ .rows = NULL, .cnt = 0UL };

    /* scratch-alloc row arr for worst-case match count */
    ecs_view_row_t *rows = (ecs_view_row_t *)arena_alloc(world->scratch, sizeof(ecs_view_row_t) * min_cnt);
    if (!rows) return (ecs_view_t){ .rows = NULL, .cnt = 0 };

    ecs_pool_t *sp      = &world->pools[min_comp];
    uint32_t    matched = 0UL;

    for (uint32_t i = 0UL; i < sp->cnt; ++i) {
        uint16_t e_idx = sp->entities[i];
        if (world->mgr.flags[e_idx] & ENTITY_FLAG_DEAD) continue;

        uint64_t mask = world->mgr.comp_mask[e_idx];
        if ((mask & with_mask) != with_mask || (mask & without_mask) != 0ULL) continue;

        ecs_view_row_t *row = &rows[matched++];
        row->ent            = entity_handle(world->mgr.gens[e_idx], world->mgr.flags[e_idx], e_idx);
        for (uint32_t c = 0UL; c < comp_cnt; ++c) row->dense_idxs[c] = world->pools[comp_ids[c]].sparse[e_idx];
    }

    return (ecs_view_t){ .rows = rows, .cnt = matched };
}

/* ============================================================================================== */
/* >COMP API                                                                                      */
/* ============================================================================================== */

void *ecs_get_raw(entity_handle_t e, comp_id_t id) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world, "Attempted to get component with NULL active world (e: %" PRIu64 ", id: %d)", e, id);

    if (!ecs_has_raw(e, id)) return NULL;

    ecs_pool_t *p     = &world->pools[id];
    uint16_t    d_idx = p->sparse[entity_idx(e)];

    return (uint8_t *)p->dense + ((size_t)d_idx * p->elem_sz);
}

bool ecs_has_raw(entity_handle_t e, comp_id_t id) {
    if (!ecs_is_alive(e) || id >= COMP_CNT) return false;

    ecs_world_t *world = ecs_get_world();
    ASSERT(world, "Attempted to check has component with NULL active world (e: %" PRIu64 ", id: %d)", e, id);

    uint16_t idx = entity_idx(e);
    return (world->mgr.comp_mask[idx] & BIT(id)) != 0ULL;
}

void *ecs_add_raw(entity_handle_t e, comp_id_t id, const void *val) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world,           "Attempted to add component with NULL active world (e: %" PRIu64 ", id: %d, val: %p)", e, id, val);
    ASSERT(ecs_is_alive(e), "Attempted to add component on dead entity (e: %" PRIu64 ", id: %d, val: %p)",         e, id, val);
    ASSERT(id < COMP_CNT,   "Attempted to add component with invalid id (e: %" PRIu64 ", id: %d, val: %p)",        e, id, val);
    if   (!world || !ecs_is_alive(e) || id >= COMP_CNT) return NULL;

    /* if already attached, update value and return existing ptr */
    if (ecs_has_raw(e, id)) {
        void *existing = ecs_get_raw(e, id);
        if (val) memcpy(existing, val, world->pools[id].elem_sz);
        return existing;
    }

    ecs_pool_t *p = &world->pools[id];
    ASSERT(p->cnt <= ECS_MAX_ENTITIES, "Pool capacity exceeded when adding component (e: %" PRIu64 ", id: %d, val: %p)", e, id, val);
    if    (p->cnt  > ECS_MAX_ENTITIES) return NULL;

    uint16_t e_idx               = entity_idx(e);
    uint16_t d_idx               = (uint16_t)p->cnt++;
    p->sparse[e_idx]             = d_idx;
    p->entities[d_idx]           = e_idx;
    p->dirty                     = true;
    world->mgr.comp_mask[e_idx] |= BIT(id);

    void *dst = (uint8_t *)p->dense + ((size_t)d_idx * p->elem_sz);
    if (val) memcpy(dst, val, p->elem_sz);
    else {
        const comp_meta_t *meta = reflect_comp_by_id(id);
        if (meta && meta->def_val) memcpy(dst, meta->def_val, p->elem_sz);
        else                       memset(dst, 0,             p->elem_sz);
    }

    return dst;
}

void ecs_del_raw(entity_handle_t e, comp_id_t id) {
    ASSERT(ecs_is_alive(e), "Attempted to delete component on dead entity (e: %" PRIu64 ", id: %d)",  e, id);
    ASSERT(id < COMP_CNT,   "Attempted to delete component with invalid id (e: %" PRIu64 ", id: %d)", e, id);
    if (!ecs_has_raw(e, id)) return;

    _ecs_cmd_push((ecs_cmd_t){ .type = ECS_CMD_DELETE_COMP, .ent = e, .comp = id });
}
