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

typedef struct ecs_snapshot_hdr {
    entity_mgr_t mgr;
    uint32_t     pool_cnts[COMP_CNT];
    void        *pool_dense[COMP_CNT];
    uint16_t    *pool_ents[COMP_CNT];
    uint32_t     subtree2d_cnt[ECS_MAX_ENTITIES + 1];
    uint32_t     subtree3d_cnt[ECS_MAX_ENTITIES + 1];
} ecs_snapshot_hdr_t;

/* Seeds the entity free list and marks generation 1 for all usable indices */
static inline void _ecs_world_entity_mgr_init(entity_mgr_t *mgr) {
    mgr->gens[0] = 0UL;
    mgr->free_hd = ECS_MAX_ENTITIES;
    for (size_t i = 0; i < ECS_MAX_ENTITIES; ++i) {
        mgr->gens[i + 1]  = 1UL;                              /* ENTITY_NULL,     1,      1,     1,      ... */
        mgr->flags[i + 1] = ENTITY_FLAG_DEAD;
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
    res_t schema_res = reflect_validate_schema();
    ASSERT(schema_res == OK, "Invalid component dependency/exclusion schema");
    if    (schema_res != OK) return NULL;

    ecs_world_t *world = (ecs_world_t *)arena_alloc(perm, sizeof(ecs_world_t));
    ASSERT(world, "Failed to allocate world to perm arena (%p)", (void *)perm);
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

res_t ecs_world_snapshot(void) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world && world->backup, "Attempted snapshot with NULL active world or backup arena");
    if   (!world || !world->backup) return EINVAL;

    arena_reset(world->backup);

    ecs_snapshot_hdr_t *hdr = (ecs_snapshot_hdr_t *)arena_alloc(world->backup, sizeof(ecs_snapshot_hdr_t));
    if (!hdr) return EOOM;

    hdr->mgr = world->mgr;
    memcpy(hdr->subtree2d_cnt, world->subtree2d_cnt, sizeof(hdr->subtree2d_cnt));
    memcpy(hdr->subtree3d_cnt, world->subtree3d_cnt, sizeof(hdr->subtree3d_cnt));

    for (size_t i = 0; i < COMP_CNT; ++i) {
        const ecs_pool_t *p = &world->pools[i];
        hdr->pool_cnts[i]   = p->cnt;

        if (!p->cnt) {
            hdr->pool_dense[i] = NULL;
            hdr->pool_ents[i]  = NULL;
            continue;
        }

        size_t dense_sz = (size_t)p->cnt * p->elem_sz;
        size_t ents_sz  = (size_t)p->cnt * sizeof(uint16_t);

        hdr->pool_dense[i] = arena_alloc(world->backup, dense_sz);
        hdr->pool_ents[i]  = (uint16_t *)arena_alloc(world->backup, ents_sz);
        if (!hdr->pool_dense[i] || !hdr->pool_ents[i]) { arena_reset(world->backup); return EOOM; }

        memcpy(hdr->pool_dense[i], p->dense,    dense_sz);
        memcpy(hdr->pool_ents[i],  p->entities, ents_sz);
    }

    return OK;
}

res_t ecs_world_rollback(void) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world && world->backup, "Attempted rollback with NULL active world or backup arena");
    if   (!world || !world->backup)                        return EINVAL;
    if   (world->backup->off < sizeof(ecs_snapshot_hdr_t)) return ENOTFND;

    const ecs_snapshot_hdr_t *hdr = (const ecs_snapshot_hdr_t *)world->backup->buf;

    world->mgr     = hdr->mgr;
    memcpy(world->subtree2d_cnt, hdr->subtree2d_cnt, sizeof(world->subtree2d_cnt));
    memcpy(world->subtree3d_cnt, hdr->subtree3d_cnt, sizeof(world->subtree3d_cnt));
    world->cmd_cnt = 0UL;
    world->cmd_cap = 0UL;
    world->cmd_buf = NULL;

    memset(world->pending_removals, 0, sizeof(world->pending_removals));

    for (size_t i = 0; i < COMP_CNT; ++i) {
        ecs_pool_t *p = &world->pools[i];

        for (uint32_t d = 0UL; d < p->cnt; ++d) p->sparse[p->entities[d]] = ECS_NULL_IDX;

        p->cnt = hdr->pool_cnts[i];
        if (!p->cnt) continue;

        memcpy(p->dense,    hdr->pool_dense[i], (size_t)p->cnt * p->elem_sz);
        memcpy(p->entities, hdr->pool_ents[i],  (size_t)p->cnt * sizeof(uint16_t));

        for (uint32_t d = 0UL; d < p->cnt; ++d) p->sparse[p->entities[d]] = (uint16_t)d;
    }

    return OK;
}

/* ============================================================================================== */
/* >ECS LIFECYCLE API                                                                             */
/* ============================================================================================== */

/* grows the cmd buffer before semantic mutation */
static bool _ecs_cmd_reserve(ecs_world_t *world, uint32_t additional) {
    ASSERT(world && world->scratch, "Command reservation requires a world and scratch");
    if   (!world || !world->scratch || additional > UINT32_MAX - world->cmd_cnt) return false;

    uint32_t needed = world->cmd_cnt + additional;
    if (needed <= world->cmd_cap) return true;

    size_t cap = world->cmd_cap ? world->cmd_cap : ECS_CMD_INIT_CAP;
    while (cap < needed) {
        if (cap > UINT32_MAX / 2U) { cap = needed; break; }
        cap *= 2U;
    }

    if (cap > SIZE_MAX / sizeof(ecs_cmd_t)) return false;
    size_t bytes = cap * sizeof(ecs_cmd_t);
    if (bytes > SIZE_MAX - (CACHE_LINE_SZ - 1U)) return false;

    ecs_cmd_t *buf = arena_alloc(world->scratch, bytes);
    if (!buf) return false;

    if (world->cmd_cnt) memcpy(buf, world->cmd_buf, (size_t)world->cmd_cnt * sizeof(*buf));
    world->cmd_buf = buf;
    world->cmd_cap = (uint32_t)cap;

    return true;
}

/* Pushes deferred mutation commands into scratch arena, doubling capacity on full */
static inline void _ecs_cmd_push(ecs_cmd_t cmd) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world && world->cmd_cnt  < world->cmd_cap, "Command capacity was not reserved");
    if   (!world || world->cmd_cnt >= world->cmd_cap) return;

    world->cmd_buf[world->cmd_cnt++] = cmd;
}

/*
==================
_ecs_tree_root

Returns the root of a valid live hierarchy. Fail-fast; links must form an acyclic forest.
==================
*/
static entity_handle_t _ecs_tree_root(ecs_world_t *world, entity_handle_t e) {
    while (ecs_has(e, hierarchy)) {
        const hierarchy *h = ecs_get_fast(world, e, hierarchy);
        if (h->parent == ENTITY_NULL) break;
        e = h->parent;
    }
    return e;
}

/*
==================
_ecs_subtree_adjust

Adjusts physical transform counts for a node and its ancestors. Counts include the node.
Fail-fast; links and cached counts must be valid.
==================
*/
static void _ecs_subtree_adjust(ecs_world_t *world, entity_handle_t e,
                                int32_t delta2d, int32_t delta3d) {
    while (e != ENTITY_NULL) {
        uint16_t index = entity_idx(e);
        int32_t next2d = (int32_t)world->subtree2d_cnt[index] + delta2d;
        int32_t next3d = (int32_t)world->subtree3d_cnt[index] + delta3d;
        ASSERT(next2d >= 0 && next2d <= (int32_t)ECS_MAX_ENTITIES, "Invalid 2D subtree count");
        ASSERT(next3d >= 0 && next3d <= (int32_t)ECS_MAX_ENTITIES, "Invalid 3D subtree count");
        world->subtree2d_cnt[index] = (uint32_t)next2d;
        world->subtree3d_cnt[index] = (uint32_t)next3d;

        if (!ecs_has(e, hierarchy)) break;
        e = ecs_get_fast(world, e, hierarchy)->parent;
    }
}

/*
==================
_ecs_pool_remove

Removes a physical pool entry with swap-and-pop. Transform removal updates ancestor counts.
Dead subtrees have already released their counts. Fail-fast for valid world and component.
==================
*/
static inline void _ecs_pool_remove(ecs_world_t *world, uint16_t e_idx, comp_id_t comp) {
    ecs_pool_t *p     = &world->pools[comp];
    uint16_t    d_idx = p->sparse[e_idx];
    if (d_idx == ECS_NULL_IDX || d_idx >= p->cnt) return;

    if (!(world->mgr.flags[e_idx] & ENTITY_FLAG_DEAD)) {
        entity_handle_t e = entity_handle(world->mgr.gens[e_idx], world->mgr.flags[e_idx], e_idx);
        if (comp == COMP_trans2d) _ecs_subtree_adjust(world, e, -1, 0);
        if (comp == COMP_trans3d) _ecs_subtree_adjust(world, e, 0, -1);
    }

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
    p->cnt--;
}

/*
==================
_ecs_hierarchy_unlink_parent

Detaches a child and subtracts its subtree counts from the old ancestors.
Fail-fast for a valid live forest; absent hierarchy or parent is a no-op.
==================
*/
static inline void _ecs_hierarchy_unlink_parent(ecs_world_t *world, entity_handle_t child, hierarchy *h) {
    if (!h || h->parent == ENTITY_NULL) return;

    uint16_t index = entity_idx(child);
    _ecs_subtree_adjust(world, h->parent,
                        -(int32_t)world->subtree2d_cnt[index],
                        -(int32_t)world->subtree3d_cnt[index]);



    if (h->prev_sibling != ENTITY_NULL && ecs_has(h->prev_sibling, hierarchy))
        ecs_get_fast(world, h->prev_sibling, hierarchy)->next_sibling = h->next_sibling;
    else if (ecs_has(h->parent, hierarchy))
        ecs_get_fast(world, h->parent, hierarchy)->first_child = h->next_sibling;

    if (h->next_sibling != ENTITY_NULL && ecs_has(h->next_sibling, hierarchy))
        ecs_get_fast(world, h->next_sibling, hierarchy)->prev_sibling = h->prev_sibling;

    if (ecs_has(h->parent, hierarchy)) {
        hierarchy *hp = ecs_get_fast(world, h->parent, hierarchy);
        if (hp->child_cnt > 0U) hp->child_cnt--;
    }

    h->parent       = ENTITY_NULL;
    h->prev_sibling = ENTITY_NULL;
    h->next_sibling = ENTITY_NULL;
}

/* counts live subtree destruction commands */
static uint32_t _ecs_destroy_cmd_cnt(ecs_world_t *world, entity_handle_t e) {
    if (!ecs_is_alive(e)) return 0;

    uint32_t cnt = 1;
    if (!ecs_has(e, hierarchy)) return cnt;

    const hierarchy *h     = ecs_get_fast(world, e, hierarchy);
    entity_handle_t  child = h->first_child;

    while (child != ENTITY_NULL) {
        entity_handle_t next = ENTITY_NULL;
        if (ecs_has(child, hierarchy))
            next = ecs_get_fast(world, child, hierarchy)->next_sibling;

        cnt  += _ecs_destroy_cmd_cnt(world, child);
        child = next;
    }

    return cnt;
}

static void _ecs_destroy_recursive(ecs_world_t *world, entity_handle_t e) {
    if (!ecs_is_alive(e)) return;

    if (ecs_has(e, hierarchy)) {
        hierarchy      *h     = ecs_get_fast(world, e, hierarchy);
        entity_handle_t child = h->first_child;
        h->first_child        = ENTITY_NULL;
        h->child_cnt          = 0U;

        while (child != ENTITY_NULL) {
            entity_handle_t next = ENTITY_NULL;

            if (ecs_has(child, hierarchy)) {
                hierarchy *hc    = ecs_get_fast(world, child, hierarchy);
                next             = hc->next_sibling;
                hc->parent       = ENTITY_NULL;
                hc->prev_sibling = ENTITY_NULL;
                hc->next_sibling = ENTITY_NULL;
            }

            _ecs_destroy_recursive(world, child);
            child = next;
        }
    }

    uint16_t idx           = entity_idx(e);
    world->subtree2d_cnt[idx] = 0;
    world->subtree3d_cnt[idx] = 0;
    world->mgr.flags[idx] |= ENTITY_FLAG_DEAD;
    _ecs_cmd_push((ecs_cmd_t){ .type = ECS_CMD_DESTROY_ENT, .ent = e, .comp = 0 });
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
    world->pending_removals[idx] = 0;
    world->subtree2d_cnt[idx] = 0;
    world->subtree3d_cnt[idx] = 0;
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

    uint32_t cmds = _ecs_destroy_cmd_cnt(world, e);
    if (!_ecs_cmd_reserve(world, cmds)) return;

    if (ecs_has(e, hierarchy))
        _ecs_hierarchy_unlink_parent(world, e, ecs_get_fast(world, e, hierarchy));

    _ecs_destroy_recursive(world, e);
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

        if (cmd.type == ECS_CMD_DELETE_COMP) {
            _ecs_pool_remove(world, e_idx, cmd.comp);
            world->pending_removals[e_idx] &= ~BIT(cmd.comp);
        } else if (cmd.type == ECS_CMD_DESTROY_ENT) {
            /* remove all comps owned by this entity */
            uint64_t mask = mgr->comp_mask[e_idx];
            while (mask) {
                _ecs_pool_remove(world, e_idx, (comp_id_t)__builtin_ctzll(mask));
                mask &= mask - 1ULL;
            }

            mgr->flags[e_idx]              = ENTITY_FLAG_DEAD;
            mgr->comp_mask[e_idx]          = 0ULL;
            world->pending_removals[e_idx] = 0;

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
    if (!world || !with_mask || (with_mask & without_mask) != 0ULL) return (ecs_view_t){ .ents = NULL, .cnt = 0UL };

    /* find smallest pool in with_mask to minimize loop iters */
    uint32_t min_cnt  = UINT32_MAX;
    uint8_t  min_comp = 0U;
    uint64_t tmp      = with_mask;

    while (tmp) {
        uint8_t cid = (uint8_t)__builtin_ctzll(tmp);
        tmp        &= tmp - 1ULL;

        if (cid >= COMP_CNT) return (ecs_view_t){ .ents = NULL, .cnt = 0UL };

        if (world->pools[cid].cnt < min_cnt) {
            min_cnt  = world->pools[cid].cnt;
            min_comp = cid;
            if (!min_cnt) return (ecs_view_t){ .ents = NULL, .cnt = 0UL };
        }
    }

    /* scratch-alloc handle arr for worst-case match count */
    entity_handle_t *ents = (entity_handle_t *)arena_alloc(world->scratch, sizeof(entity_handle_t) * min_cnt);
    if (!ents) return (ecs_view_t){ .ents = NULL, .cnt = 0UL };

    ecs_pool_t *sp      = &world->pools[min_comp];
    uint32_t    matched = 0UL;

    for (uint32_t i = 0UL; i < sp->cnt; ++i) {
        uint16_t e_idx = sp->entities[i];
        if (world->mgr.flags[e_idx] & ENTITY_FLAG_DEAD) continue;

        uint64_t mask = world->mgr.comp_mask[e_idx];
        if ((mask & with_mask) != with_mask || (mask & without_mask) != 0ULL) continue;

        ents[matched++] = entity_handle(world->mgr.gens[e_idx], world->mgr.flags[e_idx], e_idx);
    }

    return (ecs_view_t){ .ents = ents, .cnt = matched };
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
    ASSERT(world,                             "Attempted to add component with NULL active world (e: %" PRIu64 ", id: %d, val: %p)", e, id, val);
    ASSERT(ecs_is_alive(e),                   "Attempted to add component on dead entity (e: %" PRIu64 ", id: %d, val: %p)",         e, id, val);
    ASSERT((unsigned)id < (unsigned)COMP_CNT, "Attempted to add component with invalid id (e: %" PRIu64 ", id: %d, val: %p)",        e, id, val);
    if   (!world || !ecs_is_alive(e) || (unsigned)id >= (unsigned)COMP_CNT) return NULL;

    uint16_t e_idx     = entity_idx(e);
    uint64_t bit       = BIT(id);
    uint64_t current   = world->mgr.comp_mask[e_idx];
    uint64_t pending   = world->pending_removals[e_idx];
    uint64_t effective = current & ~pending;

    const comp_meta_t *meta = reflect_comp_by_id(id);
    ASSERT(meta, "Missing component metadata for id: %d", id);
    if   (!meta) return NULL;

    uint64_t missing   = meta->requires_mask & ~effective;
    uint64_t conflicts = meta->excludes_mask &  current;
    bool removing = (pending & bit) != 0;

    ASSERT(!removing,  "Entity %" PRIu64 ": %s is pending removal",             e, meta->name);
    ASSERT(!missing,   "Entity %" PRIu64 ": %s lacks prerequisites 0x%" PRIx64, e, meta->name, missing);
    ASSERT(!conflicts, "Entity %" PRIu64 ": %s conflicts with mask 0x%" PRIx64, e, meta->name, conflicts);
    if     (removing || missing || conflicts) return NULL;

    if (id == COMP_hierarchy && val) {
        const hierarchy *incoming = val;
        const hierarchy *existing = ecs_get_raw(e, id);
        const hierarchy *reference = existing ? existing : &hierarchy_def;
        bool same_links =
            incoming->parent == reference->parent &&
            incoming->first_child == reference->first_child &&
            incoming->prev_sibling == reference->prev_sibling &&
            incoming->next_sibling == reference->next_sibling &&
            incoming->child_cnt == reference->child_cnt;
        ASSERT(same_links, "Use hierarchy APIs to change links");
        if (!same_links) return NULL;
    }

    if ((id == COMP_trans2d || id == COMP_trans3d) && !(current & bit)) {
        uint16_t root = entity_idx(_ecs_tree_root(world, e));
        uint32_t opposite = id == COMP_trans2d
                            ? world->subtree3d_cnt[root]
                            : world->subtree2d_cnt[root];
        ASSERT(opposite == 0, "Entity %" PRIu64 ": transform conflicts with tree dimension", e);
        if (opposite) return NULL;
    }

    /* if already attached, update value and return existing ptr */
    if (ecs_has_raw(e, id)) {
        void *existing = ecs_get_raw(e, id);
        if (val) memcpy(existing, val, world->pools[id].elem_sz);
        return existing;
    }

    ecs_pool_t *p = &world->pools[id];
    ASSERT(p->cnt <= ECS_MAX_ENTITIES, "Pool capacity exceeded when adding component (e: %" PRIu64 ", id: %d, val: %p)", e, id, val);
    if    (p->cnt  > ECS_MAX_ENTITIES) return NULL;

    uint16_t d_idx               = (uint16_t)p->cnt++;
    p->sparse[e_idx]             = d_idx;
    p->entities[d_idx]           = e_idx;
    world->mgr.comp_mask[e_idx] |= BIT(id);

    void *dst = (uint8_t *)p->dense + ((size_t)d_idx * p->elem_sz);
    if (val) memcpy(dst, val, p->elem_sz);
    else {
        if (meta && meta->def_val) memcpy(dst, meta->def_val, p->elem_sz);
        else                       memset(dst, 0,             p->elem_sz);
    }

    if (id == COMP_trans2d) _ecs_subtree_adjust(world, e, 1, 0);
    if (id == COMP_trans3d) _ecs_subtree_adjust(world, e, 0, 1);

    return dst;
}

void ecs_del_raw(entity_handle_t e, comp_id_t id) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world,                               "Component removal requires an active world (e: %" PRIu64 ", id: %d)", e, id);
    ASSERT(ecs_is_alive(e),                     "Attempted to delete component on dead entity (e: %" PRIu64 ", id: %d)",  e, id);
    ASSERT((unsigned)id < (unsigned)COMP_CNT,   "Attempted to delete component with invalid id (e: %" PRIu64 ", id: %d)", e, id);
    if (!world || !ecs_is_alive(e) || (unsigned)id >= (unsigned)COMP_CNT || !ecs_has_raw(e, id)) return;

    uint16_t idx     = entity_idx(e);
    uint64_t bit     = BIT(id);
    uint64_t current = world->mgr.comp_mask[idx];
    uint64_t pending = world->pending_removals[idx];
    if (!(current & bit) || (pending & bit)) return;

    if (id == COMP_hierarchy) {
        const hierarchy *h = ecs_get_fast(world, e, hierarchy);
        bool linked = h->parent != ENTITY_NULL || h->first_child != ENTITY_NULL ||
                      h->prev_sibling != ENTITY_NULL || h->next_sibling != ENTITY_NULL ||
                      h->child_cnt != 0;
        ASSERT(!linked, "Detach hierarchy links before removing the component");
        if (linked) return;
    }

    const comp_meta_t *meta = reflect_comp_by_id(id);
    ASSERT(meta, "Missing component metadata for id: %d", id);
    if   (!meta) return;

    uint64_t effective  = current   & ~pending;
    uint64_t dependents = effective &  meta->required_by_mask;
    ASSERT (!dependents, "Entity %" PRIu64 ": %s required by mask 0x%" PRIx64, e, meta->name, dependents);
    if      (dependents)                  return;
    if      (!_ecs_cmd_reserve(world, 1)) return;

    _ecs_cmd_push((ecs_cmd_t){ .type = ECS_CMD_DELETE_COMP, .ent = e, .comp = id });
    world->pending_removals[idx] |= bit;
}

/* ============================================================================================== */
/* >HIERARCHY API                                                                                 */
/* ============================================================================================== */

res_t ecs_set_parent(entity_handle_t child, entity_handle_t parent) {
    if (parent == ENTITY_NULL) return ecs_remove_parent(child);

    ecs_world_t *world = ecs_get_world();
    ASSERT(world,                "Attempted set_parent with NULL active world");
    ASSERT(ecs_is_alive(child),  "Attempted set_parent on dead child (%" PRIu64 ")",  child);
    ASSERT(ecs_is_alive(parent), "Attempted set_parent on dead parent (%" PRIu64 ")", parent);
    if   (!world || !ecs_is_alive(child) || !ecs_is_alive(parent) || child == parent) return EINVAL;

    /* verify `child` is not ancestor of `parent` */
    entity_handle_t anc = parent;
    while (anc != ENTITY_NULL && ecs_has(anc, hierarchy)) {
        anc = ecs_get_fast(world, anc, hierarchy)->parent;
        if (anc == child) return EINVAL;
    }

    bool pending_links =
        (world->pending_removals[entity_idx(child)] & BIT(COMP_hierarchy)) != 0 ||
        (world->pending_removals[entity_idx(parent)] & BIT(COMP_hierarchy)) != 0;
    ASSERT(!pending_links, "Cannot parent through a hierarchy pending removal");
    if (pending_links) return EINVAL;

    uint16_t child_index = entity_idx(child);
    uint16_t root_index = entity_idx(_ecs_tree_root(world, parent));
    bool incompatible =
        (world->subtree2d_cnt[child_index] && world->subtree3d_cnt[root_index]) ||
        (world->subtree3d_cnt[child_index] && world->subtree2d_cnt[root_index]);
    ASSERT(!incompatible, "Cannot combine 2D and 3D hierarchy trees");
    if (incompatible) return EINVAL;

    if (!ecs_has(child, hierarchy)  && !ecs_add(child, hierarchy, NULL))  return EOOM;
    if (!ecs_has(parent, hierarchy) && !ecs_add(parent, hierarchy, NULL)) return EOOM;

    hierarchy *hc = ecs_get_fast(world, child,  hierarchy);
    hierarchy *hp = ecs_get_fast(world, parent, hierarchy);

    if (hc->parent == parent) return OK;

    if (hc->parent != ENTITY_NULL)
        _ecs_hierarchy_unlink_parent(world, child, hc);

    entity_handle_t old_first = hp->first_child;
    if (old_first != ENTITY_NULL && ecs_has(old_first, hierarchy))
        ecs_get_fast(world, old_first, hierarchy)->prev_sibling = child;

    hc->parent       = parent;
    hc->prev_sibling = ENTITY_NULL;
    hc->next_sibling = old_first;
    hp->first_child  = child;
    hp->child_cnt++;

    _ecs_subtree_adjust(world, parent,
                        (int32_t)world->subtree2d_cnt[entity_idx(child)],
                        (int32_t)world->subtree3d_cnt[entity_idx(child)]);

    return OK;
}

res_t ecs_remove_parent(entity_handle_t child) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world,               "Attempted remove_parent with NULL active world");
    ASSERT(ecs_is_alive(child), "Attempted remove_parent on dead child (%" PRIu64 ")", child);
    if   (!world || !ecs_is_alive(child)) return EINVAL;

    if (ecs_has(child, hierarchy))
        _ecs_hierarchy_unlink_parent(world, child, ecs_get_fast(world, child, hierarchy));

    return OK;
}

static inline trans2d _ecs_combine_trans2d(trans2d p, trans2d l) {
    vec2f ls = vec_scale(l.position, p.scale);

    if (p.rotation != 0.0f) {
        float c = cosf(p.rotation), s = sinf(p.rotation);
        ls      = vec2f(ls.x * c - ls.y * s, ls.x * s + ls.y * c);
    }

    return trans2d(vec_add(p.position, ls), p.rotation + l.rotation, p.scale * l.scale);
}

static inline quatf _ecs_trans3d_quat(trans3d t) {
    if (t.rotation.x == 0.0f && t.rotation.y == 0.0f && t.rotation.z == 0.0f && t.rotation.w == 0.0f)
        return quatf_id();
    return t.rotation;
}

static inline trans3d _ecs_combine_trans3d(trans3d p, trans3d l) {
    quatf pq     = _ecs_trans3d_quat(p);
    quatf lq     = _ecs_trans3d_quat(l);
    vec3f scaled = vec_mul(l.position, p.scale);
    vec3f rot_p  = quatf_mul_vec3(pq, scaled);
    quatf wq     = quatf_mul(pq, lq);

    return trans3d(vec_add(p.position, rot_p), wq, vec_mul(p.scale, l.scale));
}

bool ecs_get_world_trans2d(entity_handle_t e, trans2d *out) {
    ecs_world_t *world = ecs_get_world();
    if (!world || !out || !ecs_has(e, trans2d)) return false;

    const trans2d *chain[64];
    uint32_t       depth = 0U;

    entity_handle_t cur = e;
    while (cur != ENTITY_NULL && depth < ARR_CNT(chain)) {
        if (ecs_has(cur, trans2d)) chain[depth++] = ecs_get_fast(world, cur, trans2d);

        if (!ecs_has(cur, hierarchy)) break;
        cur = ecs_get_fast(world, cur, hierarchy)->parent;
    }

    trans2d acc = *chain[depth - 1U];
    for (int32_t i = (int32_t)depth - 2; i >= 0; --i) acc = _ecs_combine_trans2d(acc, *chain[i]);

    *out = acc;

    return true;
}

bool ecs_get_world_trans3d(entity_handle_t e, trans3d *out) {
    ecs_world_t *world = ecs_get_world();
    if (!world || !out || !ecs_has(e, trans3d)) return false;

    const trans3d *chain[64];
    uint32_t       depth = 0U;

    entity_handle_t cur = e;
    while (cur != ENTITY_NULL && depth < ARR_CNT(chain)) {
        if (ecs_has(cur, trans3d)) chain[depth++] = ecs_get_fast(world, cur, trans3d);

        if (!ecs_has(cur, hierarchy)) break;
        cur = ecs_get_fast(world, cur, hierarchy)->parent;
    }

    trans3d acc = *chain[depth - 1U];
    for (int32_t i = (int32_t)depth - 2; i >= 0; --i) acc = _ecs_combine_trans3d(acc, *chain[i]);

    *out = acc;

    return true;
}
