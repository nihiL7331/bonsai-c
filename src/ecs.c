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
    world->cmd_cnt = 0UL;
    world->cmd_cap = 0UL;
    world->cmd_buf = NULL;

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
    p->cnt--;
}

/* Unlinks a child node `h` from its parent's sibling list */
static inline void _ecs_hierarchy_unlink_parent(ecs_world_t *world, hierarchy *h) {
    if (!h || h->parent == ENTITY_NULL) return;

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

    if (ecs_has(e, hierarchy))
        _ecs_hierarchy_unlink_parent(world, ecs_get_fast(world, e, hierarchy));

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

        if      (cmd.type == ECS_CMD_DELETE_COMP) _ecs_pool_remove(world, e_idx, cmd.comp);
        else if (cmd.type == ECS_CMD_DESTROY_ENT) {
            /* remove all comps owned by this entity */
            uint64_t mask = mgr->comp_mask[e_idx];
            while (mask) {
                _ecs_pool_remove(world, e_idx, (comp_id_t)__builtin_ctzll(mask));
                mask &= mask - 1ULL;
            }

            mgr->flags[e_idx]     = ENTITY_FLAG_DEAD;
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

    if (!ecs_has(child, hierarchy)  && !ecs_add(child, hierarchy, NULL))  return EOOM;
    if (!ecs_has(parent, hierarchy) && !ecs_add(parent, hierarchy, NULL)) return EOOM;

    hierarchy *hc = ecs_get_fast(world, child,  hierarchy);
    hierarchy *hp = ecs_get_fast(world, parent, hierarchy);

    if (hc->parent == parent) return OK;

    if (hc->parent != ENTITY_NULL)
        _ecs_hierarchy_unlink_parent(world, hc);

    entity_handle_t old_first = hp->first_child;
    if (old_first != ENTITY_NULL && ecs_has(old_first, hierarchy))
        ecs_get_fast(world, old_first, hierarchy)->prev_sibling = child;

    hc->parent       = parent;
    hc->prev_sibling = ENTITY_NULL;
    hc->next_sibling = old_first;
    hp->first_child  = child;
    hp->child_cnt++;

    return OK;
}

res_t ecs_remove_parent(entity_handle_t child) {
    ecs_world_t *world = ecs_get_world();
    ASSERT(world,               "Attempted remove_parent with NULL active world");
    ASSERT(ecs_is_alive(child), "Attempted remove_parent on dead child (%" PRIu64 ")", child);
    if   (!world || !ecs_is_alive(child)) return EINVAL;

    if (ecs_has(child, hierarchy))
        _ecs_hierarchy_unlink_parent(world, ecs_get_fast(world, child, hierarchy));

    return OK;
}

static inline trans2d _ecs_combine_trans2d(trans2d p, trans2d l) {
    vec2f ls = vec_scale(vec2f(l.x, l.y), p.scale);

    if (p.rot != 0.0f) {
        float c = cosf(p.rot), s = sinf(p.rot);
        ls      = vec2f(ls.x * c - ls.y * s, ls.x * s + ls.y * c);
    }

    return (trans2d){
        .x     = p.x     + ls.x,
        .y     = p.y     + ls.y,
        .rot   = p.rot   + l.rot,
        .scale = p.scale * l.scale,
    };
}

static inline quatf _ecs_trans3d_quat(trans3d t) {
    if (t.qx == 0.0f && t.qy == 0.0f && t.qz == 0.0f && t.qw == 0.0f)
        return quatf_id();
    return quatf(t.qx, t.qy, t.qz, t.qw);
}

static inline trans3d _ecs_combine_trans3d(trans3d p, trans3d l) {
    quatf pq     = _ecs_trans3d_quat(p);
    quatf lq     = _ecs_trans3d_quat(l);
    vec3f scaled = vec_mul(vec3f(l.x, l.y, l.z), vec3f(p.scale_x, p.scale_y, p.scale_z));
    vec3f rot_p  = quatf_mul_vec3(pq, scaled);
    quatf wq     = quatf_mul(pq, lq);

    return (trans3d){
        .x       = p.x       + rot_p.x,
        .y       = p.y       + rot_p.y,
        .z       = p.z       + rot_p.z,
        .qx      = wq.x,
        .qy      = wq.y,
        .qz      = wq.z,
        .qw      = wq.w,
        .scale_x = p.scale_x * l.scale_x,
        .scale_y = p.scale_y * l.scale_y,
        .scale_z = p.scale_z * l.scale_z,
    };
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
