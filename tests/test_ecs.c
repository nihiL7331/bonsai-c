#include "core.h"
#include "arena.h"
#include "reflect.h"
#include "ecs.h"
#include "test.h"

#include <stdlib.h>
#include <string.h>

#define TEST_PERM_SZ    MB(32)
#define TEST_SCRATCH_SZ MB(4)

static arena_t s_perm_arena;
static arena_t s_scratch_arena;

static ecs_world_t *reset_world(void) {
    arena_reset(&s_perm_arena);
    arena_reset(&s_scratch_arena);
    return ecs_world_create(&s_perm_arena, NULL, &s_scratch_arena);
}

int main(void) {
    void *perm_buf    = malloc(TEST_PERM_SZ);
    void *scratch_buf = malloc(TEST_SCRATCH_SZ);
    if (!perm_buf || !scratch_buf) return 1;

    arena_init(&s_perm_arena,    perm_buf,    TEST_PERM_SZ);
    arena_init(&s_scratch_arena, scratch_buf, TEST_SCRATCH_SZ);

    TEST_CASE("ecs_spawn_index", "assigns sequential non-zero indices") {
        reset_world();
        entity_handle_t e1 = ecs_spawn();
        entity_handle_t e2 = ecs_spawn();
        entity_handle_t e3 = ecs_spawn();

        TEST_ASSERT(entity_idx(e1) == 1U, "e1 index exp 1, got %u", entity_idx(e1));
        TEST_ASSERT(entity_idx(e2) == 2U, "e2 index exp 2, got %u", entity_idx(e2));
        TEST_ASSERT(entity_idx(e3) == 3U, "e3 index exp 3, got %u", entity_idx(e3));
    } TEST_CASE_END();

    TEST_CASE("ecs_spawn_gen", "initializes living entity gen to 1") {
        reset_world();
        entity_handle_t e = ecs_spawn();
        TEST_ASSERT(entity_gen(e) == 1U, "initial generation exp 1, got %u", entity_gen(e));
        TEST_ASSERT(ecs_is_alive(e),     "newly spawned entity must be alive");
    } TEST_CASE_END();

    TEST_CASE("ecs_null_handle", "ENTITY_NULL evaluates to dead safely") {
        reset_world();
        TEST_ASSERT(!ecs_is_alive(ENTITY_NULL), "ENTITY_NULL must evaluate to dead");
        TEST_ASSERT(!entity_idx(ENTITY_NULL),   "null handle index must be 0");
    } TEST_CASE_END();

    TEST_CASE("ecs_destroy_dead", "marks handle dead immediately before sync") {
        reset_world();
        entity_handle_t e1 = ecs_spawn();
        entity_handle_t e2 = ecs_spawn();

        ecs_destroy(e1);
        TEST_ASSERT(!ecs_is_alive(e1), "e1 must report dead immediately after ecs_destroy");
        TEST_ASSERT(ecs_is_alive(e2),  "e2 must remain alive");
    } TEST_CASE_END();

    TEST_CASE("ecs_sync_reclaim", "decrements alive_cnt and resets cmd_buf") {
        ecs_world_t    *w = reset_world();
        entity_handle_t e = ecs_spawn();
        TEST_ASSERT(w->mgr.alive_cnt == 1U, "alive_cnt exp 1");

        ecs_destroy(e);
        TEST_ASSERT(w->mgr.alive_cnt == 1U, "alive_cnt must remain 1 before sync");
        TEST_ASSERT(w->cmd_cnt       == 1U, "cmd_cnt must be 1 before sync");

        ecs_sync();
        TEST_ASSERT(!w->mgr.alive_cnt, "alive_cnt must be 0 after sync");
        TEST_ASSERT(!w->cmd_cnt,       "cmd_cnt must reset to 0 after sync");
    } TEST_CASE_END();

    TEST_CASE("ecs_recycle_index", "reuses destroyed entity index on next spawn") {
        reset_world();
        entity_handle_t e1     = ecs_spawn();
        uint16_t        e1_idx = entity_idx(e1);

        ecs_destroy(e1);
        ecs_sync();

        entity_handle_t e2 = ecs_spawn();
        TEST_ASSERT(entity_idx(e2) == e1_idx, "expected reused idx %u, got %u", e1_idx, entity_idx(e2));
    } TEST_CASE_END();

    TEST_CASE("ecs_recycle_gen", "increments generation upon recycling") {
        reset_world();
        entity_handle_t e1 = ecs_spawn();
        ecs_destroy(e1);
        ecs_sync();

        entity_handle_t e2 = ecs_spawn();
        TEST_ASSERT(entity_gen(e2) == 2U, "generation must bump from 1 to 2, got %u", entity_gen(e2));
    } TEST_CASE_END();

    TEST_CASE("ecs_stale_handle", "stale generational handle remains dead") {
        reset_world();
        entity_handle_t e_old = ecs_spawn();
        ecs_destroy(e_old);
        ecs_sync();

        entity_handle_t e_new = ecs_spawn();
        TEST_ASSERT(ecs_is_alive(e_new),  "recycled entity must be alive");
        TEST_ASSERT(!ecs_is_alive(e_old), "stale entity handle must remain dead");
    } TEST_CASE_END();

    TEST_CASE("ecs_add_custom", "copies explicit values into dense pool") {
        reset_world();
        entity_handle_t e   = ecs_spawn();
        trans2d         val = { .x = 12.5f, .y = 34.75f, .rot = 1.0f, .scale = 2.0f };
        trans2d        *t   = ecs_add(e, trans2d, &val);

        TEST_ASSERT(t != NULL,                             "ecs_add returned NULL");
        TEST_ASSERT(t->x   == 12.5f && t->y     == 34.75f, "trans2d values not copied");
        TEST_ASSERT(t->rot == 1.0f  && t->scale == 2.0f,   "trans2d values not copied");
    } TEST_CASE_END();

    TEST_CASE("ecs_add_default", "initializes reflection defaults when val=NULL") {
        reset_world();
        entity_handle_t e = ecs_spawn();
        sprite2d       *s = ecs_add(e, sprite2d, NULL);

        TEST_ASSERT(s,                       "ecs_add returned NULL for sprite2d");
        TEST_ASSERT(s->id    == 0x0000U,     "default sprite id exp 0, got 0x%04X", s->id);
        TEST_ASSERT(s->color == 0xFFFFFFFFU, "default color exp 0xFFFFFFFF, got 0x%08X", s->color);
    } TEST_CASE_END();

    TEST_CASE("ecs_add_existing", "updates in-place when component already owned") {
        reset_world();
        entity_handle_t e    = ecs_spawn();
        trans2d         val1 = { .x = 10.0f, .y = 20.0f, .rot = 0.0f, .scale = 1.0f };
        trans2d         val2 = { .x = 99.0f, .y = 88.0f, .rot = 0.0f, .scale = 1.0f };
        trans2d        *t1   = ecs_add(e, trans2d, &val1);
        trans2d        *t2   = ecs_add(e, trans2d, &val2);

        TEST_ASSERT(t1 == t2,                         "adding existing component must return same pointer");
        TEST_ASSERT(t1->x == 99.0f && t1->y == 88.0f, "component values not updated in-place");
    } TEST_CASE_END();

    TEST_CASE("ecs_get_valid", "returns direct pointer to contiguous dense memory") {
        reset_world();
        entity_handle_t e     = ecs_spawn();
        trans2d        *added = ecs_add(e, trans2d, NULL);
        added->x = 55.0f;

        trans2d *got = ecs_get(e, trans2d);
        TEST_ASSERT(got    == added, "ecs_get pointer mismatch");
        TEST_ASSERT(got->x == 55.0f, "ecs_get value mismatch");
    } TEST_CASE_END();

    TEST_CASE("ecs_get_missing", "returns NULL on unowned component or dead handle") {
        reset_world();
        entity_handle_t e = ecs_spawn();
        TEST_ASSERT(!ecs_get(e, trans2d), "unowned component must return NULL");

        ecs_add(e, trans2d, NULL);
        ecs_destroy(e);
        TEST_ASSERT(!ecs_get(e, trans2d), "component on dead entity must return NULL");
    } TEST_CASE_END();

    TEST_CASE("ecs_del_deferred", "component remains accessible until ecs_sync") {
        ecs_world_t    *w = reset_world();
        entity_handle_t e = ecs_spawn();
        ecs_add(e, trans2d, NULL);

        ecs_del(e, trans2d);
        TEST_ASSERT(ecs_has(e, trans2d), "component must still exist before sync");
        TEST_ASSERT(w->cmd_cnt == 1U,    "del command not queued");

        ecs_sync();
        TEST_ASSERT(!ecs_has(e, trans2d), "component must be removed after sync");
        TEST_ASSERT(!ecs_get(e, trans2d), "ecs_get must return NULL after sync");
    } TEST_CASE_END();

    TEST_CASE("ecs_swap_pop", "maintains dense compactness and backpointers") {
        ecs_world_t    *w  = reset_world();
        entity_handle_t e1 = ecs_spawn();
        entity_handle_t e2 = ecs_spawn();
        entity_handle_t e3 = ecs_spawn();

        trans2d t1 = { .x = 1.0f, .y = 0.0f, .rot = 0.0f, .scale = 1.0f };
        trans2d t2 = { .x = 2.0f, .y = 0.0f, .rot = 0.0f, .scale = 1.0f };
        trans2d t3 = { .x = 3.0f, .y = 0.0f, .rot = 0.0f, .scale = 1.0f };

        ecs_add(e1, trans2d, &t1);
        ecs_add(e2, trans2d, &t2);
        ecs_add(e3, trans2d, &t3);

        ecs_pool_t *p = &w->pools[COMP_trans2d];
        TEST_ASSERT(p->cnt == 3U, "initial pool count exp 3");

        ecs_del(e2, trans2d);
        ecs_sync();

        uint16_t e3_idx = entity_idx(e3);
        uint16_t e3_dense = p->sparse[e3_idx];

        TEST_ASSERT(p->cnt == 2U,                                            "pool count exp 2 after del");
        TEST_ASSERT(!ecs_has(e2, trans2d),                                   "e2 must not have trans2d");
        TEST_ASSERT(ecs_has(e1, trans2d) && ecs_get(e1, trans2d)->x == 1.0f, "e1 corrupted");
        TEST_ASSERT(ecs_has(e3, trans2d) && ecs_get(e3, trans2d)->x == 3.0f, "e3 corrupted");
        TEST_ASSERT(e3_dense == 1U,                                          "e3 dense index exp 1, got %u", e3_dense);
        TEST_ASSERT(p->entities[e3_dense] == e3_idx,                         "backpointer mismatch");
    } TEST_CASE_END();

    TEST_CASE("ecs_query_single", "matches all entities owning target component") {
        reset_world();
        entity_handle_t e1 = ecs_spawn();
        entity_handle_t e2 = ecs_spawn();
        entity_handle_t e3 = ecs_spawn();

        ecs_add(e1, trans2d,  NULL);
        ecs_add(e2, trans2d,  NULL);
        ecs_add(e3, sprite2d, NULL);

        ecs_view_t v = ecs_query_comp(trans2d);
        TEST_ASSERT(v.cnt == 2U,                        "expected 2 matches, got %u", v.cnt);
        TEST_ASSERT(v.ents[0] == e1 && v.ents[1] == e2, "matched entity mismatch");
    } TEST_CASE_END();

    TEST_CASE("ecs_query_multi", "matches exact component intersection") {
        reset_world();
        entity_handle_t e1 = ecs_spawn();
        entity_handle_t e2 = ecs_spawn();
        entity_handle_t e3 = ecs_spawn();

        ecs_add(e1, trans2d,  NULL);
        ecs_add(e1, sprite2d, NULL);
        ecs_add(e2, trans2d,  NULL);
        ecs_add(e2, sprite2d, NULL);
        ecs_add(e2, rb2d,     NULL);
        ecs_add(e3, trans2d, NULL);

        ecs_view_t v2 = ecs_query_comp(trans2d, sprite2d);
        ecs_view_t v3 = ecs_query_comp(trans2d, sprite2d, rb2d);

        TEST_ASSERT(v2.cnt == 2U,     "expected 2 matches for (trans2d, sprite2d), got %u", v2.cnt);
        TEST_ASSERT(v3.cnt == 1U,     "expected 1 match for 3 comps, got %u",               v3.cnt);
        TEST_ASSERT(v3.ents[0] == e2, "matched wrong entity");
    } TEST_CASE_END();

    TEST_CASE("ecs_query_many", "supports >4 components and ecs_get_fast unpacking") {
        ecs_world_t    *w  = reset_world();
        entity_handle_t e1 = ecs_spawn();
        entity_handle_t e2 = ecs_spawn();

        /* e1 gets 5 components, e2 gets 6 components */
        ecs_add(e1, trans2d,    NULL);
        ecs_add(e1, sprite2d,   NULL);
        ecs_add(e1, collider2d, NULL);
        ecs_add(e1, rb2d,       NULL);
        ecs_add(e1, tag,        NULL);

        trans2d custom_t = { .x = 99.0f, .y = -42.0f, .rot = 0.5f, .scale = 2.0f };
        ecs_add(e2, trans2d,    &custom_t);
        ecs_add(e2, sprite2d,   NULL);
        ecs_add(e2, collider2d, NULL);
        ecs_add(e2, rb2d,       NULL);
        ecs_add(e2, tag,        NULL);
        ecs_add(e2, trans3d,    NULL);

        ecs_view_t v5 = ecs_query_comp(trans2d, sprite2d, collider2d, rb2d, tag);
        ecs_view_t v6 = ecs_query_comp(trans2d, sprite2d, collider2d, rb2d, tag, trans3d);

        TEST_ASSERT(v5.cnt == 2U,     "expected 2 matches for 5-comp query, got %u", v5.cnt);
        TEST_ASSERT(v6.cnt == 1U,     "expected 1 match for 6-comp query, got %u",   v6.cnt);
        TEST_ASSERT(v6.ents[0] == e2, "6-comp query matched wrong entity");

        const trans2d *fast_t = ecs_get_fast(w, v6.ents[0], trans2d);
        TEST_ASSERT(fast_t != NULL,        "ecs_get_fast returned NULL");
        TEST_ASSERT(fast_t->x == 99.0f,    "ecs_get_fast x mismatch");
        TEST_ASSERT(fast_t->y == -42.0f,   "ecs_get_fast y mismatch");
    } TEST_CASE_END();

    TEST_CASE("ecs_query_exclude", "filters out entities matching without_mask") {
        reset_world();
        entity_handle_t e1 = ecs_spawn();
        entity_handle_t e2 = ecs_spawn();

        ecs_add(e1, trans2d,  NULL);
        ecs_add(e1, sprite2d, NULL);
        ecs_add(e2, trans2d,  NULL);
        ecs_add(e2, sprite2d, NULL);
        ecs_add(e2, rb2d,     NULL);

        ecs_view_t v = ecs_query_mask(ecs_mask(trans2d, sprite2d), ecs_mask(rb2d));
        TEST_ASSERT(v.cnt     == 1U, "expected 1 match for exclusion query, got %u", v.cnt);
        TEST_ASSERT(v.ents[0] == e1, "exclusion query matched wrong entity");

        /* overlapping with and without mask must short-circuit to empty */
        ecs_view_t v_conflict = ecs_query_mask(ecs_mask(trans2d, sprite2d), ecs_mask(sprite2d));
        TEST_ASSERT(v_conflict.cnt  == 0U,   "conflicting masks must yield 0 matches");
        TEST_ASSERT(v_conflict.ents == NULL, "conflicting masks must yield NULL ents");
    } TEST_CASE_END();

    TEST_CASE("ecs_query_dead", "skips entities marked dead before sync") {
        reset_world();
        entity_handle_t e1 = ecs_spawn();
        entity_handle_t e2 = ecs_spawn();

        ecs_add(e1, trans2d, NULL);
        ecs_add(e2, trans2d, NULL);

        ecs_destroy(e1);
        ecs_view_t v = ecs_query_comp(trans2d);
        TEST_ASSERT(v.cnt     == 1U, "query must exclude dead e1, got %u", v.cnt);
        TEST_ASSERT(v.ents[0] == e2, "query matched wrong surviving entity");
    } TEST_CASE_END();

    TEST_CASE("ecs_find_tag_found", "returns handle for matching tag string") {
        reset_world();
        entity_handle_t p = ecs_spawn();
        tag             t = {0};
        strncpy((char *)t.name, "matching", 31);
        ecs_add(p, tag, &t);

        TEST_ASSERT(ecs_find_tag("matching") == p, "failed to find entity by tag 'player'");
    } TEST_CASE_END();

    TEST_CASE("ecs_find_tag_miss", "returns ENTITY_NULL for unknown or dead tag") {
        reset_world();
        entity_handle_t p = ecs_spawn();
        tag             t = {0};
        strncpy((char *)t.name, "dead", 31);
        ecs_add(p, tag, &t);

        TEST_ASSERT(ecs_find_tag("unknown") == ENTITY_NULL, "unknown tag must return ENTITY_NULL");
        ecs_destroy(p);
        TEST_ASSERT(ecs_find_tag("dead")    == ENTITY_NULL, "dead entity tag must return ENTITY_NULL");
    } TEST_CASE_END();

    ecs_set_world(NULL);
    free(perm_buf);
    free(scratch_buf);

    TEST_END();
}
