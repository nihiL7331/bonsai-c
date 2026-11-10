#include "core.h"
#include "arena.h"
#include "reflect.h"
#include "ecs.h"
#include "test.h"

#include <stdlib.h>
#include <string.h>

static arena_t s_perm_arena;
static arena_t s_backup_arena;
static arena_t s_scratch_arena;

static ecs_world_t *reset_world(void) {
    arena_reset(&s_perm_arena);
    arena_reset(&s_backup_arena);
    arena_reset(&s_scratch_arena);
    return ecs_world_create(&s_perm_arena, &s_backup_arena, &s_scratch_arena);
}

int main(void) {
    if (arena_init_vm(&s_perm_arena,    0) != OK ||
        arena_init_vm(&s_backup_arena,  0) != OK ||
        arena_init_vm(&s_scratch_arena, 0) != OK) return 1;

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

    TEST_CASE("ecs_snap_rollback", "captures and restores world across muts") {
        ecs_world_t    *w  = reset_world();
        entity_handle_t e1 = ecs_spawn();
        entity_handle_t e2 = ecs_spawn();

        trans2d t1 = { .x = 10.0f, .y = 20.0f, .rot = 0.0f, .scale = 1.0f };
        trans2d t2 = { .x = 30.0f, .y = 40.0f, .rot = 1.0f, .scale = 2.0f };
        ecs_add(e1, trans2d,  &t1);
        ecs_add(e1, sprite2d, NULL);
        ecs_add(e2, trans2d,  &t2);

        TEST_ASSERT(ecs_world_snapshot() == OK, "ecs_world_snapshot failed");

        /* mutate e1, destroy e2, spawn e3, and sync */
        ecs_get(e1, trans2d)->x = 999.0f;
        ecs_add(e1, rb2d, NULL);
        ecs_destroy(e2);
        entity_handle_t e3 = ecs_spawn();
        ecs_add(e3, tag, NULL);
        ecs_sync();

        TEST_ASSERT(ecs_get(e1, trans2d)->x == 999.0f, "expected mutated e1.x == 999");
        TEST_ASSERT(ecs_has(e1, rb2d),                 "expected e1 to own rb2d before rollback");
        TEST_ASSERT(!ecs_is_alive(e2),                 "expected e2 dead before rollback");
        TEST_ASSERT(ecs_is_alive(e3),                  "expected e3 alive before rollback");

        /* first rollback */
        TEST_ASSERT(ecs_world_rollback() == OK, "first ecs_world_rollback failed");
        TEST_ASSERT(w->mgr.alive_cnt == 2U,             "alive_cnt mismatch after rollback");
        TEST_ASSERT(ecs_is_alive(e1),                   "e1 must be alive after rollback");
        TEST_ASSERT(ecs_is_alive(e2),                   "e2 must be restored alive after rollback");
        TEST_ASSERT(!ecs_is_alive(e3),                  "e3 must not exist after rollback");
        TEST_ASSERT(!ecs_has(e1, rb2d),                 "e1 must not own rb2d after rollback");
        TEST_ASSERT(ecs_has(e1, sprite2d),              "e1 must still own sprite2d after rollback");
        TEST_ASSERT(ecs_get(e1, trans2d)->x == 10.0f,   "e1.x not restored to 10.0f");
        TEST_ASSERT(ecs_get(e2, trans2d)->x == 30.0f,   "e2.x not restored to 30.0f");

        /* queue unsynced deferred commands and mutate again, then rollback a 2nd time */
        ecs_destroy(e1);
        ecs_del(e2, trans2d);
        TEST_ASSERT(w->cmd_cnt == 2U,                   "expected 2 queued deferred commands");
        TEST_ASSERT(ecs_world_rollback() == OK,         "second repeated ecs_world_rollback failed");
        TEST_ASSERT(w->cmd_cnt == 0U,                   "rollback must discard pending deferred cmds");
        TEST_ASSERT(ecs_is_alive(e1),                   "e1 must be alive after 2nd rollback");
        TEST_ASSERT(ecs_get(e2, trans2d)->y == 40.0f,   "e2.y not intact after 2nd rollback");
    } TEST_CASE_END();

    TEST_CASE("ecs_rollback_empty", "returns ENOTFND when roll back without snapshot") {
        reset_world();
        entity_handle_t e = ecs_spawn();
        ecs_add(e, trans2d, NULL);

        TEST_ASSERT(ecs_world_rollback() == ENOTFND, "rollback without snapshot must return ENOTFND");
    } TEST_CASE_END();

    TEST_CASE("ecs_hier_parent", "attaches and detaches children in list") {
        reset_world();
        entity_handle_t root = ecs_spawn();
        entity_handle_t c1   = ecs_spawn();
        entity_handle_t c2   = ecs_spawn();

        TEST_ASSERT(!ecs_has(root, hierarchy),          "unparented entity must not own hierarchy");
        TEST_ASSERT(ecs_set_parent(c1, root) == OK,     "failed to parent c1 to root");
        TEST_ASSERT(ecs_set_parent(c2, root) == OK,     "failed to parent c2 to root");
        TEST_ASSERT(ecs_has(root, hierarchy),           "root must lazily acquire hierarchy");
        TEST_ASSERT(ecs_has(c1,   hierarchy),           "c1 must lazily acquire hierarchy");

        const hierarchy *hr  = ecs_get(root, hierarchy);
        const hierarchy *hc1 = ecs_get(c1,   hierarchy);
        const hierarchy *hc2 = ecs_get(c2,   hierarchy);

        TEST_ASSERT(hr->child_cnt     == 2U,          "expected root child_cnt == 2");
        TEST_ASSERT(hr->first_child   == c2,          "expected c2 at head of child list");
        TEST_ASSERT(hc2->parent       == root,        "c2 parent mismatch");
        TEST_ASSERT(hc2->prev_sibling == ENTITY_NULL, "c2 prev_sibling must be NULL");
        TEST_ASSERT(hc2->next_sibling == c1,          "c2 next_sibling must be c1");
        TEST_ASSERT(hc1->parent       == root,        "c1 parent mismatch");
        TEST_ASSERT(hc1->prev_sibling == c2,          "c1 prev_sibling must be c2");
        TEST_ASSERT(hc1->next_sibling == ENTITY_NULL, "c1 next_sibling must be NULL");

        /* detach head child (c2) via ecs_remove_parent */
        TEST_ASSERT(ecs_remove_parent(c2) == OK,        "failed to detach c2");
        TEST_ASSERT(hr->child_cnt         == 1U,        "expected root child_cnt == 1");
        TEST_ASSERT(hr->first_child       == c1,        "expected c1 to become first_child");
        TEST_ASSERT(hc1->prev_sibling == ENTITY_NULL,   "c1 prev_sibling must be cleared");
        TEST_ASSERT(hc2->parent       == ENTITY_NULL,   "c2 parent must be cleared");

        /* detach remaining child (c1) via ecs_set_parent(c1, ENTITY_NULL) */
        TEST_ASSERT(ecs_set_parent(c1, ENTITY_NULL) == OK, "failed to detach c1 via NULL parent");
        TEST_ASSERT(hr->child_cnt   == 0U,                 "expected root child_cnt == 0");
        TEST_ASSERT(hr->first_child == ENTITY_NULL,        "expected root first_child == NULL");
    } TEST_CASE_END();

    TEST_CASE("ecs_hier_reparent", "moves entire subtree between parents in O(1)") {
        reset_world();
        entity_handle_t root_a = ecs_spawn();
        entity_handle_t root_b = ecs_spawn();
        entity_handle_t branch = ecs_spawn();
        entity_handle_t leaf   = ecs_spawn();

        TEST_ASSERT(ecs_set_parent(branch, root_a) == OK, "parent branch -> root_a");
        TEST_ASSERT(ecs_set_parent(leaf,   branch) == OK, "parent leaf -> branch");

        /* move branch from root_a to root_b */
        TEST_ASSERT(ecs_set_parent(branch, root_b) == OK, "reparent branch -> root_b");

        TEST_ASSERT(ecs_get(root_a, hierarchy)->child_cnt   == 0U,          "root_a must have 0 children");
        TEST_ASSERT(ecs_get(root_a, hierarchy)->first_child == ENTITY_NULL, "root_a first_child must be NULL");
        TEST_ASSERT(ecs_get(root_b, hierarchy)->child_cnt   == 1U,          "root_b must have 1 child");
        TEST_ASSERT(ecs_get(root_b, hierarchy)->first_child == branch,      "root_b first_child must be branch");
        TEST_ASSERT(ecs_get(branch, hierarchy)->parent      == root_b,      "branch parent must be root_b");
        TEST_ASSERT(ecs_get(leaf,   hierarchy)->parent      == branch,      "leaf must remain attached to branch");
    } TEST_CASE_END();

    TEST_CASE("ecs_hier_cycle", "rejects self-parenting and ancestor cycles") {
        reset_world();
        entity_handle_t a = ecs_spawn();
        entity_handle_t b = ecs_spawn();
        entity_handle_t c = ecs_spawn();

        TEST_ASSERT(ecs_set_parent(b, a) == OK,     "b -> a must succeed");
        TEST_ASSERT(ecs_set_parent(c, b) == OK,     "c -> b must succeed");

        /* self-parenting and ancestor cycle attempts must return EINVAL */
        TEST_ASSERT(ecs_set_parent(a, a) == EINVAL, "self-parenting must return EINVAL");
        TEST_ASSERT(ecs_set_parent(a, b) == EINVAL, "2-node cycle (a -> b -> a) must return EINVAL");
        TEST_ASSERT(ecs_set_parent(a, c) == EINVAL, "3-node cycle (a -> c -> b -> a) must return EINVAL");
        TEST_ASSERT(ecs_get(a, hierarchy)->parent == ENTITY_NULL, "a must remain root after rejected cycles");
    } TEST_CASE_END();

    TEST_CASE("ecs_hier_cascade", "unlinks and cascades destruction") {
        ecs_world_t    *w      = reset_world();
        entity_handle_t root   = ecs_spawn();
        entity_handle_t branch = ecs_spawn();
        entity_handle_t leaf1  = ecs_spawn();
        entity_handle_t leaf2  = ecs_spawn();

        ecs_set_parent(branch, root);
        ecs_set_parent(leaf1,  branch);
        ecs_set_parent(leaf2,  branch);

        /* destroy middle node `branch`; root must stay alive with 0 children */
        ecs_destroy(branch);

        TEST_ASSERT(ecs_is_alive(root),    "root must survive child subtree destruction");
        TEST_ASSERT(!ecs_is_alive(branch), "branch must be marked dead immediately");
        TEST_ASSERT(!ecs_is_alive(leaf1),  "leaf1 must be marked dead via cascade");
        TEST_ASSERT(!ecs_is_alive(leaf2),  "leaf2 must be marked dead via cascade");
        TEST_ASSERT(ecs_get(root, hierarchy)->child_cnt   == 0U,          "root child_cnt must be 0");
        TEST_ASSERT(ecs_get(root, hierarchy)->first_child == ENTITY_NULL, "root first_child must be NULL");

        ecs_sync();
        TEST_ASSERT(w->mgr.alive_cnt == 1U, "only root must remain alive after sync");
    } TEST_CASE_END();

    TEST_CASE("ecs_world_trans2d", "accumulates 2D pos, rot, and scale down hierarchy") {
        reset_world();
        entity_handle_t root   = ecs_spawn();
        entity_handle_t folder = ecs_spawn(); /* organizational node without trans2d */
        entity_handle_t child  = ecs_spawn();

        /* root at (10, 20), rotated +90 deg (PI/2), scale 2.0 */
        trans2d tr = { .x = 10.0f, .y = 20.0f, .rot = 1.57079632679f, .scale = 2.0f };
        /* child at local (3, 0), rot 0.5, scale 0.5 */
        trans2d tc = { .x = 3.0f,  .y = 0.0f,  .rot = 0.5f,           .scale = 0.5f };

        ecs_add(root,  trans2d, &tr);
        ecs_add(child, trans2d, &tc);

        ecs_set_parent(folder, root);
        ecs_set_parent(child,  folder);

        trans2d wt = {0};
        TEST_ASSERT(ecs_get_world_trans(child, &wt), "ecs_get_world_trans2d failed");

        /* local (3, 0) scaled by 2 -> (6, 0), rotated +90 deg -> (0, 6), + (10, 20) = (10, 26) */
        TEST_ASSERT(ABS(wt.x     - 10.0f) < 1e-4f, "expected world x ~ 10.0, got %.4f", wt.x);
        TEST_ASSERT(ABS(wt.y     - 26.0f) < 1e-4f, "expected world y ~ 26.0, got %.4f", wt.y);
        TEST_ASSERT(ABS(wt.scale - 1.0f)  < 1e-4f, "expected world scale ~ 1.0, got %.4f", wt.scale);
        TEST_ASSERT(ABS(wt.rot   - (tr.rot + tc.rot)) < 1e-4f, "world rot mismatch");
    } TEST_CASE_END();

    TEST_CASE("ecs_world_trans3d", "accumulates 3D pos, quatf, and scale down hier") {
        reset_world();
        entity_handle_t parent = ecs_spawn();
        entity_handle_t child  = ecs_spawn();

        /* 90 deg rotation around Z axis: q = (0, 0, sin(45 deg), cos(45 deg)) */
        quatf   qz90 = quatf_axis_angle(vec3f(0.0f, 0.0f, 1.0f), 1.57079632679f);
        trans3d tp   = {
            .x = 5.0f, .y = 10.0f, .z = 15.0f,
            .qx = qz90.x, .qy = qz90.y, .qz = qz90.z, .qw = qz90.w,
            .scale_x = 2.0f, .scale_y = 2.0f, .scale_z = 3.0f,
        };
        trans3d tc = {
            .x = 4.0f, .y = 0.0f, .z = 2.0f,
            .qx = 0.0f, .qy = 0.0f, .qz = 0.0f, .qw = 1.0f,
            .scale_x = 0.5f, .scale_y = 1.0f, .scale_z = 1.0f,
        };

        ecs_add(parent, trans3d, &tp);
        ecs_add(child,  trans3d, &tc);
        ecs_set_parent(child, parent);

        trans3d wt = {0};
        TEST_ASSERT(ecs_get_world_trans(child, &wt), "ecs_get_world_trans3d failed");

        /* local (4, 0, 2) scaled by (2, 2, 3) -> (8, 0, 6), rot +90Z -> (0, 8, 6), + (5, 10, 15) = (5, 18, 21) */
        TEST_ASSERT(ABS(wt.x       - 5.0f)  < 1e-4f, "expected world x ~ 5.0, got %.4f",  wt.x);
        TEST_ASSERT(ABS(wt.y       - 18.0f) < 1e-4f, "expected world y ~ 18.0, got %.4f", wt.y);
        TEST_ASSERT(ABS(wt.z       - 21.0f) < 1e-4f, "expected world z ~ 21.0, got %.4f", wt.z);
        TEST_ASSERT(ABS(wt.scale_x - 1.0f)  < 1e-4f, "expected world scale_x ~ 1.0, got %.4f", wt.scale_x);
    } TEST_CASE_END();

    ecs_set_world(NULL);
    arena_free_vm(&s_perm_arena);
    arena_free_vm(&s_backup_arena);
    arena_free_vm(&s_scratch_arena);

    TEST_END();
}
