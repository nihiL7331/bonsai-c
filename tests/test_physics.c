#include "physics.h"
#include "arena.h"
#include "ecs.h"
#include "gmath.h"
#include "test.h"

#include <math.h>
#include <stdlib.h>

#define PI_F 3.14159265f

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
    if (arena_init_vm(&s_perm_arena,    MB(64)) != OK) return 1;
    if (arena_init_vm(&s_backup_arena,  MB(64)) != OK) return 1;
    if (arena_init_vm(&s_scratch_arena, MB(16)) != OK) return 1;

    TEST_CASE("physics_empty", "steps empty 2D and 3D worlds safely") {
        ecs_world_t *w = reset_world();

        physics_events2d_t ev2d = {0};
        res_t r2d = physics_step(w, 0.016f, 0.0f, &ev2d);
        TEST_ASSERT(r2d == OK,        "2D empty step failed: %d", r2d);
        TEST_ASSERT(ev2d.cnt == 0,    "2D empty step exp 0 hits, got %u", ev2d.cnt);
        TEST_ASSERT(ev2d.hits == NULL, "2D empty step hits exp NULL");

        physics_events3d_t ev3d = {0};
        res_t r3d = physics_step(w, 0.016f, 0.0f, &ev3d);
        TEST_ASSERT(r3d == OK,        "3D empty step failed: %d", r3d);
        TEST_ASSERT(ev3d.cnt == 0,    "3D empty step exp 0 hits, got %u", ev3d.cnt);
        TEST_ASSERT(ev3d.hits == NULL, "3D empty step hits exp NULL");
    } TEST_CASE_END();

    TEST_CASE("physics_kin_2d", "integrates 2D vel, rot, drag, and inv_mass") {
        reset_world();

        /* dynamic body with drag */
        entity_handle_t dyn = ecs_spawn();
        ecs_add(dyn, trans2d, (&(trans2d){ .x = 10.0f, .y = 20.0f, .rot = 0.0f, .scale = 1.0f }));
        ecs_add(dyn, rb2d,    (&(rb2d){ .vel_x = 100.0f, .vel_y = -50.0f, .angular_vel = 2.0f, .inv_mass = 1.0f, .drag = 1.0f }));

        /* static body (inv_mass == 0.0f) must not move even with non-zero velocity */
        entity_handle_t stat = ecs_spawn();
        ecs_add(stat, trans2d, (&(trans2d){ .x = 5.0f, .y = 5.0f, .rot = 0.0f, .scale = 1.0f }));
        ecs_add(stat, rb2d,    (&(rb2d){ .vel_x = 999.0f, .vel_y = 999.0f, .angular_vel = 5.0f, .inv_mass = 0.0f, .drag = 0.0f }));

        float dt = 1.0f;
        res_t r  = physics_step2d(NULL, dt, 0.0f, NULL);
        TEST_ASSERT(r == OK, "step2d failed: %d", r);

        /* damp = 1 / (1 + 1*1) = 0.5 -> vel = (50, -25), pos = (10 + 50, 20 - 25) = (60, -5) */
        const trans2d *td = ecs_get(dyn, trans2d);
        const rb2d    *rd = ecs_get(dyn, rb2d);
        TEST_ASSERT(fabsf(rd->vel_x - 50.0f)  < 1e-4f, "vel_x exp 50, got %.4f",  rd->vel_x);
        TEST_ASSERT(fabsf(rd->vel_y - -25.0f) < 1e-4f, "vel_y exp -25, got %.4f", rd->vel_y);
        TEST_ASSERT(fabsf(td->x - 60.0f)      < 1e-4f, "x exp 60, got %.4f",      td->x);
        TEST_ASSERT(fabsf(td->y - -5.0f)      < 1e-4f, "y exp -5, got %.4f",      td->y);
        TEST_ASSERT(fabsf(td->rot - 2.0f)     < 1e-4f, "rot exp 2, got %.4f",     td->rot);

        const trans2d *ts = ecs_get(stat, trans2d);
        TEST_ASSERT(fabsf(ts->x - 5.0f) < 1e-5f && fabsf(ts->y - 5.0f) < 1e-5f, "static body moved");
    } TEST_CASE_END();

    TEST_CASE("physics_kin_3d", "integrates 3D vel, quat orient, and drag") {
        reset_world();

        entity_handle_t e = ecs_spawn();
        ecs_add(e, trans3d, (&(trans3d){ .x = 0.0f, .y = 0.0f, .z = 0.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f }));
        ecs_add(e, rb3d,    (&(rb3d){
            .vel_x     = 30.0f, .vel_y     = 0.0f,  .vel_z     = -15.0f,
            .ang_vel_x = 0.0f,  .ang_vel_y = 0.0f,  .ang_vel_z = PI_F,
            .inv_mass  = 1.0f,  .drag      = 0.5f,  .ang_drag  = 1.0f,
        }));

        float dt = 1.0f;
        res_t r  = physics_step3d(NULL, dt, 0.0f, NULL);
        TEST_ASSERT(r == OK, "step3d failed: %d", r);

        /* linear damp = 1 / 1.5 = 2/3 -> vel = (20, 0, -10) */
        /* ang_damp    = 1 / 2.0 = 0.5 -> ang_vel_z = PI / 2 (90 deg around Z) */
        const trans3d *t  = ecs_get(e, trans3d);
        const rb3d    *rb = ecs_get(e, rb3d);
        TEST_ASSERT(fabsf(rb->vel_x - 20.0f)            < 1e-4f, "vel_x exp 20, got %.4f", rb->vel_x);
        TEST_ASSERT(fabsf(rb->vel_z - -10.0f)           < 1e-4f, "vel_z exp -10, got %.4f", rb->vel_z);
        TEST_ASSERT(fabsf(rb->ang_vel_z - (PI_F * 0.5f)) < 1e-4f, "ang_vel_z exp PI/2, got %.4f", rb->ang_vel_z);
        TEST_ASSERT(fabsf(t->x - 20.0f)                 < 1e-4f, "x exp 20, got %.4f", t->x);
        TEST_ASSERT(fabsf(t->z - -10.0f)                < 1e-4f, "z exp -10, got %.4f", t->z);

        quatf exp_q = quatf_axis_angle(vec3f(0.0f, 0.0f, 1.0f), PI_F * 0.5f);
        TEST_ASSERT(fabsf(t->qz - exp_q.z) < 1e-4f, "qz exp %.4f, got %.4f", exp_q.z, t->qz);
        TEST_ASSERT(fabsf(t->qw - exp_q.w) < 1e-4f, "qw exp %.4f, got %.4f", exp_q.w, t->qw);
    } TEST_CASE_END();

    TEST_CASE("physics_wall_2d", "separates 2D dyn body from static wall and slides") {
        reset_world();

        /* static wall at x = [10, 20], y = [-50, 50] (center = (15, 0), half = (5, 50)) */
        entity_handle_t wall = ecs_spawn();
        ecs_add(wall, trans2d,    (&(trans2d){ .x = 15.0f, .y = 0.0f, .rot = 0.0f, .scale = 1.0f }));
        ecs_add(wall, collider2d, (&(collider2d){ .half_x = 5.0f, .half_y = 50.0f, .mask = 0x01, .trigger = false }));

        /* dynamic player at x = 7, half = (4, 4) -> [3, 11], overlapping wall [10, 20] by 1 unit on X */
        /* moving diagonally into the wall: vel = (20, 15) */
        entity_handle_t player = ecs_spawn();
        ecs_add(player, trans2d,    (&(trans2d){ .x = 7.0f, .y = 0.0f, .rot = 0.0f, .scale = 1.0f }));
        ecs_add(player, collider2d, (&(collider2d){ .half_x = 4.0f, .half_y = 4.0f, .mask = 0x01, .trigger = false }));
        ecs_add(player, rb2d,       (&(rb2d){ .vel_x = 20.0f, .vel_y = 15.0f, .inv_mass = 1.0f }));

        physics_events2d_t ev = {0};
        res_t r = physics_step(NULL, 0.0f, 32.0f, &ev);
        TEST_ASSERT(r == OK,     "step failed: %d", r);
        TEST_ASSERT(ev.cnt == 1, "exp 1 collision event, got %u", ev.cnt);
        TEST_ASSERT(!ev.hits[0].is_trigger, "exp solid hit, got trigger");
        TEST_ASSERT(fabsf(ev.hits[0].depth - 1.0f) < 1e-4f, "exp depth 1.0, got %.4f", ev.hits[0].depth);

        /* player should be pushed out to x = 6.0 (so player max_x = 10.0 == wall min_x) */
        /* normal velocity (vel_x) zeroed; tangential velocity (vel_y = 15.0) preserved */
        const trans2d *tp = ecs_get(player, trans2d);
        const rb2d    *rp = ecs_get(player, rb2d);
        TEST_ASSERT(fabsf(tp->x - 6.0f)     < 1e-4f, "player x exp 6.0, got %.4f", tp->x);
        TEST_ASSERT(fabsf(rp->vel_x - 0.0f) < 1e-4f, "player vel_x exp 0.0, got %.4f", rp->vel_x);
        TEST_ASSERT(fabsf(rp->vel_y - 15.0f) < 1e-4f, "player vel_y exp 15.0, got %.4f", rp->vel_y);
    } TEST_CASE_END();

    TEST_CASE("physics_wall_3d", "separates 3D dyn body from static wall and slides") {
        reset_world();

        /* static floor at y = [-10, 0] (center = (0, -5, 0), half = (50, 5, 50)) */
        entity_handle_t floor_ent = ecs_spawn();
        ecs_add(floor_ent, trans3d,    (&(trans3d){ .x = 0.0f, .y = -5.0f, .z = 0.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f }));
        ecs_add(floor_ent, collider3d, (&(collider3d){ .half_x = 50.0f, .half_y = 5.0f, .half_z = 50.0f, .mask = 0x01, .trigger = false }));

        /* dynamic box at y = 1.5, half = (2, 2, 2) -> y in [-0.5, 3.5], penetrating floor by 0.5 on Y */
        entity_handle_t box = ecs_spawn();
        ecs_add(box, trans3d,    (&(trans3d){ .x = 0.0f, .y = 1.5f, .z = 0.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f }));
        ecs_add(box, collider3d, (&(collider3d){ .half_x = 2.0f, .half_y = 2.0f, .half_z = 2.0f, .mask = 0x01, .trigger = false }));
        ecs_add(box, rb3d,       (&(rb3d){ .vel_x = 8.0f, .vel_y = -12.0f, .vel_z = 4.0f, .inv_mass = 1.0f }));

        physics_events3d_t ev = {0};
        res_t r = physics_step(NULL, 0.0f, 32.0f, &ev);
        TEST_ASSERT(r == OK,     "step3d failed: %d", r);
        TEST_ASSERT(ev.cnt == 1, "exp 1 hit, got %u", ev.cnt);
        TEST_ASSERT(fabsf(ev.hits[0].depth - 0.5f) < 1e-4f, "exp depth 0.5, got %.4f", ev.hits[0].depth);

        const trans3d *tb = ecs_get(box, trans3d);
        const rb3d    *rb = ecs_get(box, rb3d);
        TEST_ASSERT(fabsf(tb->y - 2.0f)     < 1e-4f, "box y exp 2.0, got %.4f", tb->y);
        TEST_ASSERT(fabsf(rb->vel_y - 0.0f) < 1e-4f, "box vel_y exp 0.0, got %.4f", rb->vel_y);
        TEST_ASSERT(fabsf(rb->vel_x - 8.0f) < 1e-4f, "box vel_x exp 8.0, got %.4f", rb->vel_x);
        TEST_ASSERT(fabsf(rb->vel_z - 4.0f) < 1e-4f, "box vel_z exp 4.0, got %.4f", rb->vel_z);
    } TEST_CASE_END();

    TEST_CASE("physics_impulse_2d", "transfers momentum and separates 2D bodies by mass") {
        reset_world();

        /* body A: mass = 1 kg (inv_mass = 1.0), center = (0, 0), half = (5, 5), vel_x = 15 */
        entity_handle_t a = ecs_spawn();
        ecs_add(a, trans2d,    (&(trans2d){ .x = 0.0f, .y = 0.0f, .scale = 1.0f }));
        ecs_add(a, collider2d, (&(collider2d){ .half_x = 5.0f, .half_y = 5.0f, .mask = 0x01 }));
        ecs_add(a, rb2d,       (&(rb2d){ .vel_x = 15.0f, .vel_y = 0.0f, .inv_mass = 1.0f }));

        /* body B: mass = 2 kg (inv_mass = 0.5), center = (7, 0), half = (5, 5), vel_x = 0 */
        /* overlap = (5 + 5) - 7 = 3.0 along X */
        entity_handle_t b = ecs_spawn();
        ecs_add(b, trans2d,    (&(trans2d){ .x = 7.0f, .y = 0.0f, .scale = 1.0f }));
        ecs_add(b, collider2d, (&(collider2d){ .half_x = 5.0f, .half_y = 5.0f, .mask = 0x01 }));
        ecs_add(b, rb2d,       (&(rb2d){ .vel_x = 0.0f, .vel_y = 0.0f, .inv_mass = 0.5f }));

        physics_events2d_t ev = {0};
        res_t r = physics_step(NULL, 0.0f, 32.0f, &ev);
        TEST_ASSERT(r == OK && ev.cnt == 1, "exp 1 hit");

        /* im_sum = 1.5 -> A moves -3 * (1.0/1.5) = -2.0; B moves +3 * (0.5/1.5) = +1.0 */
        /* j = 15 / 1.5 = 10 -> vel_a = 15 - 10*1.0 = 5.0; vel_b = 0 + 10*0.5 = 5.0 */
        const trans2d *ta = ecs_get(a, trans2d);
        const trans2d *tb = ecs_get(b, trans2d);
        const rb2d    *ra = ecs_get(a, rb2d);
        const rb2d    *rb = ecs_get(b, rb2d);

        TEST_ASSERT(fabsf(ta->x - -2.0f)    < 1e-4f, "ta->x exp -2.0, got %.4f", ta->x);
        TEST_ASSERT(fabsf(tb->x - 8.0f)     < 1e-4f, "tb->x exp 8.0, got %.4f",  tb->x);
        TEST_ASSERT(fabsf(ra->vel_x - 5.0f) < 1e-4f, "ra->vel_x exp 5.0, got %.4f", ra->vel_x);
        TEST_ASSERT(fabsf(rb->vel_x - 5.0f) < 1e-4f, "rb->vel_x exp 5.0, got %.4f", rb->vel_x);
    } TEST_CASE_END();

    TEST_CASE("physics_impulse_3d", "transfers momentum and separates 3D bodies by mass") {
        reset_world();

        entity_handle_t a = ecs_spawn();
        ecs_add(a, trans3d,    (&(trans3d){ .x = 0.0f, .y = 0.0f, .z = 0.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f }));
        ecs_add(a, collider3d, (&(collider3d){ .half_x = 5.0f, .half_y = 5.0f, .half_z = 5.0f, .mask = 0x01 }));
        ecs_add(a, rb3d,       (&(rb3d){ .vel_z = 15.0f, .inv_mass = 1.0f }));

        entity_handle_t b = ecs_spawn();
        ecs_add(b, trans3d,    (&(trans3d){ .x = 0.0f, .y = 0.0f, .z = 7.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f }));
        ecs_add(b, collider3d, (&(collider3d){ .half_x = 5.0f, .half_y = 5.0f, .half_z = 5.0f, .mask = 0x01 }));
        ecs_add(b, rb3d,       (&(rb3d){ .vel_z = 0.0f, .inv_mass = 0.5f }));

        physics_events3d_t ev = {0};
        res_t r = physics_step(NULL, 0.0f, 32.0f, &ev);
        TEST_ASSERT(r == OK && ev.cnt == 1, "exp 1 3D hit");

        const trans3d *ta = ecs_get(a, trans3d);
        const trans3d *tb = ecs_get(b, trans3d);
        const rb3d    *ra = ecs_get(a, rb3d);
        const rb3d    *rb = ecs_get(b, rb3d);

        TEST_ASSERT(fabsf(ta->z - -2.0f)    < 1e-4f, "ta->z exp -2.0, got %.4f", ta->z);
        TEST_ASSERT(fabsf(tb->z - 8.0f)     < 1e-4f, "tb->z exp 8.0, got %.4f",  tb->z);
        TEST_ASSERT(fabsf(ra->vel_z - 5.0f) < 1e-4f, "ra->vel_z exp 5.0, got %.4f", ra->vel_z);
        TEST_ASSERT(fabsf(rb->vel_z - 5.0f) < 1e-4f, "rb->vel_z exp 5.0, got %.4f", rb->vel_z);
    } TEST_CASE_END();

    TEST_CASE("physics_triggers", "records trigger overlaps and filters by layer mask") {
        reset_world();

        /* dynamic entity on layer 0x01 */
        entity_handle_t player = ecs_spawn();
        ecs_add(player, trans2d,    (&(trans2d){ .x = 0.0f, .y = 0.0f, .scale = 1.0f }));
        ecs_add(player, collider2d, (&(collider2d){ .half_x = 5.0f, .half_y = 5.0f, .mask = 0x01, .trigger = false }));
        ecs_add(player, rb2d,       (&(rb2d){ .vel_x = 10.0f, .vel_y = 0.0f, .inv_mass = 1.0f }));

        /* trigger zone on layer 0x01 overlapping player */
        entity_handle_t zone = ecs_spawn();
        ecs_add(zone, trans2d,    (&(trans2d){ .x = 2.0f, .y = 0.0f, .scale = 1.0f }));
        ecs_add(zone, collider2d, (&(collider2d){ .half_x = 5.0f, .half_y = 5.0f, .mask = 0x01, .trigger = true }));

        /* solid wall on layer 0x02 (disjoint mask from player's 0x01) overlapping player */
        entity_handle_t ghost_wall = ecs_spawn();
        ecs_add(ghost_wall, trans2d,    (&(trans2d){ .x = 1.0f, .y = 0.0f, .scale = 1.0f }));
        ecs_add(ghost_wall, collider2d, (&(collider2d){ .half_x = 5.0f, .half_y = 5.0f, .mask = 0x02, .trigger = false }));

        physics_events2d_t ev = {0};
        res_t r = physics_step(NULL, 0.0f, 32.0f, &ev);
        TEST_ASSERT(r == OK, "step failed: %d", r);

        /* only the trigger zone should match; ghost_wall is filtered out by mask */
        TEST_ASSERT(ev.cnt == 1,            "exp 1 trigger hit, got %u", ev.cnt);
        TEST_ASSERT(ev.hits[0].is_trigger,  "exp is_trigger == true");
        TEST_ASSERT(ev.hits[0].ent_a == player && ev.hits[0].ent_b == zone, "unexpected hit pair");

        /* player position and velocity must be untouched */
        const trans2d *tp = ecs_get(player, trans2d);
        const rb2d    *rp = ecs_get(player, rb2d);
        TEST_ASSERT(fabsf(tp->x - 0.0f)      < 1e-5f, "trigger must not separate player");
        TEST_ASSERT(fabsf(rp->vel_x - 10.0f) < 1e-5f, "trigger must not alter player velocity");
    } TEST_CASE_END();

    TEST_CASE("physics_rollback", "produces identical simulation after world rollback") {
        reset_world();

        entity_handle_t a = ecs_spawn();
        ecs_add(a, trans2d,    (&(trans2d){ .x = 0.0f, .y = 0.0f, .scale = 1.0f }));
        ecs_add(a, collider2d, (&(collider2d){ .half_x = 4.0f, .half_y = 4.0f, .mask = 0x01 }));
        ecs_add(a, rb2d,       (&(rb2d){ .vel_x = 50.0f, .vel_y = 10.0f, .inv_mass = 1.0f, .drag = 0.2f }));

        entity_handle_t b = ecs_spawn();
        ecs_add(b, trans2d,    (&(trans2d){ .x = 12.0f, .y = 2.0f, .scale = 1.0f }));
        ecs_add(b, collider2d, (&(collider2d){ .half_x = 4.0f, .half_y = 4.0f, .mask = 0x01 }));
        ecs_add(b, rb2d,       (&(rb2d){ .vel_x = -20.0f, .vel_y = 0.0f, .inv_mass = 0.5f, .drag = 0.1f }));

        TEST_ASSERT(ecs_world_snapshot() == OK, "snapshot failed");

        /* run 5 frames */
        for (int i = 0; i < 5; ++i) {
            arena_reset(&s_scratch_arena);
            TEST_ASSERT(physics_step2d(NULL, 0.05f, 32.0f, NULL) == OK, "step %d failed", i);
        }

        trans2d run1_ta = *ecs_get(a, trans2d);
        trans2d run1_tb = *ecs_get(b, trans2d);
        rb2d    run1_ra = *ecs_get(a, rb2d);
        rb2d    run1_rb = *ecs_get(b, rb2d);

        /* rollback and re-run 5 frames */
        TEST_ASSERT(ecs_world_rollback() == OK, "rollback failed");
        for (int i = 0; i < 5; ++i) {
            arena_reset(&s_scratch_arena);
            TEST_ASSERT(physics_step2d(NULL, 0.05f, 32.0f, NULL) == OK, "re-step %d failed", i);
        }

        const trans2d *run2_ta = ecs_get(a, trans2d);
        const trans2d *run2_tb = ecs_get(b, trans2d);
        const rb2d    *run2_ra = ecs_get(a, rb2d);
        const rb2d    *run2_rb = ecs_get(b, rb2d);

        TEST_ASSERT(run1_ta.x == run2_ta->x && run1_ta.y == run2_ta->y, "A trans mismatch after rollback");
        TEST_ASSERT(run1_tb.x == run2_tb->x && run1_tb.y == run2_tb->y, "B trans mismatch after rollback");
        TEST_ASSERT(run1_ra.vel_x == run2_ra->vel_x && run1_ra.vel_y == run2_ra->vel_y, "A vel mismatch after rollback");
        TEST_ASSERT(run1_rb.vel_x == run2_rb->vel_x && run1_rb.vel_y == run2_rb->vel_y, "B vel mismatch after rollback");
    } TEST_CASE_END();

    TEST_CASE("physics_raycast_2d", "distance sorting, trigger flags, and masks") {
        reset_world();

        /* 1. trigger sensor at x = [10, 20] (center = 15, half = 5) */
        entity_handle_t trig = ecs_spawn();
        ecs_add(trig, trans2d,    (&(trans2d){ .x = 15.0f, .y = 0.0f, .scale = 1.0f }));
        ecs_add(trig, collider2d, (&(collider2d){ .half_x = 5.0f, .half_y = 5.0f, .mask = 0x01, .trigger = true }));

        /* 2. far solid wall at x = [50, 60] (spawned BEFORE near wall to verify distance sorting!) */
        entity_handle_t far_wall = ecs_spawn();
        ecs_add(far_wall, trans2d,    (&(trans2d){ .x = 55.0f, .y = 0.0f, .scale = 1.0f }));
        ecs_add(far_wall, collider2d, (&(collider2d){ .half_x = 5.0f, .half_y = 5.0f, .mask = 0x01, .trigger = false }));

        /* 3. near solid wall at x = [30, 40] */
        entity_handle_t near_wall = ecs_spawn();
        ecs_add(near_wall, trans2d,    (&(trans2d){ .x = 35.0f, .y = 0.0f, .scale = 1.0f }));
        ecs_add(near_wall, collider2d, (&(collider2d){ .half_x = 5.0f, .half_y = 5.0f, .mask = 0x01, .trigger = false }));

        /* 4. masked wall at x = [22, 28] on layer 0x02 */
        entity_handle_t layer2_wall = ecs_spawn();
        ecs_add(layer2_wall, trans2d,    (&(trans2d){ .x = 25.0f, .y = 0.0f, .scale = 1.0f }));
        ecs_add(layer2_wall, collider2d, (&(collider2d){ .half_x = 3.0f, .half_y = 5.0f, .mask = 0x02, .trigger = false }));

        physics_ray_hit2d_t hits[4];

        /* A. PHYSICS_RAY_ALL on mask 0x01: should hit trig (d=10), near_wall (d=30), far_wall (d=50) in order */
        uint32_t cnt = physics_raycast(NULL, vec2f(0.0f, 0.0f), vec2f(10.0f, 0.0f), 100.0f, 0x01, PHYSICS_RAY_ALL, hits, 4);
        TEST_ASSERT(cnt == 3, "exp 3 hits on mask 0x01, got %u", cnt);
        TEST_ASSERT(hits[0].ent == trig      && fabsf(hits[0].dist - 10.0f) < 1e-4f && hits[0].is_trigger,  "hit[0] mismatch");
        TEST_ASSERT(hits[1].ent == near_wall && fabsf(hits[1].dist - 30.0f) < 1e-4f && !hits[1].is_trigger, "hit[1] mismatch");
        TEST_ASSERT(hits[2].ent == far_wall  && fabsf(hits[2].dist - 50.0f) < 1e-4f && !hits[2].is_trigger, "hit[2] mismatch");
        TEST_ASSERT(fabsf(hits[1].normal.x - -1.0f) < 1e-4f && fabsf(hits[1].point.x - 30.0f) < 1e-4f,      "hit[1] normal/point mismatch");

        /* B. PHYSICS_RAY_SOLID with max_out = 1: should skip trigger and return only near_wall (closest solid) */
        cnt = physics_raycast(NULL, vec2f(0.0f, 0.0f), vec2f(1.0f, 0.0f), 100.0f, 0x01, PHYSICS_RAY_SOLID, hits, 1);
        TEST_ASSERT(cnt == 1 && hits[0].ent == near_wall, "PHYSICS_RAY_SOLID max_out=1 exp near_wall");

        /* C. PHYSICS_RAY_TRIGGER only: should return only trig */
        cnt = physics_raycast(NULL, vec2f(0.0f, 0.0f), vec2f(1.0f, 0.0f), 100.0f, 0x01, PHYSICS_RAY_TRIGGER, hits, 4);
        TEST_ASSERT(cnt == 1 && hits[0].ent == trig, "PHYSICS_RAY_TRIGGER exp trig only");
    } TEST_CASE_END();

    TEST_CASE("physics_raycast_3d", "casts rays and returns sorted inters and normals") {
        reset_world();

        entity_handle_t far_box = ecs_spawn();
        ecs_add(far_box, trans3d,    (&(trans3d){ .x = 0.0f, .y = 0.0f, .z = -40.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f }));
        ecs_add(far_box, collider3d, (&(collider3d){ .half_x = 4.0f, .half_y = 4.0f, .half_z = 4.0f, .mask = 0x01, .trigger = false }));

        entity_handle_t near_box = ecs_spawn();
        ecs_add(near_box, trans3d,    (&(trans3d){ .x = 0.0f, .y = 0.0f, .z = -15.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f }));
        ecs_add(near_box, collider3d, (&(collider3d){ .half_x = 3.0f, .half_y = 3.0f, .half_z = 3.0f, .mask = 0x01, .trigger = false }));

        physics_ray_hit3d_t hits[4];
        uint32_t cnt = physics_raycast(NULL, vec3f(0.0f, 0.0f, 0.0f), vec3f(0.0f, 0.0f, -1.0f), 100.0f, 0x01, 0, hits, 4);
        TEST_ASSERT(cnt == 2, "exp 2 3D hits, got %u", cnt);
        TEST_ASSERT(hits[0].ent == near_box && fabsf(hits[0].dist - 12.0f) < 1e-4f, "near_box dist exp 12.0, got %.4f", hits[0].dist);
        TEST_ASSERT(fabsf(hits[0].normal.z - 1.0f) < 1e-4f,                         "near_box normal.z exp +1.0, got %.4f", hits[0].normal.z);
        TEST_ASSERT(hits[1].ent == far_box  && fabsf(hits[1].dist - 36.0f) < 1e-4f, "far_box dist exp 36.0, got %.4f", hits[1].dist);
    } TEST_CASE_END();

    TEST_CASE("physics_circle_2d", "2D circle-circle and box-circle colls and rays") {
        reset_world();

        /* circle A (r = 5) at (0, 0), circle B (r = 5) at (6, 8) -> dist = 10 -> no overlap */
        /* move B to (3, 4) -> dist = 5, overlap = 5 along normal (0.6, 0.8) */
        entity_handle_t ca = ecs_spawn();
        ecs_add(ca, trans2d,    (&(trans2d){ .x = 0.0f, .y = 0.0f, .scale = 1.0f }));
        ecs_add(ca, collider2d, (&(collider2d){ .half_x = 5.0f, .mask = 0x01, .shape = COLLIDER_CIRCLE }));
        ecs_add(ca, rb2d,       (&(rb2d){ .inv_mass = 0.0f }));

        entity_handle_t cb = ecs_spawn();
        ecs_add(cb, trans2d,    (&(trans2d){ .x = 3.0f, .y = 4.0f, .scale = 1.0f }));
        ecs_add(cb, collider2d, (&(collider2d){ .half_x = 5.0f, .mask = 0x01, .shape = COLLIDER_CIRCLE }));
        ecs_add(cb, rb2d,       (&(rb2d){ .vel_x = -6.0f, .vel_y = -8.0f, .inv_mass = 1.0f }));

        physics_events2d_t ev = {0};
        TEST_ASSERT(physics_step(NULL, 0.0f, 32.0f, &ev) == OK && ev.cnt == 1, "exp 1 circle-circle hit");
        TEST_ASSERT(fabsf(ev.hits[0].depth - 5.0f)    < 1e-4f, "exp depth 5.0, got %.4f", ev.hits[0].depth);
        TEST_ASSERT(fabsf(ev.hits[0].normal.x - 0.6f) < 1e-4f, "exp normal.x 0.6, got %.4f", ev.hits[0].normal.x);
        TEST_ASSERT(fabsf(ev.hits[0].normal.y - 0.8f) < 1e-4f, "exp normal.y 0.8, got %.4f", ev.hits[0].normal.y);

        const trans2d *tb = ecs_get(cb, trans2d);
        TEST_ASSERT(fabsf(tb->x - 6.0f) < 1e-4f && fabsf(tb->y - 8.0f) < 1e-4f, "cb separated to (6, 8)");

        /* raycast against circle A at (0, 0) with r = 5 from (-15, 0) along +X -> hit at dist = 10, point = (-5, 0) */
        physics_ray_hit2d_t rhit;
        uint32_t rcnt = physics_raycast(NULL, vec2f(-15.0f, 0.0f), vec2f(1.0f, 0.0f), 50.0f, 0x01, 0, &rhit, 1);
        TEST_ASSERT(rcnt == 1 && rhit.ent == ca,       "raycast exp hit ca");
        TEST_ASSERT(fabsf(rhit.dist - 10.0f)  < 1e-4f, "circle ray dist exp 10.0, got %.4f", rhit.dist);
        TEST_ASSERT(fabsf(rhit.normal.x - -1.0f) < 1e-4f, "circle ray normal.x exp -1.0");
    } TEST_CASE_END();

    TEST_CASE("physics_sphere_3d", "3D sphere-box colls and sphere raycasts") {
        reset_world();

        /* static box [-5, -5, -5] .. [5, 5, 5] */
        entity_handle_t box = ecs_spawn();
        ecs_add(box, trans3d,    (&(trans3d){ .x = 0.0f, .y = 0.0f, .z = 0.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f }));
        ecs_add(box, collider3d, (&(collider3d){ .half_x = 5.0f, .half_y = 5.0f, .half_z = 5.0f, .mask = 0x01, .shape = COLLIDER_BOX }));

        /* dynamic sphere (r = 3) at (7, 0, 0) -> closest point on box is (5, 0, 0), dist = 2, depth = 1 */
        entity_handle_t sph = ecs_spawn();
        ecs_add(sph, trans3d,    (&(trans3d){ .x = 7.0f, .y = 0.0f, .z = 0.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f }));
        ecs_add(sph, collider3d, (&(collider3d){ .half_x = 3.0f, .mask = 0x01, .shape = COLLIDER_SPHERE }));
        ecs_add(sph, rb3d,       (&(rb3d){ .vel_x = -10.0f, .vel_y = 5.0f, .inv_mass = 1.0f }));

        physics_events3d_t ev = {0};
        TEST_ASSERT(physics_step(NULL, 0.0f, 32.0f, &ev) == OK && ev.cnt == 1, "exp 1 box-sphere hit");
        TEST_ASSERT(fabsf(ev.hits[0].depth - 1.0f) < 1e-4f, "exp depth 1.0, got %.4f", ev.hits[0].depth);

        const trans3d *ts = ecs_get(sph, trans3d);
        const rb3d    *rs = ecs_get(sph, rb3d);
        TEST_ASSERT(fabsf(ts->x - 8.0f)     < 1e-4f, "sphere x exp 8.0, got %.4f", ts->x);
        TEST_ASSERT(fabsf(rs->vel_x - 0.0f) < 1e-4f, "sphere vel_x exp 0.0");
        TEST_ASSERT(fabsf(rs->vel_y - 5.0f) < 1e-4f, "sphere tangential vel_y exp 5.0");

        /* raycast from (20, 0, 0) along -X -> hits sphere at x = 11 (dist = 9) */
        physics_ray_hit3d_t rhit;
        uint32_t rcnt = physics_raycast(NULL, vec3f(20.0f, 0.0f, 0.0f), vec3f(-1.0f, 0.0f, 0.0f), 50.0f, 0x01, 0, &rhit, 1);
        TEST_ASSERT(rcnt == 1 && rhit.ent == sph,     "raycast exp hit sph");
        TEST_ASSERT(fabsf(rhit.dist - 9.0f)  < 1e-4f, "sphere ray dist exp 9.0, got %.4f", rhit.dist);
        TEST_ASSERT(fabsf(rhit.normal.x - 1.0f) < 1e-4f, "sphere ray normal.x exp +1.0");
    } TEST_CASE_END();

    TEST_CASE("physics_capsule", "resolves 3D capsule cols and cylinder/cap rays") {
        reset_world();

        /* static floor at y = [-10, 0] */
        entity_handle_t floor_ent = ecs_spawn();
        ecs_add(floor_ent, trans3d,    (&(trans3d){ .x = 0.0f, .y = -5.0f, .z = 0.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f }));
        ecs_add(floor_ent, collider3d, (&(collider3d){ .half_x = 50.0f, .half_y = 5.0f, .half_z = 50.0f, .mask = 0x01, .shape = COLLIDER_BOX }));

        /* upright player capsule: radius = 2, half_height = 6 (spine y in [y-4, y+4], bottom at y-6) */
        /* place at y = 5.0 -> bottom at -1.0, penetrating floor by 1.0 */
        entity_handle_t cap = ecs_spawn();
        ecs_add(cap, trans3d,    (&(trans3d){ .x = 0.0f, .y = 5.0f, .z = 0.0f, .scale_x = 1.0f, .scale_y = 1.0f, .scale_z = 1.0f }));
        ecs_add(cap, collider3d, (&(collider3d){ .half_x = 2.0f, .half_y = 6.0f, .mask = 0x01, .shape = COLLIDER_CAPSULE }));
        ecs_add(cap, rb3d,       (&(rb3d){ .vel_y = -20.0f, .inv_mass = 1.0f }));

        physics_events3d_t ev = {0};
        TEST_ASSERT(physics_step(NULL, 0.0f, 32.0f, &ev) == OK && ev.cnt == 1, "exp 1 floor-capsule hit");
        TEST_ASSERT(fabsf(ev.hits[0].depth - 1.0f)    < 1e-4f, "exp depth 1.0, got %.4f", ev.hits[0].depth);
        TEST_ASSERT(fabsf(ev.hits[0].normal.y - 1.0f) < 1e-4f, "exp normal.y +1.0, got %.4f", ev.hits[0].normal.y);

        const trans3d *tc = ecs_get(cap, trans3d);
        TEST_ASSERT(fabsf(tc->y - 6.0f) < 1e-4f, "capsule pushed up to y = 6.0, got %.4f", tc->y);

        /* 1. raycast hitting capsule cylindrical body at y = 8.0 from (-10, 8, 0) along +X -> dist = 8.0, normal = (-1, 0, 0) */
        physics_ray_hit3d_t rhit;
        uint32_t rcnt = physics_raycast(NULL, vec3f(-10.0f, 8.0f, 0.0f), vec3f(1.0f, 0.0f, 0.0f), 50.0f, 0x01, 0, &rhit, 1);
        TEST_ASSERT(rcnt == 1 && rhit.ent == cap,         "exp hit capsule cylinder body");
        TEST_ASSERT(fabsf(rhit.dist - 8.0f)      < 1e-4f, "cylinder body dist exp 8.0, got %.4f", rhit.dist);
        TEST_ASSERT(fabsf(rhit.normal.x - -1.0f) < 1e-4f, "cylinder body normal.x exp -1.0");

        /* 2. raycast straight down from (0, 20, 0) onto capsule top hemisphere (top at y = 6 + 6 = 12) -> dist = 8.0, normal = (0, 1, 0) */
        rcnt = physics_raycast(NULL, vec3f(0.0f, 20.0f, 0.0f), vec3f(0.0f, -1.0f, 0.0f), 50.0f, 0x01, 0, &rhit, 1);
        TEST_ASSERT(rcnt == 1 && rhit.ent == cap,        "exp hit capsule top cap");
        TEST_ASSERT(fabsf(rhit.dist - 8.0f)     < 1e-4f, "top cap dist exp 8.0, got %.4f", rhit.dist);
        TEST_ASSERT(fabsf(rhit.normal.y - 1.0f) < 1e-4f, "top cap normal.y exp +1.0");
    } TEST_CASE_END();

    arena_free_vm(&s_perm_arena);
    arena_free_vm(&s_backup_arena);
    arena_free_vm(&s_scratch_arena);

    TEST_END();
}
