#pragma once

/*
====================
physics.h

Built-in 2D and 3D impulse physics subsystem.
Operates directly on ECS component pools
(`trans2d/3d`, `rb2d/3d`, `collider2d/3d`) and
`world->scratch` memory to guarantee deterministic
`ecs_world_rollback` compatibility with zero
persistent heap allocations.
====================
*/

#include "arena.h"
#include "core.h"
#include "ecs.h"
#include "gmath.h"
#include "spatial.h"

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================================== */
/* >CONSTANTS                                                                                     */
/* ============================================================================================== */

#define PHYSICS_INIT_HIT_CAP 64U

typedef enum physics_ray_flags {
    PHYSICS_RAY_SOLID   = BIT(0), /* match non-trigger solid colliders */
    PHYSICS_RAY_TRIGGER = BIT(1), /* match trigger sensor colliders    */
    PHYSICS_RAY_ALL     = PHYSICS_RAY_SOLID | PHYSICS_RAY_TRIGGER,
} physics_ray_flags_t;

/* ============================================================================================== */
/* >2D PHYSICS TYPES                                                                              */
/* ============================================================================================== */

/*
====================
physics_hit2d_t

Public 2D collision or trigger overlap event record.

Fields:
- `ent_a`:      first colliding entity handle (`entity_idx(ent_a) < entity_idx(ent_b)`)
- `ent_b`:      second colliding entity handle
- `normal`:     unit collision normal in 2D world space pointing from `ent_a` toward `ent_b`
- `depth`:      penetration depth along `normal` in world units (>0.0f)
- `is_trigger`: `true` if either collider is a trigger sensor
====================
*/
typedef struct physics_hit2d {
    entity_handle_t ent_a;
    entity_handle_t ent_b;
    vec2f           normal;
    float           depth;
    bool            is_trigger;
} physics_hit2d_t;

/*
====================
physics_ray_hit2d_t

Public 2D raycast intersection result.

Fields:
- `ent`:        hit entity handle
- `point`:      world-space intersection point (`origin + dir * dist`)
- `normal`:     outward surface unit normal of the hit AABB face
- `dist`:       distance from ray `origin` to `point` in world units ([0.0f, max_dist])
- `is_trigger`: `true` if the hit collider is a trigger sensor
====================
*/
typedef struct physics_ray_hit2d {
    entity_handle_t ent;
    vec2f           point;
    vec2f           normal;
    float           dist;
    bool            is_trigger;
} physics_ray_hit2d_t;

/*
====================
physics_events2d_t

Scratch-allocated 2D collision event slice produced by
`physics_step2d`. Stores a contiguous array of
`physics_hit2d_t` records (`hits`) and match count
(`cnt`), valid until the next frame scratch reset.
====================
*/
typedef struct physics_events2d {
    physics_hit2d_t *hits;
    uint32_t         cnt;
} physics_events2d_t;

/* ============================================================================================== */
/* >3D PHYSICS TYPES                                                                              */
/* ============================================================================================== */

/*
====================
physics_hit3d_t

Public 3D collision or trigger overlap event record.

Fields:
- `ent_a`:      first colliding entity handle (`entity_idx(ent_a) < entity_idx(ent_b)`)
- `ent_b`;      second colliding entity handle
- `normal`:     unit collision normal in 3D world space pointing from `ent_a` toward `ent_b`
- `depth`:      penetration depth along `normal` in world units (>0.0f)
- `is_trigger`: `true` if either collider is a trigger sensor
====================
*/
typedef struct physics_hit3d {
    entity_handle_t ent_a;
    entity_handle_t ent_b;
    vec3f           normal;
    float           depth;
    bool            is_trigger;
} physics_hit3d_t;

/*
====================
physics_ray_hit3d_t

Public 3D raycast intersection result.

Fields:
- `ent`:        hit entity handle
- `point`:      world-space intersection point (`origin + dir * dist`)
- `normal`:     outward surface unit normal of the hit AABB face
- `dist`:       distance from ray `origin` to `point` in world units ([0.0f, max_dist])
- `is_trigger`: `true` if the hit collider is a trigger sensor
====================
*/
typedef struct physics_ray_hit3d {
    entity_handle_t ent;
    vec3f           point;
    vec3f           normal;
    float           dist;
    bool            is_trigger;
} physics_ray_hit3d_t;

/*
====================
physics_events3d_t

Scratch-allocated 3D collision event slice produced by
`physics_step3d`. Stores a contiguous array of
`physics_hit3d_t` records (`hits`) and match count
(`cnt`), valid until the next frame scratch reset.
====================
*/
typedef struct physics_events3d {
    physics_hit3d_t *hits;
    uint32_t         cnt;
} physics_events3d_t;

/* ============================================================================================== */
/* >API                                                                                           */
/* ============================================================================================== */

/*
====================
physics_step2d

Advances the 2D physics simulation by `dt` seconds:
1. Integrates linear/angular velocity and frame-rate independent drag
   for dynamic `(trans2d, rb2d)` entities (`inv_mass > 0.0f`).
2. Builds a 2D spatial hash grid (`cell_sz`, or `SPATIAL_DEF_CELL_SZ` if `0.0f`)
   in `world->scratch` over all `(trans2d, collider2d)` entitites.
3. Detects AABB overlaps filtered by `collider2d.mask` (`(mask_a & mask_b) != 0`),
   resolves positional penetration and velocity impulses weighted by `inv_mass`
   for non-trigger pairs, and records hits into `*out_events` (if non-null).
If `world` is `NULL`, uses `ecs_get_world()`.
Asserts `dt >= 0.0f` and `cell_sz >= 0.0f` in debug,
returns error code in release.

Returns:
- `OK`      on successful simulation step.
- `EINVAL`  if `dt < 0.0f` or `cell_sz < 0.0f`.
- `ENOTFND` if `world` is `NULL` and no active ECS world is bound.
- `EOOM`    if `world->scratch` lacks capacity for broadphase or hit storage.
====================
*/
res_t physics_step2d(ecs_world_t *world, float dt, float cell_sz, physics_events2d_t *out_events);

/*
====================
physics_step3d

Advances the 3D physics simulation by `dt` seconds:
1. Integrates linear/angular velocity, quaternion orientation,
   and linear/angular drag for dynamic `(trans3d, rb3d)`
   entities (`inv_mass > 0.0f`).
2. Builds a 3D spatial hash grid (`cell_sz`, or `SPATIAL_DEF_CELL_SZ` if `0.0f`)
   in `world->scratch` over all `(trans3d, collider3d)` entitites.
3. Detects AABB overlaps filtered by `collider3d.mask` (`(mask_a & mask_b) != 0`),
   resolves positional penetration and velocity impulses weighted by `inv_mass`
   for non-trigger pairs, and records hits into `*out_events` (if non-null).
If `world` is `NULL`, uses `ecs_get_world()`.
Asserts `dt >= 0.0f` and `cell_sz >= 0.0f` in debug,
returns error code in release.

Returns:
- `OK`      on successful simulation step.
- `EINVAL`  if `dt < 0.0f` or `cell_sz < 0.0f`.
- `ENOTFND` if `world` is `NULL` and no active ECS world is bound.
- `EOOM`    if `world->scratch` lacks capacity for broadphase or hit storage.
====================
*/
res_t physics_step3d(ecs_world_t *world, float dt, float cell_sz, physics_events3d_t *out_events);

/*
====================
physics_step

Type-generic physics simulation step dispatching
to `physics_step2d` or `physics_step3d` based
on the `out_events` pointer type.

Returns:
- `OK`      on successful simulation step.
- `EINVAL`  if `dt < 0.0f` or `cell_sz < 0.0f`.
- `ENOTFND` if `world` is `NULL` and no active ECS world is bound.
- `EOOM`    if `world->scratch` lacks capacity for broadphase or hit storage.
====================
*/
#define physics_step(world, dt, cell_sz, out_events) _Generic((out_events),                        \
    physics_events2d_t *: physics_step2d,                                                          \
    physics_events3d_t *: physics_step3d                                                           \
    )((world), (dt), (cell_sz), (out_events))

/*
====================
physics_raycast2d

Casts a 2D ray from `origin` along `dir` up to `max_dist`
world units against active `(trans2d, collider2d)` entities
in `world`. Filters by layer `mask` (0 matches all) and
`flags` (`PHYSICS_RAY_SOLID`, `PHYSICS_RAY_TRIGGER`, or
0 / `PHYSICS_RAY_ALL`). Normalizes `dir` internally and
writes up to `max_out` closest hits into `out_hits` sorted
by `dist` ascending (`out_hits[0]` is closest).
If `world` is `NULL`, uses `ecs_get_world()`.
Asserts non-null `out_hits` and `max_dist >= 0.0f` in debug,
returns 0 in release.

Returns count of hits written to `out_hits` (up to `max_out`).
====================
*/
uint32_t physics_raycast2d(ecs_world_t *world, vec2f origin, vec2f dir, float max_dist,
                           uint32_t mask, uint32_t flags, physics_ray_hit2d_t *out_hits, uint32_t max_out);

/*
====================
physics_raycast3d

Casts a 3D ray from `origin` along `dir` up to `max_dist`
world units against active `(trans3d, collider3d)` entities
in `world`. Filters by layer `mask` (0 matches all) and
`flags` (`PHYSICS_RAY_SOLID`, `PHYSICS_RAY_TRIGGER`, or
0 / `PHYSICS_RAY_ALL`). Normalizes `dir` internally and
writes up to `max_out` closest hits into `out_hits` sorted
by `dist` ascending (`out_hits[0]` is closest).
If `world` is `NULL`, uses `ecs_get_world()`.
Asserts non-null `out_hits` and `max_dist >= 0.0f` in debug,
returns 0 in release.

Returns count of hits written to `out_hits` (up to `max_out`).
====================
*/
uint32_t physics_raycast3d(ecs_world_t *world, vec3f origin, vec3f dir, float max_dist,
                           uint32_t mask, uint32_t flags, physics_ray_hit3d_t *out_hits, uint32_t max_out);

/*
====================
physics_raycast

Type-generic raycast query dispatching to `physics_raycast2d`
or `physics_raycast3d` based on the `out_hits` pointer type.

Returns count of hits written to `out_hits` (up to `max_out`).
====================
*/
#define physics_raycast(world, origin, dir, max_dist, mask, flags, out_hits, max_out)              \
    _Generic((out_hits),                                                                           \
        physics_ray_hit2d_t *: physics_raycast2d,                                                  \
        physics_ray_hit3d_t *: physics_raycast3d)                                                  \
    ((world), (origin), (dir), (max_dist), (mask), (flags), (out_hits), (max_out))
