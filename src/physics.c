/*
====================
physics.c

2D and 3D impulse physics implementation.
Features:
- Frame-rate independent linear and angular damping
- Exact axis-angle quaternion angular velocity integration to 3D
- Spatial hash broadphase paired with MTV narrowphase
- Inverse-mass weighted positional separation and inelastic impulse resolution
- Scratch-allocated collision and trigger event reporting
====================
*/

#include "core.h"
#include "physics.h"
#include "spatial.h"

#include <math.h>
#include <string.h>

/* ============================================================================================== */
/* >NARROWPHASE HELPERS                                                                           */
/* ============================================================================================== */

static inline aabb3f _physics_lift_aabb2d(aabb2f b) {
    return aabb3f(b.min.x, b.min.y, -1e6f, b.max.x, b.max.y, 1e6f);
}

static inline bool _physics_mtv_aabb3d(aabb3f a, aabb3f b, vec3f *out_normal, float *out_depth) {
    vec3f half_a = vec_scale(aabb_size(a), 0.5f);
    vec3f half_b = vec_scale(aabb_size(b), 0.5f);
    vec3f delta  = vec_sub(aabb_center(b), aabb_center(a));

    vec3f pen = vec_sub(vec_add(half_a, half_b), vec_abs(delta));
    if (pen.x <= 0.0f || pen.y <= 0.0f || pen.z <= 0.0f) return false;

    if (pen.x <= pen.y && pen.x <= pen.z) {
        *out_normal = vec3f((delta.x < 0.0f) ? -1.0f : 1.0f, 0.0f, 0.0f);
        *out_depth  = pen.x;
    } else if (pen.y <= pen.z) {
        *out_normal = vec3f(0.0f, (delta.y < 0.0f) ? -1.0f : 1.0f, 0.0f);
        *out_depth = pen.y;
    } else {
        *out_normal = vec3f(0.0f, 0.0f, (delta.z < 0.0f) ? -1.0f : 1.0f);
        *out_depth = pen.z;
    }

    return true;
}

static inline bool _physics_mtv_round3d(aabb3f a, aabb3f b, vec3f *out_normal, float *out_depth) {
    vec3f ca    = aabb_center(a);
    vec3f cb    = aabb_center(b);
    float ra    = (a.max.x - a.min.x) * 0.5f;
    float rb    = (b.max.x - b.min.x) * 0.5f;
    float r_sum = ra + rb;

    float ya = CLAMP(cb.y, a.min.y + ra, a.max.y - ra);
    float yb = CLAMP(ya,   b.min.y + rb, b.max.y - rb);
    ca.y     = CLAMP(yb,   a.min.y + ra, a.max.y - ra);
    cb.y     = yb;

    vec3f delta = vec_sub(cb, ca);
    float d_sq  = vec_len_sq(delta);
    if (d_sq >= r_sum * r_sum) return false;

    float d     = sqrtf(d_sq);
    *out_depth  = r_sum - d;
    *out_normal = (d > 1e-6f) ? vec_scale(delta, 1.0f / d) : vec3f(1, 0, 0);

    return true;
}

static inline bool _physics_mtv_box_round3d(aabb3f box, aabb3f round_b, vec3f *out_normal, float *out_depth) {
    vec3f cb   = aabb_center(round_b);
    float rb   = (round_b.max.x - round_b.min.x) * 0.5f;
    float sp_y = CLAMP((box.min.y + box.max.y) * 0.5f, round_b.min.y + rb, round_b.max.y - rb);
    sp_y       = CLAMP(sp_y,                           box.min.y,          box.max.y);
    cb.y       = CLAMP(sp_y,                           round_b.min.y + rb, round_b.max.y - rb);

    vec3f closest = vec_clamp(cb, box.min, box.max);
    vec3f delta   = vec_sub(cb, closest);
    float d_sq    = vec_len_sq(delta);
    if (d_sq >= rb * rb) return false;
    if (d_sq <= 1e-12f)  return _physics_mtv_aabb3d(box, round_b, out_normal, out_depth);

    float d     = sqrtf(d_sq);
    *out_depth  = rb - d;
    *out_normal = vec_scale(delta, 1.0f / d);

    return true;
}

static inline bool _physics_mtv3d(aabb3f box_a, uint8_t shape_a, aabb3f box_b, uint8_t shape_b, vec3f *out_normal, float *out_depth) {
    if (shape_a == COLLIDER_BOX && shape_b == COLLIDER_BOX)
        return _physics_mtv_aabb3d(box_a, box_b, out_normal, out_depth);
    if (shape_a != COLLIDER_BOX && shape_b != COLLIDER_BOX)
        return _physics_mtv_round3d(box_a, box_b, out_normal, out_depth);
    if (shape_a == COLLIDER_BOX)
        return _physics_mtv_box_round3d(box_a, box_b, out_normal, out_depth);
    /* shape_a != COLLIDER_BOX && shape_b == COLLIDER_BOX */
    if (!_physics_mtv_box_round3d(box_b, box_a, out_normal, out_depth))
        return false;
    *out_normal = vec_scale(*out_normal, -1.0f);
    return true;
}

/* ============================================================================================== */
/* >RAYCAST HELPERS                                                                               */
/* ============================================================================================== */

static inline bool _physics_ray_aabb3d(vec3f origin, vec3f dir, float max_dist, aabb3f box, vec3f *out_normal, float *out_dist) {
    float t_min = 0.0f, t_max = max_dist;
    vec3f norm = {0};

    for (size_t i = 0; i < 3; ++i) {
        if (ABS(dir.v[i]) < 1e-8f) {
            if (origin.v[i] < box.min.v[i] || origin.v[i] > box.max.v[i]) return false;
            continue;
        } else {
            float inv_d = 1.0f / dir.v[i];
            float t1    = (box.min.v[i] - origin.v[i]) * inv_d;
            float t2    = (box.max.v[i] - origin.v[i]) * inv_d;
            float sgn   = -1.0f;

            if (t1 > t2)       { SWAP(t1, t2); sgn = 1.0f;                           }
            if (t1 > t_min)    { t_min = t1; norm = vec3f_splat(0); norm.v[i] = sgn; }
            if (t2 < t_max)    { t_max = t2;                                         }
            if (t_min > t_max) { return false;                                       }
        }
    }

    if (norm.x == 0.0f && norm.y == 0.0f && norm.z == 0.0f) norm = vec_scale(dir, -1.0f);
    *out_normal = norm;
    *out_dist   = t_min;
    return true;
}

static inline bool _physics_ray_sphere3d(vec3f origin, vec3f dir, float max_dist, vec3f c, float r, vec3f *out_normal, float *out_dist) {
    if (r <= 0.0f) return false;

    vec3f m = vec_sub(origin, c);
    float b = vec_dot(m, dir);
    float k = vec_len_sq(m) - r * r;
    if (k > 0.0f && b > 0.0f) return false;

    float disc = b * b - k;
    if (disc < 0.0f) return false;

    float t = -b - sqrtf(disc);
    if (t < 0.0f) {
        *out_dist   = 0.0f;
        *out_normal = vec_scale(dir, -1.0f);

        return true;
    }
    if (t > max_dist) return false;

    *out_dist   = t;
    *out_normal = vec_scale(vec_add(m, vec_scale(dir, t)), 1.0f / r);

    return true;
}

static inline bool _physics_ray_shape3d(vec3f origin, vec3f dir, float max_dist, aabb3f box, uint8_t shape, vec3f *out_normal, float *out_dist) {
    if (shape == COLLIDER_BOX) return _physics_ray_aabb3d(origin, dir, max_dist, box, out_normal, out_dist);

    vec3f c = aabb_center(box);
    float r = (box.max.x - box.min.x) * 0.5f;
    if (shape == COLLIDER_SPHERE) return _physics_ray_sphere3d(origin, dir, max_dist, c, r, out_normal, out_dist);

    /* upright capsule: infinite XZ cylinder clamped to [y_bot, y_top] + 2 endcap spheres */
    float y_bot = box.min.y + r, y_top = box.max.y - r;
    float best  = max_dist;
    bool  hit   = false;
    vec3f n;
    float d;

    vec3f dir_xz = vec3f(dir.x, 0.0f, dir.z);
    float len_xz = vec_len(dir_xz);
    if (y_top > y_bot && len_xz > 1e-6f) {
        vec3f n_xz;
        float d2d;

        if (_physics_ray_sphere3d(vec3f(origin.x, 0.0f, origin.z), vec_scale(dir_xz, 1.0f / len_xz),
                                                best * len_xz, vec3f(c.x, 0.0f, c.z), r, &n_xz, &d2d)) {
            float t3d   = d2d / len_xz;
            float hit_y = origin.y + dir.y * t3d;

            if (t3d <= best && hit_y >= y_bot && hit_y <= y_top) {
                best        = t3d;
                *out_normal = n_xz;
                *out_dist   = t3d;
                hit         = true;
            }
        }
    }

    if (_physics_ray_sphere3d(origin, dir, best, vec3f(c.x, y_bot, c.z), r, &n, &d)) {
        best        = d;
        *out_normal = n;
        *out_dist   = d;
        hit         = true;
    }

    if (_physics_ray_sphere3d(origin, dir, best, vec3f(c.x, y_top, c.z), r, &n, &d)) {
        *out_normal = n;
        *out_dist   = d;
        hit         = true;
    }

    return hit;
}

/* ============================================================================================== */
/* >API                                                                                           */
/* ============================================================================================== */

res_t physics_step2d(ecs_world_t *world, float dt, float cell_sz, physics_events2d_t *out_events) {
    ASSERT(dt >= 0.0f,      "physics_step2d called with negative dt (%.4f)",      dt);
    ASSERT(cell_sz >= 0.0f, "physics_step2d called with negative cell_sz (%.4f)", cell_sz);
    if (out_events)                  *out_events = (physics_events2d_t){0};
    if (dt < 0.0f || cell_sz < 0.0f) return EINVAL;

    ecs_world_t *w = world ? world : ecs_get_world();
    if (!w)          return ENOTFND;
    if (!w->scratch) return EINVAL;

    ecs_world_t *prev = ecs_get_world();
    if (world) ecs_set_world(world);

    /* kinematics / damping integration */
    if (dt > 0.0f) {
        ecs_view_t dyn_view = ecs_query_comp(trans2d, rb2d);

        for (uint32_t i = 0; i < dyn_view.cnt; ++i) {
            entity_handle_t ent = dyn_view.ents[i];
            rb2d           *rb  = ecs_get_fast(w, ent, rb2d);
            if (rb->inv_mass <= 0.0f) continue;

            if (rb->drag > 0.0f) {
                float damp = 1.0f / (1.0f + rb->drag * dt);
                rb->velocity = vec_scale(rb->velocity, damp);
            }

            trans2d *t   = ecs_get_fast(w, ent, trans2d);
            t->position  = vec_add(t->position, vec_scale(rb->velocity, dt));
            t->rotation += rb->angular_velocity * dt;
        }
    }

    /* broadphase spatial hash construction */
    spatial_grid2d_t grid;
    res_t res = spatial_grid2d_init(&grid, cell_sz, 0);
    if (res != OK) { if (world) ecs_set_world(prev); return res; }

    res = spatial_grid2d_build_from_ecs(&grid, w, w->scratch);
    if (res == WEMPTY) { if (world) ecs_set_world(prev); return OK;  }
    if (res != OK)     { if (world) ecs_set_world(prev); return res; }

    /* narrowphase / impulse resolution */
    entity_handle_t *candidates = arena_alloc(w->scratch, grid.entity_cnt * sizeof(entity_handle_t));
    if (!candidates) { if (world) ecs_set_world(prev); return EOOM; }

    physics_hit2d_t *hits = NULL;
    uint32_t         hit_cnt = 0;
    uint32_t         hit_cap = 0;

    if (out_events) {
        hit_cap = MAX(grid.entity_cnt * 2U, PHYSICS_INIT_HIT_CAP);
        hits    = arena_alloc(w->scratch, hit_cap * sizeof(physics_hit2d_t));
        if (!hits) { if (world) ecs_set_world(prev); return EOOM; }
    }

    for (uint32_t i = 0; i < grid.entity_cnt; ++i) {
        spatial_entity2d_t proxy_a = grid.entities[i];
        if (!proxy_a.mask) continue;

        entity_handle_t   ent_a = proxy_a.ent;
        const collider2d *col_a = ecs_get_fast(w, ent_a, collider2d);
        aabb2f            box_a = spatial_calc_aabb(ent_a, col_a);

        uint32_t cand_cnt = spatial_query_aabb2d(&grid, box_a, proxy_a.mask, candidates, grid.entity_cnt, w->scratch);
        for (uint32_t k = 0; k < cand_cnt; ++k) {
            entity_handle_t ent_b = candidates[k];
            if (entity_idx(ent_a) >= entity_idx(ent_b)) continue;

            const collider2d *col_b = ecs_get_fast(w, ent_b, collider2d);
            if (!(col_a->mask & col_b->mask)) continue;

            bool  is_trigger = col_a->trigger || col_b->trigger;
            rb2d *rb_a       = ecs_has(ent_a, rb2d)            ? ecs_get_fast(w, ent_a, rb2d) : NULL;
            rb2d *rb_b       = ecs_has(ent_b, rb2d)            ? ecs_get_fast(w, ent_b, rb2d) : NULL;
            float im_a       = (rb_a && rb_a->inv_mass > 0.0f) ? rb_a->inv_mass               : 0.0f;
            float im_b       = (rb_b && rb_b->inv_mass > 0.0f) ? rb_b->inv_mass               : 0.0f;
            float im_sum     = im_a + im_b;

            if (!is_trigger && im_sum <= 0.0f) continue;

            box_a        = spatial_calc_aabb(ent_a, col_a);
            aabb2f box_b = spatial_calc_aabb(ent_b, col_b);

            vec3f norm3d;
            float depth;
            if (!_physics_mtv3d(_physics_lift_aabb2d(box_a), col_a->shape,
                                _physics_lift_aabb2d(box_b), col_b->shape, &norm3d, &depth)) continue;
            vec2f normal = vec2f(norm3d.x, norm3d.y);

            if (!is_trigger && im_sum > 0.0f) {
                float inv_sum = 1.0f / im_sum;

                if (im_a > 0.0f) {
                    trans2d *ta  = ecs_get_fast(w, ent_a, trans2d);
                    ta->position = vec_sub(ta->position, vec_scale(normal, depth * im_a * inv_sum));
                }
                if (im_b > 0.0f) {
                    trans2d *tb  = ecs_get_fast(w, ent_b, trans2d);
                    tb->position = vec_add(tb->position, vec_scale(normal, depth * im_b * inv_sum));
                }

                vec2f vel_a  = rb_a ? rb_a->velocity : vec2f_splat(0);
                vec2f vel_b  = rb_b ? rb_b->velocity : vec2f_splat(0);
                float rel_vn = vec_dot(vec_sub(vel_b, vel_a), normal);

                if (rel_vn < 0.0f) {
                    float j = -rel_vn * inv_sum;

                    if (rb_a && im_a > 0.0f)
                        rb_a->velocity = vec_sub(rb_a->velocity, vec_scale(normal, j * im_a));

                    if (rb_b && im_b > 0.0f)
                        rb_b->velocity = vec_add(rb_b->velocity, vec_scale(normal, j * im_b));
                }
            }

            if (out_events) {
                if (hit_cnt == hit_cap) {
                    uint32_t         new_cap  = hit_cap * 2U;
                    physics_hit2d_t *new_hits = arena_alloc(w->scratch, new_cap * sizeof(physics_hit2d_t));
                    if (!new_hits) { if (world) ecs_set_world(prev); return EOOM; }

                    memcpy(new_hits, hits, hit_cnt * sizeof(physics_hit2d_t));
                    hits    = new_hits;
                    hit_cap = new_cap;
                }

                hits[hit_cnt++] = (physics_hit2d_t){
                    .ent_a      = ent_a,
                    .ent_b      = ent_b,
                    .normal     = normal,
                    .depth      = depth,
                    .is_trigger = is_trigger,
                };
            }
        }
    }

    if (out_events && hit_cnt) { out_events->hits = hits; out_events->cnt = hit_cnt; }
    if (world) ecs_set_world(prev);

    return OK;
}

res_t physics_step3d(ecs_world_t *world, float dt, float cell_sz, physics_events3d_t *out_events) {
    ASSERT(dt >= 0.0f,      "physics_step3d called with negative dt (%.4f)",      dt);
    ASSERT(cell_sz >= 0.0f, "physics_step3d called with negative cell_sz (%.4f)", cell_sz);
    if (out_events)                  *out_events = (physics_events3d_t){0};
    if (dt < 0.0f || cell_sz < 0.0f) return EINVAL;

    ecs_world_t *w = world ? world : ecs_get_world();
    if (!w)          return ENOTFND;
    if (!w->scratch) return EINVAL;

    ecs_world_t *prev = ecs_get_world();
    if (world) ecs_set_world(world);

    /* kinematics / damping integration */
    if (dt > 0.0f) {
        ecs_view_t dyn_view = ecs_query_comp(trans3d, rb3d);

        for (uint32_t i = 0; i < dyn_view.cnt; ++i) {
            entity_handle_t ent = dyn_view.ents[i];
            rb3d           *rb  = ecs_get_fast(w, ent, rb3d);
            if (rb->inv_mass <= 0.0f) continue;

            if (rb->drag > 0.0f) {
                float damp = 1.0f / (1.0f + rb->drag * dt);
                rb->velocity = vec_scale(rb->velocity, damp);
            }

            if (rb->ang_drag > 0.0f) {
                float ang_damp = 1.0f / (1.0f + rb->ang_drag * dt);
                rb->angular_velocity = vec_scale(rb->angular_velocity, ang_damp);
            }

            trans3d *t  = ecs_get_fast(w, ent, trans3d);
            t->position = vec_add(t->position, vec_scale(rb->velocity, dt));

            float ang_vel = vec_len(rb->angular_velocity);
            if (ang_vel > 1e-6f) {
                quatf dq    = quatf_axis_angle(vec_norm(rb->angular_velocity), ang_vel * dt);
                t->rotation = quatf_norm(quatf_mul(dq, t->rotation));
            }
        }
    }

    /* broadphase spatial hash construction */
    spatial_grid3d_t grid;
    res_t res = spatial_grid3d_init(&grid, cell_sz, 0);
    if (res != OK) { if (world) ecs_set_world(prev); return res; }

    res = spatial_grid3d_build_from_ecs(&grid, w, w->scratch);
    if (res == WEMPTY) { if (world) ecs_set_world(prev); return OK;  }
    if (res != OK)     { if (world) ecs_set_world(prev); return res; }

    /* narrowphase / impulse resolution */
    entity_handle_t *candidates = arena_alloc(w->scratch, grid.entity_cnt * sizeof(entity_handle_t));
    if (!candidates) { if (world) ecs_set_world(prev); return EOOM; }

    physics_hit3d_t *hits = NULL;
    uint32_t         hit_cnt = 0;
    uint32_t         hit_cap = 0;

    if (out_events) {
        hit_cap = MAX(grid.entity_cnt * 2U, PHYSICS_INIT_HIT_CAP);
        hits    = arena_alloc(w->scratch, hit_cap * sizeof(physics_hit3d_t));
        if (!hits) { if (world) ecs_set_world(prev); return EOOM; }
    }

    for (uint32_t i = 0; i < grid.entity_cnt; ++i) {
        spatial_entity3d_t proxy_a = grid.entities[i];
        if (!proxy_a.mask) continue;

        entity_handle_t   ent_a = proxy_a.ent;
        const collider3d *col_a = ecs_get_fast(w, ent_a, collider3d);
        aabb3f            box_a = spatial_calc_aabb(ent_a, col_a);

        uint32_t cand_cnt = spatial_query_aabb3d(&grid, box_a, proxy_a.mask, candidates, grid.entity_cnt, w->scratch);
        for (uint32_t k = 0; k < cand_cnt; ++k) {
            entity_handle_t ent_b = candidates[k];
            if (entity_idx(ent_a) >= entity_idx(ent_b)) continue;

            const collider3d *col_b = ecs_get_fast(w, ent_b, collider3d);
            if (!(col_a->mask & col_b->mask)) continue;

            bool  is_trigger = col_a->trigger || col_b->trigger;
            rb3d *rb_a       = ecs_has(ent_a, rb3d)            ? ecs_get_fast(w, ent_a, rb3d) : NULL;
            rb3d *rb_b       = ecs_has(ent_b, rb3d)            ? ecs_get_fast(w, ent_b, rb3d) : NULL;
            float im_a       = (rb_a && rb_a->inv_mass > 0.0f) ? rb_a->inv_mass               : 0.0f;
            float im_b       = (rb_b && rb_b->inv_mass > 0.0f) ? rb_b->inv_mass               : 0.0f;
            float im_sum     = im_a + im_b;

            if (!is_trigger && im_sum <= 0.0f) continue;

            box_a        = spatial_calc_aabb(ent_a, col_a);
            aabb3f box_b = spatial_calc_aabb(ent_b, col_b);

            vec3f normal;
            float depth;
            if (!_physics_mtv3d(box_a, col_a->shape, box_b, col_b->shape, &normal, &depth)) continue;

            if (!is_trigger && im_sum > 0.0f) {
                float inv_sum = 1.0f / im_sum;

                if (im_a > 0.0f) {
                    trans3d *ta  = ecs_get_fast(w, ent_a, trans3d);
                    ta->position = vec_sub(ta->position, vec_scale(normal, depth * im_a * inv_sum));
                }
                if (im_b > 0.0f) {
                    trans3d *tb  = ecs_get_fast(w, ent_b, trans3d);
                    tb->position = vec_add(tb->position, vec_scale(normal, depth * im_b * inv_sum));
                }

                vec3f vel_a  = rb_a ? rb_a->velocity : vec3f_splat(0);
                vec3f vel_b  = rb_b ? rb_b->velocity : vec3f_splat(0);
                float rel_vn = vec_dot(vec_sub(vel_b, vel_a), normal);

                if (rel_vn < 0.0f) {
                    float j = -rel_vn * inv_sum;

                    if (rb_a && im_a > 0.0f)
                        rb_a->velocity = vec_sub(rb_a->velocity, vec_scale(normal, j * im_a));

                    if (rb_b && im_b > 0.0f)
                        rb_b->velocity = vec_add(rb_b->velocity, vec_scale(normal, j * im_b));
                }
            }

            if (out_events) {
                if (hit_cnt == hit_cap) {
                    uint32_t         new_cap  = hit_cap * 2U;
                    physics_hit3d_t *new_hits = arena_alloc(w->scratch, new_cap * sizeof(physics_hit3d_t));
                    if (!new_hits) { if (world) ecs_set_world(prev); return EOOM; }

                    memcpy(new_hits, hits, hit_cnt * sizeof(physics_hit3d_t));
                    hits    = new_hits;
                    hit_cap = new_cap;
                }

                hits[hit_cnt++] = (physics_hit3d_t){
                    .ent_a      = ent_a,
                    .ent_b      = ent_b,
                    .normal     = normal,
                    .depth      = depth,
                    .is_trigger = is_trigger,
                };
            }
        }
    }

    if (out_events && hit_cnt) { out_events->hits = hits; out_events->cnt = hit_cnt; }
    if (world) ecs_set_world(prev);

    return OK;
}

uint32_t physics_raycast2d(ecs_world_t *world, vec2f origin, vec2f dir, float max_dist, uint32_t mask,
                           uint32_t flags, physics_ray_hit2d_t *out_hits, uint32_t max_out) {
    ASSERT(out_hits,         "physics_raycast2d called with NULL out_hits");
    ASSERT(max_dist >= 0.0f, "physics_raycast2d called with negative max_dist (%.4f)", max_dist);
    if   (!out_hits || !max_out || max_dist < 0.0f) return 0;

    dir = vec_norm(dir);
    if (dir.x == 0 && dir.y == 0) return 0;

    if (!flags) flags = PHYSICS_RAY_ALL;

    ecs_world_t *w = world ? world : ecs_get_world();
    if (!w) return 0;

    ecs_world_t *prev = ecs_get_world();
    if (world) ecs_set_world(world);

    ecs_view_t view       = ecs_query_comp(trans2d, collider2d);
    float      limit_dist = max_dist;
    uint32_t   found      = 0;

    for (uint32_t i = 0; i < view.cnt; ++i) {
        entity_handle_t   ent = view.ents[i];
        const collider2d *col = ecs_get_fast(w, ent, collider2d);
        if (!col->mask    ||  (mask && !(col->mask & mask))) continue;
        if  (col->trigger && !(flags & PHYSICS_RAY_TRIGGER)) continue;
        if (!col->trigger && !(flags & PHYSICS_RAY_SOLID))   continue;

        aabb3f box = _physics_lift_aabb2d(spatial_calc_aabb(ent, col));
        vec3f  norm;
        float  dist;
        if (_physics_ray_shape3d(vec3f(origin.x, origin.y, 0.0f), vec3f(dir.x, dir.y, 0.0f),
                                                    limit_dist, box, col->shape, &norm, &dist)) {
            physics_ray_hit2d_t h = {
                .ent        = ent,
                .point      = vec_add(origin, vec_scale(dir, dist)),
                .normal     = vec2f(norm.x, norm.y),
                .dist       = dist,
                .is_trigger = col->trigger,
            };

            if (found < max_out) found++;
            uint32_t pos = found - 1U;
            while (pos > 0 && out_hits[pos - 1U].dist > dist) {
                out_hits[pos] = out_hits[pos - 1U];
                pos--;
            }

            out_hits[pos] = h;
            if (found == max_out) limit_dist = out_hits[max_out - 1U].dist;
        }
    }

    if (world) ecs_set_world(prev);
    return found;
}

uint32_t physics_raycast3d(ecs_world_t *world, vec3f origin, vec3f dir, float max_dist, uint32_t mask,
                           uint32_t flags, physics_ray_hit3d_t *out_hits, uint32_t max_out) {
    ASSERT(out_hits,         "physics_raycast3d called with NULL out_hits");
    ASSERT(max_dist >= 0.0f, "physics_raycast3d called with negative max_dist (%.4f)", max_dist);
    if   (!out_hits || !max_out || max_dist < 0.0f) return 0;

    dir = vec_norm(dir);
    if (dir.x == 0 && dir.y == 0 && dir.z == 0) return 0;

    if (!flags) flags = PHYSICS_RAY_ALL;

    ecs_world_t *w = world ? world : ecs_get_world();
    if (!w) return 0;

    ecs_world_t *prev = ecs_get_world();
    if (world) ecs_set_world(world);

    ecs_view_t view       = ecs_query_comp(trans3d, collider3d);
    float      limit_dist = max_dist;
    uint32_t   found      = 0;

    for (uint32_t i = 0; i < view.cnt; ++i) {
        entity_handle_t   ent = view.ents[i];
        const collider3d *col = ecs_get_fast(w, ent, collider3d);
        if (!col->mask    ||  (mask && !(col->mask & mask))) continue;
        if  (col->trigger && !(flags & PHYSICS_RAY_TRIGGER)) continue;
        if (!col->trigger && !(flags & PHYSICS_RAY_SOLID))   continue;

        aabb3f box = spatial_calc_aabb(ent, col);
        vec3f  norm;
        float  dist;
        if (_physics_ray_shape3d(origin, dir, limit_dist, box, col->shape, &norm, &dist)) {
            physics_ray_hit3d_t h = {
                .ent        = ent,
                .point      = vec_add(origin, vec_scale(dir, dist)),
                .normal     = norm,
                .dist       = dist,
                .is_trigger = col->trigger,
            };

            if (found < max_out) found++;
            uint32_t pos = found - 1U;
            while (pos > 0 && out_hits[pos - 1U].dist > dist) {
                out_hits[pos] = out_hits[pos - 1U];
                pos--;
            }

            out_hits[pos] = h;
            if (found == max_out) limit_dist = out_hits[max_out - 1U].dist;
        }
    }

    if (world) ecs_set_world(prev);
    return found;
}
