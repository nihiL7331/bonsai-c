#include "backend/cpu.h"

#include <math.h>
#include <string.h>

/* ============================================================================================== */
/* >CONSTANTS AND TYPES                                                                           */
/* ============================================================================================== */

#define BACKEND_CPU_TRI_VERT_CNT     3U
#define BACKEND_CPU_QUAD_VERT_CNT    4U
#define BACKEND_CPU_PX_CENTER_OFF    0.5
#define BACKEND_CPU_CHAN_ROUND_BIAS  0.5f
#define BACKEND_CPU_TEXEL_ROUND_BIAS 0.5
#define BACKEND_CPU_DEPTH_CLEAR      1.0f

typedef enum _backend_cpu_clip_plane {
    BACKEND_CPU_CLIP_LEFT,
    BACKEND_CPU_CLIP_RIGHT,
    BACKEND_CPU_CLIP_BOTTOM,
    BACKEND_CPU_CLIP_TOP,
    BACKEND_CPU_CLIP_NEAR,
    BACKEND_CPU_CLIP_FAR,
    BACKEND_CPU_CLIP_PLANE_CNT,
} _backend_cpu_clip_plane_t;

/* clipping a convex polygon against one plane adds at most one vertex */
#define BACKEND_CPU_CLIP_VERT_CAP (BACKEND_CPU_TRI_VERT_CNT + BACKEND_CPU_CLIP_PLANE_CNT)

typedef struct _backend_cpu_vert {
    vec4f clip;
    vec2f uv;
    vec3f pos;
    vec3f norm;
} _backend_cpu_vert_t;

/* ============================================================================================== */
/* >COLOR AND GEOMETRY HELPERS                                                                    */
/* ============================================================================================== */

static inline vec3f _backend_cpu_vec4f_to_vec3f(vec4f v) {
    return vec3f(v.x, v.y, v.z);
}

/* components retain the channel range [0, COLOR_CHAN_MAX], not normalized */
static inline vec4f _backend_cpu_color_to_vec4f(uint32_t c) {
    return vec4f(COLOR_R(c), COLOR_G(c), COLOR_B(c), COLOR_A(c));
}

static inline uint8_t _backend_cpu_chan_to_u8(float channel) {
    return (uint8_t){ CLAMP(channel, (float)COLOR_CHAN_MIN, (float)COLOR_CHAN_MAX) + BACKEND_CPU_CHAN_ROUND_BIAS };
}

static inline uint32_t _backend_cpu_color_to_u32(vec4f c) {
    return COLOR_RGBA(
        _backend_cpu_chan_to_u8(c.x),
        _backend_cpu_chan_to_u8(c.y),
        _backend_cpu_chan_to_u8(c.z),
        _backend_cpu_chan_to_u8(c.w)
    );
}

/* returns twice the signed area of triangle (a, b, [x, y]).                */
/* its sign determines which side of the directed edge contains the sample. */
static inline double _backend_cpu_signed_double_area(vec3f a, vec3f b, double x, double y) {
    return ((double)b.x - a.x) * (y - a.y) - ((double)b.y - a.y) * (x - a.x);
}

static inline bool _backend_cpu_is_top_left_edge(vec3f a, vec3f b) {
    return b.y < a.y || (b.y == a.y && b.x > a.x);
}

/* returns the homogeneous half-space value, nonnegative means inside */
static inline double _backend_cpu_clip_plane_dist(vec4f p, uint32_t plane) {
    switch (plane) {
        case BACKEND_CPU_CLIP_LEFT:   return (double)p.w + p.x;
        case BACKEND_CPU_CLIP_RIGHT:  return (double)p.w - p.x;
        case BACKEND_CPU_CLIP_BOTTOM: return (double)p.w + p.y;
        case BACKEND_CPU_CLIP_TOP:    return (double)p.w - p.y;
        case BACKEND_CPU_CLIP_NEAR:   return (double)p.w + p.z;
        case BACKEND_CPU_CLIP_FAR:    return (double)p.w - p.z;
        default:                      ASSERT(false, "Invalid CPU clip plane"); return 0;
    }
}

static inline _backend_cpu_vert_t _backend_cpu_lerp_vert(_backend_cpu_vert_t a, _backend_cpu_vert_t b, float t) {
    return (_backend_cpu_vert_t){
        .clip = vec_add(vec_scale(a.clip, 1 - t), vec_scale(b.clip, t)),
        .uv   = vec_add(vec_scale(a.uv,   1 - t), vec_scale(b.uv,   t)),
        .pos  = vec_add(vec_scale(a.pos,  1 - t), vec_scale(b.pos,  t)),
        .norm = vec_add(vec_scale(a.norm, 1 - t), vec_scale(b.norm, t)),
    };
}

/* clips successively against each homogeneous plane and returns the resulting polygon size */
static uint32_t _backend_cpu_clip_tri(const _backend_cpu_vert_t in[BACKEND_CPU_TRI_VERT_CNT], _backend_cpu_vert_t out[BACKEND_CPU_CLIP_VERT_CAP]) {
    _backend_cpu_vert_t polygon[2][BACKEND_CPU_CLIP_VERT_CAP];
    memcpy(polygon[0], in, BACKEND_CPU_TRI_VERT_CNT * sizeof(*in));

    for (uint32_t i = 0; i < BACKEND_CPU_TRI_VERT_CNT; ++i) {
        vec4f clip   = in[i].clip;
        bool  finite = isfinite(clip.x) && isfinite(clip.y) && isfinite(clip.z) && isfinite(clip.w);
        ASSERT(finite, "Non-finite projected vertex");
        if   (!finite) return 0;
    }

    uint32_t cnt = BACKEND_CPU_TRI_VERT_CNT, src = 0;

    for (uint32_t plane = BACKEND_CPU_CLIP_LEFT; plane < BACKEND_CPU_CLIP_PLANE_CNT && cnt; ++plane) {
        uint32_t        next_cnt = 0;
        _backend_cpu_vert_t a    = polygon[src][cnt - 1];
        double          a_dist   = _backend_cpu_clip_plane_dist(a.clip, (_backend_cpu_clip_plane_t)plane);

        for (uint32_t i = 0; i < cnt; ++i) {
            _backend_cpu_vert_t b  = polygon[src][i];
            double          b_dist = _backend_cpu_clip_plane_dist(b.clip, (_backend_cpu_clip_plane_t)plane);

            if ((a_dist >= 0) != (b_dist >= 0)) {
                ASSERT(next_cnt <  BACKEND_CPU_CLIP_VERT_CAP, "Clipped polygon overflow");
                if    (next_cnt >= BACKEND_CPU_CLIP_VERT_CAP) return 0;

                float t                      = (float)(a_dist / (a_dist - b_dist));
                polygon[1 - src][next_cnt++] = _backend_cpu_lerp_vert(a, b, t);
            }

            if (b_dist >= 0) {
                ASSERT(next_cnt  < BACKEND_CPU_CLIP_VERT_CAP, "Clipped polygon overflow");
                if    (next_cnt >= BACKEND_CPU_CLIP_VERT_CAP) return 0;

                polygon[1 - src][next_cnt++] = b;
            }

            a      = b;
            a_dist = b_dist;
        }

        cnt = next_cnt;
        src = 1 - src;
    }

    memcpy(out, polygon[src], cnt * sizeof(*out));
    return cnt;
}

/* samples the nearest texel, clamped inside the rect even when its UV axes are flipped */
static uint32_t _backend_cpu_sample_atlas(const asset_atlas_t *atlas, vec2f uv, aabb2f rect) {
    int32_t min_x = (int32_t)floor((double)MIN(rect.min.x, rect.max.x) * atlas->size.x + BACKEND_CPU_TEXEL_ROUND_BIAS);
    int32_t min_y = (int32_t)floor((double)MIN(rect.min.y, rect.max.y) * atlas->size.y + BACKEND_CPU_TEXEL_ROUND_BIAS);
    int32_t max_x = (int32_t)floor((double)MAX(rect.min.x, rect.max.x) * atlas->size.x + BACKEND_CPU_TEXEL_ROUND_BIAS) - 1;
    int32_t max_y = (int32_t)floor((double)MAX(rect.min.y, rect.max.y) * atlas->size.y + BACKEND_CPU_TEXEL_ROUND_BIAS) - 1;

    min_x = CLAMP(min_x, 0,     atlas->size.x - 1);
    min_y = CLAMP(min_y, 0,     atlas->size.y - 1);
    max_x = CLAMP(max_x, min_x, atlas->size.x - 1);
    max_y = CLAMP(max_y, min_y, atlas->size.y - 1);

    int32_t x = (int32_t)MIN(atlas->size.x - 1.0, floor((double)CLAMP(uv.x, 0.0f, 1.0f) * atlas->size.x));
    int32_t y = (int32_t)MIN(atlas->size.y - 1.0, floor((double)CLAMP(uv.y, 0.0f, 1.0f) * atlas->size.y));

    x = CLAMP(x, min_x, max_x);
    y = CLAMP(y, min_y, max_y);

    return atlas->pixels[IDX_2D(x, y, atlas->size.x)];
}

/* evaluates ambient and Lambert diffuse RGB using world-space position and normal */
static vec3f _backend_cpu_eval_diffuse_light(const render_batch_t *batch, vec3f pos, vec3f norm) {
    if (!batch->light_cnt) return vec3f_splat(1);

    norm      = vec_norm(norm);
    vec3f rgb = vec_scale(_backend_cpu_vec4f_to_vec3f(
        _backend_cpu_color_to_vec4f(batch->ambient)),
        1.0f / COLOR_CHAN_MAX
    );

    for (uint32_t i = 0; i < batch->light_cnt; ++i) {
        const render_light_gpu_t *light       = &batch->lights[i];
        vec3f                     dir         = vec_scale(_backend_cpu_vec4f_to_vec3f(light->dir_type), -1);
        float                     attenuation = 1;

        if (light->dir_type.w != LIGHT_DIR) {
            vec3f off  = vec_sub(_backend_cpu_vec4f_to_vec3f(light->pos_range), pos);
            float dist = vec_len(off);
            if (light->pos_range.w <= 0 || dist >= light->pos_range.w) continue;

            dir          = vec_norm(off);
            attenuation  = 1 - dist / light->pos_range.w;
            attenuation *= attenuation;

            if (light->dir_type.w == LIGHT_SPOT) {
                float cosine     = vec_dot(vec_scale(dir, -1), vec_norm(_backend_cpu_vec4f_to_vec3f(light->dir_type)));
                float cone_width = light->cone.x - light->cone.y;
                float t          = cone_width > 0 ? CLAMP((cosine - light->cone.y) / cone_width, 0.0f, 1.0f)
                                                  : (cosine >= light->cone.x ? 1.0f : 0.0f);
                attenuation     *= t * t * (3 - 2 * t); /* cubic smoothstep */
            }
        }

        float diffuse = MAX(0, vec_dot(norm, vec_norm(dir))) * attenuation;
        rgb           = vec_add(rgb, vec_scale(_backend_cpu_vec4f_to_vec3f(light->color), diffuse));
    }

    return rgb;
}

/* ============================================================================================== */
/* >RASTERIZATION                                                                                 */
/* ============================================================================================== */

/* rasterizes a clipped triangle at pixel centers with exclusive shared-edge ownership */
static void _backend_cpu_rasterize_tri(const render_batch_t *batch, const asset_atlas_t *atlas, const backend_cpu_target_t *target,
                                        _backend_cpu_vert_t verts[3], aabb2f rect, uint32_t tint, bool mesh) {
    vec3f screen[BACKEND_CPU_TRI_VERT_CNT];
    float inv_w[BACKEND_CPU_TRI_VERT_CNT];

    for (uint32_t i = 0; i < BACKEND_CPU_TRI_VERT_CNT; ++i) {
        if (verts[i].clip.w <= 0) return;
        inv_w[i] = 1 / verts[i].clip.w;

        /* map NDC [-1, 1] to vp pixels, NDC +Y points toward the top */
        screen[i] = vec3f(
             (verts[i].clip.x * inv_w[i] * 0.5f + 0.5f) * target->size.x,
            -(verts[i].clip.y * inv_w[i] * 0.5f - 0.5f) * target->size.y,
              verts[i].clip.z * inv_w[i]
        );
    }

    double area = _backend_cpu_signed_double_area(screen[0], screen[1], screen[2].x, screen[2].y);

    /* CCW front face becomes CW after reversing screen Y */
    if (!isfinite(area) || !area || (mesh && area >= 0)) return;

    if (area < 0) {
        SWAP(screen[1], screen[2]);
        SWAP(inv_w[1],  inv_w[2]);
        SWAP(verts[1],  verts[2]);
        area = -area;
    }

    /* clamp in double precision before converting bounds to pixel indices */
    double right = target->size.x - 1.0, bottom = target->size.y - 1.0;

    int32_t min_x = (int32_t)CLAMP(floor(MIN(screen[0].x, screen[1].x, screen[2].x)),     0.0, right);
    int32_t min_y = (int32_t)CLAMP(floor(MIN(screen[0].y ,screen[1].y, screen[2].y)),     0.0, bottom);
    int32_t max_x = (int32_t)CLAMP(ceil( MAX(screen[0].x, screen[1].x, screen[2].x)) - 1, 0.0, right);
    int32_t max_y = (int32_t)CLAMP(ceil( MAX(screen[0].y, screen[1].y, screen[2].y)) - 1, 0.0, bottom);

    bool top_left[BACKEND_CPU_TRI_VERT_CNT] = {
        _backend_cpu_is_top_left_edge(screen[1], screen[2]),
        _backend_cpu_is_top_left_edge(screen[2], screen[0]),
        _backend_cpu_is_top_left_edge(screen[0], screen[1]),
    };

    for (int32_t y = min_y; y <= max_y; ++y)
        for (int32_t x = min_x; x <= max_x; ++x) {
            double sample_x = x + BACKEND_CPU_PX_CENTER_OFF, sample_y = y + BACKEND_CPU_PX_CENTER_OFF;
            double edge[BACKEND_CPU_TRI_VERT_CNT] = {
                _backend_cpu_signed_double_area(screen[1], screen[2], sample_x, sample_y),
                _backend_cpu_signed_double_area(screen[2], screen[0], sample_x, sample_y),
                _backend_cpu_signed_double_area(screen[0], screen[1], sample_x, sample_y),
            };

            if (edge[0] < 0 || (!edge[0] && !top_left[0]) ||
                edge[1] < 0 || (!edge[1] && !top_left[1]) ||
                edge[2] < 0 || (!edge[2] && !top_left[2])) continue;

            float barycentric[BACKEND_CPU_TRI_VERT_CNT] = {
                (float)(edge[0] / area),
                (float)(edge[1] / area),
                (float)(edge[2] / area),
            };

            /* NDC depth is screen-linear, convert [-1, 1] to depth [0, 1] */
            float depth = CLAMP(
                (barycentric[0] * screen[0].z +
                 barycentric[1] * screen[1].z +
                 barycentric[2] * screen[2].z) * 0.5f + 0.5f,
                0.0f,
                1.0f
            );

            if (mesh && depth > target->depth[IDX_2D(x, y, target->depth_stride_pixels)]) continue;

            /* attributes use normalized barycentric/clip-w weights for perspective correction */
            float weight[BACKEND_CPU_TRI_VERT_CNT];
            float weight_sum = 0;

            for (uint32_t i = 0; i < BACKEND_CPU_TRI_VERT_CNT; ++i) {
                weight[i]   = barycentric[i] * inv_w[i];
                weight_sum += weight[i];
            }

            if (weight_sum <= 0 || !isfinite(weight_sum)) continue;

            vec2f uv  = vec2f_splat(0);
            vec3f pos = vec3f_splat(0), norm = vec3f_splat(0);

            for (uint32_t i = 0; i < BACKEND_CPU_TRI_VERT_CNT; ++i) {
                weight[i] /= weight_sum;
                uv         = vec_add(uv, vec_scale(verts[i].uv, weight[i]));

                if (!mesh) continue;

                pos  = vec_add(pos,  vec_scale(verts[i].pos,  weight[i]));
                norm = vec_add(norm, vec_scale(verts[i].norm, weight[i]));
            }

            vec4f color = vec_scale(vec_mul(
                _backend_cpu_color_to_vec4f(_backend_cpu_sample_atlas(atlas, uv, rect)),
                _backend_cpu_color_to_vec4f(tint)
            ), 1.0f / COLOR_CHAN_MAX);

            if (mesh) {
                vec3f light = _backend_cpu_eval_diffuse_light(batch, pos, norm);
                color.x *= light.x; color.y *= light.y; color.z *= light.z;
                target->depth[IDX_2D(x, y, target->depth_stride_pixels)] = depth;
            } else {
                float alpha = color.w / COLOR_CHAN_MAX;
                vec4f dest  = _backend_cpu_color_to_vec4f(target->pixels[IDX_2D(x, y, target->stride_pixels)]);

                color   = vec_add(vec_scale(color, alpha), vec_scale(dest, 1 - alpha));
                color.w = alpha * COLOR_CHAN_MAX + dest.w * (1 - alpha);
            }

            target->pixels[IDX_2D(x, y, target->stride_pixels)] = _backend_cpu_color_to_u32(color);
        }
}

/* clips a triangle, then rasterizes the surviving convex polygon as a triangle fan */
static void _backend_cpu_draw_clipped_tri(const render_batch_t *batch, const asset_atlas_t *atlas, const backend_cpu_target_t *target,
                                           const _backend_cpu_vert_t tri[3], aabb2f rect, uint32_t tint, bool mesh) {
    _backend_cpu_vert_t polygon[BACKEND_CPU_CLIP_VERT_CAP];
    uint32_t        cnt = _backend_cpu_clip_tri(tri, polygon);

    for (uint32_t i = 1; i + 1 < cnt; ++i) {
        _backend_cpu_vert_t fan[BACKEND_CPU_TRI_VERT_CNT] = { polygon[0], polygon[i], polygon[i + 1] };
        _backend_cpu_rasterize_tri(batch, atlas, target, fan, rect, tint, mesh);
    }
}

res_t backend_cpu_render(const render_batch_t *batch, const asset_atlas_t *atlas,
                         const backend_cpu_target_t *target, uint32_t clear) {
    if (!batch || !backend_cpu_target_is_valid(target, batch->cmd3d_cnt != 0) ||
        target->size.x != batch->viewport.x || target->size.y != batch->viewport.y ||
        batch->light_cnt > RENDER_MAX_LIGHTS ||
        (batch->cmd3d_cnt && !batch->cmds3d) || (batch->sprite2d_cnt && !batch->sprites2d)) return EINVAL;

    if ((batch->cmd3d_cnt || batch->sprite2d_cnt) &&
        (!atlas || !atlas->pixels || atlas->size.x <= 0 || atlas->size.y <= 0)) return EINVAL;

    for (int32_t y = 0; y < target->size.y; ++y)
        for (int32_t x = 0; x < target->size.x; ++x) {
            target->pixels[IDX_2D(x, y, target->stride_pixels)] = clear;
            if (target->depth)
                target->depth[IDX_2D(x, y, target->depth_stride_pixels)] = BACKEND_CPU_DEPTH_CLEAR;
        }

    for (uint32_t c = 0; c < batch->cmd3d_cnt; ++c) {
        const render_cmd3d_t *cmd  = &batch->cmds3d[c];
        const mesh_t         *mesh = cmd->mesh;

        if (!mesh || !mesh->verts || !mesh->indices || mesh->idx_cnt % BACKEND_CPU_TRI_VERT_CNT) return EINVAL;

        for (uint32_t i = 0; i < mesh->idx_cnt; i += BACKEND_CPU_TRI_VERT_CNT) {
            _backend_cpu_vert_t tri[BACKEND_CPU_TRI_VERT_CNT];

            for (uint32_t j = 0; j < BACKEND_CPU_TRI_VERT_CNT; ++j) {
                uint32_t vert = mesh->indices[i + j];
                ASSERT(vert  < mesh->vert_cnt, "CPU mesh index out of range");
                if    (vert >= mesh->vert_cnt) return EINVAL;

                const mesh_vert_t *v    = &mesh->verts[vert];
                vec4f              pos  = mat4f_mul_vec4f(cmd->model, vec4f(v->pos.x, v->pos.y, v->pos.z, 1));
                vec3f              norm = vec_add(
                    vec_scale(cmd->norm_mat.col[0], v->norm.x),
                    vec_scale(cmd->norm_mat.col[1], v->norm.y),
                    vec_scale(cmd->norm_mat.col[2], v->norm.z)
                );

                tri[j] = (_backend_cpu_vert_t){
                    .clip = mat4f_mul_vec4f(batch->view_proj3d, pos),
                    .uv   = render_remap_uv(v->uv, cmd->uv_rect),
                    .pos  = _backend_cpu_vec4f_to_vec3f(pos),
                    .norm = norm,
                };
            }

            _backend_cpu_draw_clipped_tri(batch, atlas, target, tri, cmd->uv_rect, cmd->color, true);
        }
    }

    for (uint32_t i = 0; i < batch->sprite2d_cnt; ++i) {
        const render_sprite_inst_t *sprite = &batch->sprites2d[i];
        vec2f                       corners[BACKEND_CPU_QUAD_VERT_CNT];
        render_sprite_quad_verts(sprite, corners);

        _backend_cpu_vert_t verts[BACKEND_CPU_QUAD_VERT_CNT] = {0};

        for (uint32_t j = 0; j < BACKEND_CPU_QUAD_VERT_CNT; ++j) {
            verts[j].clip = mat4f_mul_vec4f(batch->view_proj2d, vec4f(corners[j].x, corners[j].y, 0, 1));
            verts[j].uv = vec2f(
                (j == 1 || j == 2) ? sprite->uv.max.x : sprite->uv.min.x,
                 j >= 2            ? sprite->uv.max.y : sprite->uv.min.y
            );
        }

        /* quad corner order is TL, TR, BR, BL, both tris share TL->BR */
        _backend_cpu_vert_t fst[BACKEND_CPU_TRI_VERT_CNT] = { verts[0], verts[1], verts[2] };
        _backend_cpu_vert_t snd[BACKEND_CPU_TRI_VERT_CNT] = { verts[0], verts[2], verts[3] };
        _backend_cpu_draw_clipped_tri(batch, atlas, target, fst, sprite->uv, sprite->color, false);
        _backend_cpu_draw_clipped_tri(batch, atlas, target, snd, sprite->uv, sprite->color, false);
    }

    return OK;
}
