#pragma once

/*
====================
render.h

Backend-agnostic 2D/3D render batch extraction.
Converts ECS renderables into a flat, scratch-allocated
`render_batch_t` consumed identically by the sokol GPU
backend and the headless software rasterizer.

Pipeline:
- pass 1 (3D opaque): `(trans3d, mesh3d)` entities, sorted by
  `(mesh_id, tex_id)` key, depth test + depth write enabled.
  Evaluates up to 8 lights from `(trans3d, light3d)` entities.
  When no lights exist, pass 1 renders purely unlit.
- pass 2 (2D overlay): `(trans2d, sprite2d)` entities, stable
  1-pass 256-bucket sort by `layer`, alpha blended on top of the
  3D pass with depth test disabled and unlit shading.

Never touches GPU state or `asset_atlas_t.dirty`,
backends own texture upload and clip-space conventions.
====================
*/

#include "arena.h"
#include "assets.h"
#include "core.h"
#include "ecs.h"
#include "gmath.h"
#include "mesh.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================================================== */
/* >CONSTANTS                                                                                     */
/* ============================================================================================== */

#define RENDER_LAYER_CNT      256U
#define RENDER_DEF_ASPECT     (16.0f / 9.0f)
#define RENDER_DEF_FOV_RAD    (GMATH_PI / 3.0f)
#define RENDER_DEF_NEAR_Z     0.1f
#define RENDER_DEF_FAR_Z      1000.0f
#define RENDER_DEF_VIEWPORT_W 320.0f
#define RENDER_DEF_VIEWPORT_H 180.0f
#define RENDER_DEF_AMBIENT    0x404040FFU
#define RENDER_MAX_LIGHTS     8U

typedef enum render_flip_flags {
    RENDER_FLIP_NONE = 0,
    RENDER_FLIP_H    = BIT(0), /* mirror sprite UVs horizontally */
    RENDER_FLIP_V    = BIT(1), /* mirror sprite UVs vertically   */
} render_flip_flags_t;

/*
====================
light_type_t

Categorization of 3D light sources for pass 1 lighting evaluation.
Matches `light3d.type` in `components.def`.
====================
*/
typedef enum light_type {
    LIGHT_DIR   = 0,
    LIGHT_POINT = 1,
    LIGHT_SPOT  = 2,
} light_type_t;

/* ============================================================================================== */
/* >VIEW TYPES                                                                                    */
/* ============================================================================================== */

/*
====================
render_cam2d_t

Public 2D orthographic camera (Y-down screen space).

Fields:
- `pos`:      2D world camera position <x, y> in world units              (default: <0, 0>)
- `rot`:      camera rotation around Z in radians                         (default: 0.0f)
- `zoom`:     uniform magnification factor, >0.0f                         (default: 1.0f)
- `centered`: `true` places `pos` at viewport center, `false` at top-left (default: false)
====================
*/
typedef struct render_cam2d {
    vec2f    pos;
    float    rot;
    float    zoom;
    bool     centered;
    uint8_t _pad[3];
} render_cam2d_t;

/*
====================
render_cam3d_t

Public 3D right-handed perspective camera.

Fields:
- `pos`:     eye position in world space           (default: <0, 0, 0>)
- `target`:  look-at target point in world space   (default: <0, 0, -1>)
- `up`:      world up direction vector             (default: <0, 1, 0>)
- `fov_rad`: vertical field of view in radians     (default: PI/3 = 60 deg)
- `near_z`:  near clipping plane distance, >0.0f   (default: 0.1f)
- `far_z`:   far clipping plane distance,  >near_z (default: 1000.0f)
====================
*/
typedef struct render_cam3d {
    vec3f pos;
    vec3f target;
    vec3f up;
    float fov_rad;
    float near_z;
    float far_z;
} render_cam3d_t;

/*
====================
render_view_t

Public per-frame view description passed to `render_build_batch`.

Fields:
- `cam3d`:    3D perspective camera for pass 1
- `cam2d`:    2D orthographic camera for pass 2
- `ambient`:  packed RGBA `0xRRGGBBAA` ambient color (default: `0x404040FF`)
- `viewport`: target size in pixels <width, height>, both >0
====================
*/
typedef struct render_view {
    render_cam3d_t cam3d;
    render_cam2d_t cam2d;
    uint32_t       ambient;
    vec2i          viewport;
} render_view_t;

/*
====================
render_pass_state_t

Public fixed-function pipeline state for a render pass.

Fields:
- `depth_test`:  test fragments against depth buffer (`LESS_EQUAL`)
- `depth_write`: write fragment depth into depth buffer
- `cull_back`:   cull back faces (front faces are counter-clockwise)
- `alpha_blend`: `SRC_ALPHA / ONE_MINUS_SRC_ALPHA` blending
====================
*/
typedef struct render_pass_state {
    bool depth_test;
    bool depth_write;
    bool cull_back;
    bool alpha_blend;
} render_pass_state_t;

#define RENDER_PASS3D_STATE ((render_pass_state_t){ .depth_test = true,  .depth_write = true,  .cull_back = true,  .alpha_blend = false })
#define RENDER_PASS2D_STATE ((render_pass_state_t){ .depth_test = false, .depth_write = false, .cull_back = false, .alpha_blend = true  })

/* ============================================================================================== */
/* >BATCH TYPES                                                                                   */
/* ============================================================================================== */

/*
====================
render_cmd3d_t

Public pass 1 opaque mesh draw command.
Backends compute clip position as `view_proj3d * model * pos`.

Fields:
- `model`:    local-to-world matrix
- `norm_mat`: inverse-transpose of `model` upper 3x3 for world-space normals
- `uv_rect`:  atlas sub-rectangle [u0, v0, u1, v1], remap mesh UVs via `render_remap_uv`
- `mesh`:     resolved mesh (never NULL, invalid IDs fall back to `MESH_ID_CUBE`)
- `ent`:      source entity handle
- `sort_key`: `(mesh_id << 16) | tex_id`, batch is sorted ascending by this key
- `color`:    packed RGBA tint `0xRRGGBBAA`
====================
*/
typedef struct render_cmd3d {
    mat4f           model;
    mat3f           norm_mat;
    aabb2f          uv_rect;
    const mesh_t   *mesh;
    entity_handle_t ent;
    uint32_t        sort_key;
    uint32_t        color;
} render_cmd3d_t;

/*
====================
render_sprite_inst_t

Public pass 2 per-instance record, laid out for direct
upload as a `SG_VERTEXSTEP_PER_INSTANCE` vertex buffer.

Fields:
- `pos`:   anchor position in 2D world units
- `size`:  quad size in world units (atlas pixel size * `trans2d.scale`)
- `pivot`: normalized anchor within the quad, [0, 1] (<0, 0> = `PIVOT_TOP_LEFT`)
- `uv`:    atlas UV rectangle [u0, v0, u1, v1], already H/V flipped
- `rot`:   rotation around the anchor in radians
- `color`: packed RGBA tint `0xRRGGBBAA`
====================
*/
typedef struct render_sprite_tint {
    vec2f    pos;
    vec2f    size;
    vec2f    pivot;
    aabb2f   uv;
    float    rot;
    uint32_t color;
} render_sprite_inst_t;

_Static_assert(sizeof(render_sprite_inst_t) == 48, "render_sprite_inst_t must stay 48 bytes for GPU upload");

/*
====================
render_light_gpu_t

Uniform representation of a single 3D light.
Packed into 64 bytes for direct uniform buffer binding.

Fields:
- `pos_range`: world-space position <x, y, z>, radius/range in `w` (0.0f for directional)
- `dir_type`:  world-space direction light travels <x, y, z>, type in `w` (0=dir, 1=point, 2=spot)
- `color`:     linear diffuse RGB <r, g, b> (color * intensity), unused `w`
- `cone`:      cos(cone_in) in `x`, cos(cone_out) in `y`, unused <z, w>
====================
*/
typedef struct render_light_gpu {
    vec4f    pos_range;
    vec4f    dir_type;
    vec4f    color;
    vec4f    cone;
} render_light_gpu_t;

_Static_assert(sizeof(render_light_gpu_t) == 64, "render_light_gpu_t must be 64 bytes for std140 uniform alignment");

/*
====================
render_batch_t

Per-frame render submission produced by `render_build_batch`.
All arrays are borrowed from `world->scratch` and remain valid
until the next scratch reset.

Invariants:
- `cmds3d`         is sorted ascending by `sort_key` (stable).
- `sprites2d`      is sorted ascending by `sprite2d.ayer` (stable, back-to-front).
- `sprite_ents[i]` is the source entity of `sprites2d[i]`.
- `lights`         contains up to `RENDER_MAX_LIGHTS` active lights, prioritized
                   by directional first, then point/spot by camera proximity.
- `light_cnt`      is the number of valid lights in `lights`. When 0, pass 1
                   renders strictly unlit with zero lighting overhead.
- `view_proj3d`    uses OpenGL clip depth [-1, 1], backends with [0, 1] depth
                   must apply `render_proj_to_z01` once per frame.
- Pass state is fixed: pass 1 = `RENDER_PASS3D_STATE`, pass 2 = `RENDER_PASS2D_STATE`.
====================
*/
typedef struct render_batch {
    render_cmd3d_t       *cmds3d;      /* sz: cmd3d_cnt,    scratch-alloced */
    render_sprite_inst_t *sprites2d;   /* sz: sprite2d_cnt, scratch-alloced */
    entity_handle_t      *sprite_ents; /* sz: sprite2d_cnt, scratch-alloced */
    mat4f                 view_proj3d;
    mat4f                 view_proj2d;
    render_light_gpu_t    lights[RENDER_MAX_LIGHTS];
    uint32_t              light_cnt;
    uint32_t              ambient;
    vec2i                 viewport;
    uint32_t              cmd3d_cnt;
    uint32_t              sprite2d_cnt;
} render_batch_t;

/* ============================================================================================== */
/* >DEFAULTS                                                                                      */
/* ============================================================================================== */

static inline render_cam2d_t render_cam2d_def(void) {
    return (render_cam2d_t){ .pos = vec2f_splat(0), .rot = 0.0f, .zoom = 1.0f, .centered = false };
}

static inline render_cam3d_t render_cam3d_def(void) {
    return (render_cam3d_t){
        .pos     = vec3f_splat(0),
        .target  = vec3f_splat(0),
        .up      = vec3f(0, 1, 0),
        .fov_rad = RENDER_DEF_FOV_RAD,
        .near_z  = RENDER_DEF_NEAR_Z,
        .far_z   = RENDER_DEF_FAR_Z,
    };
}

/*
====================
render_view_def

Returns a view with default 3D/2D cameras and default ambient for `viewport`.
====================
*/
static inline render_view_t render_view_def(vec2i viewport) {
    return (render_view_t){
        .cam3d    = render_cam3d_def(),
        .cam2d    = render_cam2d_def(),
        .ambient  = RENDER_DEF_AMBIENT,
        .viewport = viewport,
    };
}

/* ============================================================================================== */
/* >MATRIX HELPERS                                                                                */
/* ============================================================================================== */

/*
====================
render_mat4f_lookat

Builds a right-handed view matrix looking from `eye` toward `target`.
Degenerate inputs (`eye == target`, `up` parallel to view direction)
fall back to a stable orthonormal basis instead of producing NaNs.

Returns the 4x4 view matrix.
====================
*/
mat4f render_mat4f_lookat(vec3f eye, vec3f target, vec3f up);

/*
====================
render_cam3d_vp

Computes `proj * view` for `cam` and `viewport` (OpenGL clip depth [-1, 1]).
`NULL` `cam` uses `render_cam3d_def()`, invalid fov/near/far and non-positive
viewport fallback to defaults.

Returns the 4x4 3D view-projection matrix.
====================
*/
mat4f render_cam3d_vp(const render_cam3d_t *cam, vec2i viewport);

/*
====================
render_cam2d_vp

Computes the Y-down orthographic `proj * view` for `cam` and `viewport`.
Rotation and zoom apply around `pos` (viewport center if `centered`,
top-left otherwise). `NULL` `cam` uses `render_cam2d_def()`.
====================
*/
mat4f render_cam2d_vp(const render_cam2d_t *cam, vec2i viewport);

/*
====================
render_trans3d_mat4

Builds a local-to-world matrix from a resolved `trans3d` (translation,
quaternion orientation, per-axis scale). A zeroed quaternion is treated
as identity. `NULL` `wt` returns identity.

Returns the 4x4 model matrix.
====================
*/
mat4f render_trans3d_mat4(const trans3d *wt);

/*
====================
render_proj_to_z01

Remaps a projection with OpenGL clip depth [-1, 1] to [0, 1]
(`z = z / 2 + w / 2`) for D3D11, Metal and WebGPU backends.

Returns the remapped 4x4 matrix.
====================
*/
static inline mat4f render_proj_to_z01(mat4f proj) {
    return mat4f_mul(mat4f(
        vec4f(1, 0, 0,    0),
        vec4f(0, 1, 0,    0),
        vec4f(0, 0, 0.5f, 0),
        vec4f(0, 0, 0.5f, 1)
    ), proj);
}

/*
====================
render_remap_uv

Maps a mesh UV to [0, 1] into atlas sub-rectangle `uv_rect`.
Input is calmped to [0, 1], atlas sub-rectangles cannot tile.

Returns the atlas UV coordinate.
====================
*/
static inline vec2f render_remap_uv(vec2f uv, aabb2f uv_rect) {
    uv = vec_clamp(uv, vec2f_splat(0), vec2f_splat(1));
    return vec_add(uv_rect.min, vec_mul(uv, aabb_size(uv_rect)));
}

/* ============================================================================================== */
/* >BATCH EXTRACTION                                                                              */
/* ============================================================================================== */

/*
====================
render_build_batch

Extracts all renderables in `world` into `out_batch` using `world->scratch`.
Pass 1 resolves hierarchical `trans3d`, model/normal matrices, meshes and
atlas UV rects for `(trans3d, mesh3d)`, then radix-sorts by `sort_key`.
Up to 8 lights from `(trans3d, light3d)` entities are extracted into
`out_batch->lights` (directional lights first, then point/spot lights
prioritized by proximity to `cam3d.pos`). If no lights exist,
`out_batch->light_cnt` is 0 (unlit).
Pass 2 resolves hierarchical `trans3d`, atlas size/pivot/UVs (invalid
sprite IDs fallback to `ASSET_SPRITE_CHECKER`) and applies H/V flips for
`(trans2d, sprite2d)`, stable-sorted by `layer`.
`atlas` may be `NULL` only if no sprites exist, `meshes` only if no meshes exist.
If `world` is `NULL`, uses `ecs_get_world()`. On any error `*out_batch` is zeroed.

Returns:
- `OK`      on success (empty batch if nothing is renderable).
- `EINVAL`  if `view` or `out_batch` is `NULL`, viewport is non-positive,
            `world->scratch` is `NULL`, or a requred `atlas`/`meshes` is `NULL`.
- `ENOTFND` if `world` is `NULL` and no active ECS world is bound.
- `EOOM`    if `world->scratch` lacks capacity for batch arrays or sort buffers.
====================
*/
res_t render_build_batch(ecs_world_t *world, const asset_atlas_t *atlas, const mesh_registry_t *meshes,
                         const render_view_t *view, render_batch_t *out_batch);

/*
====================
render_sprite_quad_verts

Writes the 4 world-space corners of `inst` (0: top-left, 1: top-right,
2: bottom-right, 3: bottom-left, before rotation) accounting for pivot,
size and rotation. Used by the software rasterizer and CPU-side picking.
Asserts non-null pointers in debug.
====================
*/
void render_sprite_quad_verts(const render_sprite_inst_t *inst, vec2f out_corners[4]);
