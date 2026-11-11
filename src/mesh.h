#pragma once

/*
====================
mesh.h

3D mesh registry, ASCII Wavefront .obj ingestion,
and procedural 3D primitive generation.
Stores indexed triangle meshes (`mesh_vert_t` + `uint32_t` indices)
in permanent arena memory with automatic local AABB calculation.
====================
*/

#include "arena.h"
#include "core.h"
#include "ecs.h"
#include "gmath.h"
#include "spatial.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint16_t mesh_id_t;

/* ============================================================================================== */
/* >CONSTANTS                                                                                     */
/* ============================================================================================== */

#define MESH_DEF_MAX    256U
#define MESH_DEF_SLICES 16U
#define MESH_DEF_STACKS 12U

#define MESH_ID_CUBE       ((mesh_id_t)0U)
#define MESH_ID_USER_START ((mesh_id_t)1U)
#define MESH_ID_NULL       ((mesh_id_t)0xFFFFU)

/* ============================================================================================== */
/* >DATA TYPES                                                                                    */
/* ============================================================================================== */

/*
====================
mesh_vert_t

Public 32-byte 3D mesh vertex attribute layout.

Fields:
- `pos`:  local-space 3D vertex position <x, y, z> in world units
- `norm`: local-space unit surface normal <nx, ny, nz>
- `uv`:   normalized texture coordinates <u, v> in [0.0f, 1.0f]
====================
*/
typedef struct mesh_vert {
    vec3f pos;
    vec3f norm;
    vec2f uv;
} mesh_vert_t;

/*
====================
mesh_t

Public 3D triangle mesh descriptor stored in a `mesh_registry_t`.

Fields:
- `verts`:    contiguous vertex array allocated in perm arena memory
- `indices`:  contiguous triangle index array (multiple of 3) in perm arena memory
- `bounds`:   tight local-space AABB enclosing all `verts`
- `vert_cnt`: total number of vertices in `verts`
- `idx_cnt`:  total number of triangle indices in `indices` (tri_cnt = idx_cnt / 3)
- `id`:       unique mesh identifier corresponding to `mesh3d.mesh_id`
====================
*/
typedef struct mesh {
    mesh_vert_t *verts;
    uint32_t    *indices;
    aabb3f       bounds;
    uint32_t     vert_cnt;
    uint32_t     idx_cnt;
    mesh_id_t    id;
    uint8_t     _pad[6];
} mesh_t;

/*
====================
mesh_registry_t

Managed 3D mesh storage subsystem.
Allocates mesh metadata tables, vertex buffers, and
index buffers inside a caller-provided perm arena.
Permanently reserves `MESH_ID_CUBE` (`id = 0` 1x1x1 unit box)
as a safe fallback for uninitialized or invalid mesh handles.

Invariants:
- `id = 0` (`MESH_ID_CUBE`) is permanently reserved and generated during init.
- `base_arena_off` records the arena offset immediately after `MESH_ID_CUBE`
  so `mesh_registry_reset` can reclaim wihout clearing the default cube.

Lifecycle:
- Initialized via `mesh_registry_init`.
- Populated via `mesh_load_obj_*`, `mesh_gen_*`, or `mesh_register`.
- Reset via `mesh_registry_reset`.
====================
*/
typedef struct mesh_registry {
    arena_t *arena;  /* borrowed perm arena for vert/idx storage      */
    mesh_t  *meshes; /* sz: max_meshes * sizeof(mesh_t), perm-alloced */
    size_t   base_arena_off;
    uint32_t max_meshes;
    uint32_t mesh_cnt;
    size_t   tail_off;
} mesh_registry_t;

/* ============================================================================================== */
/* >LIFECYCLE                                                                                     */
/* ============================================================================================== */

/*
====================
mesh_registry_init

Initializes the 3D mesh registry in permanent arena `perm`.
Allocates the `meshes` descriptor table (defaults to `MESH_DEF_MAX`
if `max_meshes == 0`) and generates the built-in 1x1x1 unit cube
at `MESH_ID_CUBE` (`id = 0`).
Asserts non-null pointers in debug, returns error code in release.

Returns:
- `OK`     on successful initialization.
- `EINVAL` if `reg` or `perm` is `NULL`.
- `EOOM`   if `perm` lacks capacity for the descriptor table or unit cube.
====================
*/
res_t mesh_registry_init(mesh_registry_t *reg, arena_t *perm, uint32_t max_meshes);

/*
====================
mesh_registry_reset

Clears all user-registered meshes, restores `mesh_cnt = 1`,
and rewinds `reg->arena->off` to `base_arena_off` if no unrelated
allocations occured after it.
Asserts non-null `reg` in debug, returns error code in release.

Returns:
- `OK`     on successful reset.
- `EINVAL` if `reg` is `NULL` or uninitialized.
====================
*/
res_t mesh_registry_reset(mesh_registry_t *reg);

/*
====================
mesh_register

Allocates a new mesh slot in `reg`, copies `vert_cnt` vertices
and `idx_cnt` triangle indices into `reg->arena`, computes the
tight local AABB, and writes the new handle to `*out_id`.
`idx_cnt` must be non-zero and a multiple of 3.
Asserts valid inputs in debug, returns error code in release.

Returns:
- `OK`     on successful registration.
- `EINVAL` if pointers are `NULL`, `vert_cnt == 0`, `idx_cnt == 0`,
           `idx_cnt % 3 != 0`, or any index `>=vert_cnt`.
- `EFULL`  if `reg->mesh_cnt >= reg->max_meshes`.
- `EOOM`   if `reg->arena` lacks capacity for vertex or index buffers.
====================
*/
res_t mesh_register(mesh_registry_t *reg, const mesh_vert_t *verts, uint32_t vert_cnt,
                                    const uint32_t *indices, uint32_t idx_cnt, mesh_id_t *out_id);

/* ============================================================================================== */
/* >WAVEFRONT .OBJ                                                                                */
/* ============================================================================================== */

/*
====================
mesh_load_obj_mem

Parses an ASCII Wavefront `.obj` string `obj_str` of
byte length `len` into a newly registered mesh in `reg`.
Supports `v` (positions), `vt` (UVs), `vn` (normals),
and `f` (triangle, quad, and convex N-gon faces with
1-based positive or negative relative indices in `v`,
`v/vt`, `v//vn`, or `v/vt/vn` formats).
Automatically computes flat geometric normals for faces
that omit `vn`. Uses `scratch` for temporary parsing
buffers (unwound before return).
`scratch` must be a different arena from `reg->arena`.
Each face of the object must have no more than 64 vertices.
Asserts non-null pointers in debug,
returns error code in release.

Returns:
- `OK`      on successful parsing and registration.
- `EINVAL`  if `reg`, `obj_str`, `scratch` or `out_id` is `NULL`, or `len == 0`.
- `ENOTFND` if the `.obj` data contains no valid faces/vertices or has out-of-bounds indices.
- `EFULL`   if `reg->mesh_cnt >= reg->max_meshes`.
- `EOOM`    if `scratch` or `reg->arena` lacks memory.
====================
*/
res_t mesh_load_obj_mem(mesh_registry_t *reg, const char *obj_str, size_t len, arena_t *scratch, mesh_id_t *out_id);

/*
====================
mesh_load_obj_file

Reads an ASCII Wavefront `.obj` file from filesystem
`path` into `scratch` and parses it into `reg` via
`mesh_load_obj_mem` with zero heap allocations.
`scratch` must be a different arena from `reg->arena`.
Each face of the object must have no more than 64 vertices.
Asserts non-null pointers in debug,
returns error code in release.

- `OK`      on successful file read and mesh registration.
- `EINVAL`  if `reg`, `path`, `scratch`, or `out_id` is `NULL`.
- `ENOTFND` if `path` cannot be opened or contains no valid mesh geometry.
- `EFULL`   if `reg->mesh_cnt >= reg->max_meshes`.
- `EOOM`    if `scratch` or `reg->arena` lacks memory.
====================
*/
res_t mesh_load_obj_file(mesh_registry_t *reg, const char *path, arena_t *scratch, mesh_id_t *out_id);

/* ============================================================================================== */
/* >PROCEDURAL 3D PRIMITIVES                                                                      */
/* ============================================================================================== */

/*
====================
mesh_gen_box

Generates an axis-aligned 3D box mesh centered
at the origin with dimensions `size`
(<width, height, depth>, 24 vertices, 36 indices).
Asserts valid parameters in debug,
returns error code in release.

Returns:
- `OK`     on successful generation.
- `EINVAL` if `reg` or `out_id` is `NULL`, or any component of `size <= 0.0f`.
- `EFULL`  if `reg->mesh_cnt >= reg->max_meshes`.
- `EOOM`   if `reg->arena` lacks capacity.
====================
*/
res_t mesh_gen_box(mesh_registry_t *reg, vec3f size, mesh_id_t *out_id);

/*
====================
mesh_gen_plane

Generates a horizontal XZ plane mesh at Y=0
centered at the origin with dimensions `size`
(<width_x, depth_z>) and grid subdivisions
`subdiv_x` by `subdiv_z` (if 0, defaults to 1).
Outward normal is +Y.
Asserts valid parameters in debug,
returns error code in release.

Returns:
- `OK`     on successful generation.
- `EINVAL` if `reg` or `out_id` is `NULL`, or `size.x <= 0.0f || size.y <= 0.0f`.
- `EFULL`  if `reg->mesh_cnt >= reg->max_meshes`.
- `EOOM`   if `reg->arena` lacks capacity.
====================
*/
res_t mesh_gen_plane(mesh_registry_t *reg, vec2f size, uint32_t subdiv_x, uint32_t subdiv_z, mesh_id_t *out_id);

/*
====================
mesh_gen_sphere

Generates a UV sphere mesh centered at the
origin with `radius > 0.0f`, longitudal
`slices` (if 0, defaults to `MESH_DEF_SLICES` = 16, min 3),
and latitudal `stacks` (if 0, defaults to `MESH_DEF_STACKS`
= 12, min 2). Asserts valid parameters in debug,
returns error code in release.

Returns:
- `OK`     on successful generation.
- `EINVAL` if `reg` or `out_id` is `NULL`, or `radius <= 0.0f`.
- `EFULL`  if `reg->mesh_cnt >= reg->max_meshes`.
- `EOOM`   if `reg->arena` lacks capacity.
====================
*/
res_t mesh_gen_sphere(mesh_registry_t *reg, float radius, uint32_t slices, uint32_t stacks, mesh_id_t *out_id);

/*
====================
mesh_gen_cylinder

Generates an upright Y-axis cylinder mesh
centered at the origin (`y` in [-height/2, height/2])
with `radius > 0.0f`, `height > 0.0f`, and radical
`slices` (if 0, defaults to `MESH_DEF_SLICES` = 16, min 3),
including smooth side normals and flat top (+Y)
and bottom (-Y) disc caps.
Asserts valid parameters in debug,
returns error code in release.

Returns:
- `OK`     on successful generation.
- `EINVAL` if `reg` or `out_id` is `NULL`, or `radius <= 0.0f || height <= 0.0f`.
- `EFULL`  if `reg->mesh_cnt >= reg->max_meshes`.
- `EOOM`   if `reg->arena` lacks capacity.
====================
*/
res_t mesh_gen_cylinder(mesh_registry_t *reg, float radius, float height, uint32_t slices, mesh_id_t *out_id);

/* ============================================================================================== */
/* >QUERIES                                                                                       */
/* ============================================================================================== */

/*
====================
mesh_get

Looks up the mesh descriptor for `id` in `reg`.
If `id` is out of bounds (`id >= reg->mesh_cnt`),
falls back to `MESH_ID_CUBE` (`id = 0`) so rendering
pipelines always receive valid geometry. Returns
`NULL` only if `reg` is `NULL` or uninitialized.

Returns:
- pointer to `mesh_t` on successful lookup.
- mesh under `MESH_ID_CUBE` if `id` was not found.
- `NULL` if `reg` is `NULL` or uninitialized.
====================
*/
const mesh_t *mesh_get(const mesh_registry_t *reg, mesh_id_t id);

/*
====================
mesh_get_strict

Looks up the mesh descriptor for `id` in `reg` without fallback.

Returns:
- pointer to `mesh_t` on successful lookup.
- `NULL` if `id` was not found, `reg` is `NULL` or is uninitialized.
====================
*/
static inline const mesh_t *mesh_get_strict(const mesh_registry_t *reg, mesh_id_t id) {
    if (!reg || !reg->meshes || id >= reg->mesh_cnt) return NULL;
    return &reg->meshes[id];
}

/*
====================
mesh_fit_collider3d

Constructs a `COLLIDER_BOX` `collider3d` component
tightly fitted to `mesh->bounds` (setting `half_x/y/z`
to half-extents and `off_x/y/z` to the local bounding
box center).

Returns:
- fitted `collider3d` component value on success.
- zeroed `collider3d` if `mesh` is `NULL`.
====================
*/
static inline collider3d mesh_fit_collider3d(const mesh_t *mesh, uint32_t mask, bool trigger) {
    if (!mesh) return (collider3d){ .mask = mask, .trigger = trigger, .shape = COLLIDER_BOX };

    vec3f half   = vec_scale(aabb_size(mesh->bounds), 0.5f);
    vec3f center = aabb_center(mesh->bounds);

    return collider3d(half, center, mask, trigger, COLLIDER_BOX);
}
