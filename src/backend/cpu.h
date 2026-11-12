#pragma once

/*
====================
cpu.h

CPU rendering target definitions.

Targets borrow writable color and optional depth storage
from the caller. Declaring a target does not allocate,
clear or take ownership of either buffer.
====================
*/

#include "image.h"
#include "render.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================================================== */
/* >TYPES                                                                                         */
/* ============================================================================================== */

/*
====================
backend_cpu_target_t

Borrowed writable target for single-sample CPU rendering.
Color and depth share dimensions and a top-left origin.
X increases right and Y increases down.

Fields:
- `pixels`:                first writable numeric 0xRRGGBBAA color.
- `depth`:                 first writable float depth value, or `NULL`.
- `size`:                  visible width and height in pixels, both positive.
- `stride_pixels`:         distance between color row starts in uint32_t elements.
- `capacity_pixels`:       accessible uint32_t elements starting at pixels.
- `depth_stride_pixels`:   distance between depth row starts in float elements.
- `depth_capacity_pixels`: accessible float elements starting at depth.

Each stored depth value corresponds to one pixel.
Color and depth strides may differ, each present buffers
stride must be at least `size.x`. Capacities include padding
between rows. Padding after the final row is not required.

Depth values are normalized [0, 1], with 0 nearest and
1 farthest. Clearing depth to 1 prepares it for the built-in
LESS_EQUAL comparison.

When `depth` is `NULL`, `depth_stride_pixels` and
`depth_capacity_pixels` must both be zero. Rendering that
requires depth must reject a target without depth storage.

Color and depth storage must not overlap each other or
input storage read by the render operation.

The caller keeps both buffers alive and exclusively available
to the renderer until the CPU render call returns. Afterward,
the caller may read or modify them. Later consumers may impose
their own borrowing periods before storage can be reused.

Copying the target does not copy its buffers or extend their
lifetimes. A zero-initialized target is invalid.

Rendering entry points must validate dimensions, strides,
capacities and arithmetic bounds before writing. Capacity
fields describe caller-provided storage, they cannot prove
the size of the underlying allocations.
====================
*/
typedef struct backend_cpu_target {
    uint32_t *pixels;
    float    *depth;
    vec2i     size;
    size_t    stride_pixels;
    size_t    capacity_pixels;
    size_t    depth_stride_pixels;
    size_t    depth_capacity_pixels;
} backend_cpu_target_t;

/* ============================================================================================== */
/* >API                                                                                           */
/* ============================================================================================== */

/*
====================
backend_cpu_target_is_valid

Checks color and optional depth layouts without accessing their contents.
Color storage must staifsy `image_view_is_valid`.

When `require_depth` is `true`, depth storage must be present.
Present depth storage is always validated, even when not required.
Absent depth requires zero depth stride and capacity.

Both layouts must contain every visible pixel without arithmetic
overflow. Padding after the final visible row is not required.

Defensive in debug and release, invalid descriptions return `false`.
A `true` result validates metadata only. The caller remains responsible
for actual allocation bounds, pointer validity, storage lifetime and
the non-overlap requirements documented by `backend_cpu_target_t`.
====================
*/
static inline bool backend_cpu_target_is_valid(const backend_cpu_target_t *target, bool require_depth) {
    if (!target) return false;

    image_view_t image = {
        .pixels          = target->pixels,
        .size            = target->size,
        .stride_pixels   = target->stride_pixels,
        .capacity_pixels = target->capacity_pixels,
    };
    if (!image_view_is_valid(&image)) return false;

    if (!target->depth) return !require_depth && !target->depth_stride_pixels && !target->depth_capacity_pixels;

    size_t wid = (size_t)target->size.x, hei = (size_t)target->size.y;
    if (target->depth_stride_pixels < wid || target->depth_capacity_pixels < wid) return false;
    if (target->depth_capacity_pixels > SIZE_MAX / sizeof(*target->depth))        return false;

    /* reserve the final rows visible values, then bound the preceding row strides */
    return hei - 1 <= (target->depth_capacity_pixels - wid) / target->depth_stride_pixels;
}

/*
====================
backend_cpu_target_image_view

Creates a read-only view of the targets existing color storage.
Copies the descriptor without allocating or copying pixels.
Depth is optional, but any supplied depth metadata must be valid.

The view borrows the original storage and does not extend its lifetime.
The owner must keep pixels alive and unchanged while a consumer reads
them. This function does not render, clear or synchronize the target.

Defensive in debug and release.

Returns:
- A borrowed color view when `target` metadata is valid.
- A zero-initialized view when `target` is `NULL` or its metadata is invalid.
====================
*/
static inline image_view_t backend_cpu_target_image_view(const backend_cpu_target_t *target) {
    if (!backend_cpu_target_is_valid(target, false)) return (image_view_t){0};

    return (image_view_t){
        .pixels          = target->pixels,
        .size            = target->size,
        .stride_pixels   = target->stride_pixels,
        .capacity_pixels = target->capacity_pixels,
    };
}

/*
====================
backend_cpu_render

Clears and renders a batch into caller-owned CPU storage.
Target dimensions must exactly match `batch->viewport`.

Clears visible color pixels to `clear`. When depth storage is
present, clears visible depth values to `1`. Row padding and
storage beyond the visible image remain unchanged.

Mesh draws require depth storage. Meshes use CCW front faces,
LESS_EQUAL depth testing and depth writes. Sprites blend in
batch order without testing or writing depth.

Uses six-plane homogeneous clipping, pixel-center top-left
coverage and perspective-correct attributes. Atlas sampling
is nearest and clamped to each draw's atlas rectangle.

Mesh lighting uses Lambert diffuse, squared linear range
falloff and smooth spot cones. No lights means unlit.
Color arithmetic performs no gamma conversion.

Sprites use `rgb = src_rgb * src_a + dst_rgb * (1 - src_a)`,
            `a   = src_a + dst_a * (1 - src_a)`.
Use an opaque clear color for opaque application output.

The batch, atlas and targe tdescriptor are borrowed for this
call. Rendering completes before return. The target descriptor
is unchanged, its pixel and depth buffers are modified.

Allocates no heap or arena storage. Output buffers must not
overlap each other or any input read during rendering.

Boundary arguments and target layouts are defensive in release.
Invalid boundary arguments leave output storage unchanged.
Batch records must satifsy engine contracts, debug assertions
diagnose malformed indices and non-finite clip positions.
If malformed draw data is detected after rendering begins,
output may be cleared or partially rendered.

Returns:
- `OK`      on success, including an empty cleared batch.
 - `EINVAL` for invalid arguments, incompatible target dimensions,
            missing required depth or atlas storage, or detected
            malformed mesh topology.
====================
*/
res_t backend_cpu_render(const render_batch_t *batch, const asset_atlas_t *atlas,
                         const backend_cpu_target_t *target, uint32_t clear);
