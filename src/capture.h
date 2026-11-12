#pragma once

/*
====================
capture.h

Scratch-backed capture through the CPU renderer
and PNG encoding of borrowed image views.

Colors use numeric 0xRRGGBBAA with a top-left origin.
PNG encoding preserves stored channels without color-space
or alpha conversion and skips padding between image rows.
====================
*/

#include "image.h"
#include "render.h"

/* ============================================================================================== */
/* >STRUCTS                                                                                       */
/* ============================================================================================== */

/*
====================
capture_frame_t

Borrowed scratch-backed output.
- `pixels`: `size.x * size.y` numeric RGBA colors.
- `depth`:  normalized [0, 1], initially 1, `NULL` for batches without meshes.
- `size`:   framebuffer dimensions in pixels.

Arrays remain valid until their scratch storage is reset or unwound.
====================
*/
typedef struct capture_frame {
    uint32_t *pixels;
    float    *depth;
    vec2i     size;
} capture_frame_t;

/* ============================================================================================== */
/* >API                                                                                           */
/* ============================================================================================== */

/*
====================
capture_render

Renders a batch into newly allocated scratch storage,
initially filled with clear. The batch and atlas are
borrowed for the call. Earler scratch allocations
survive. Uses six-plane homogeneous clipping, pixel-center
top-left coverage, and perspective-correct attributes.
Mesh lighting uses Lambert diffuse, squared linear range
falloff, and smooth spot cones. No lights means unlit.
No gamma conversion.

Sprites use rgb = src_rgb * src_a + dst_rgb * (1 - src_a),
              a = src_a + dst_a * (1 - src_a).
Use an opaque clear color for an opaque capture.

Boundary arguments are defensive in release. Batch records
must be valid engine data, debug asserts diagnose malformed
indices and non-finite clip positions. On failure, output
is zeroed and allocations are unwound. OOM asserts in debug.

Returns:
- `OK`     on success, including an empty cleared batch.
- `EINVAL` for invalid arguments, viewport, missing arrays, or malformed mesh topology.
- `EOOM`   when scratch cannot hold the color and optional depth arrays.
====================
*/
res_t capture_render(const render_batch_t *batch, const asset_atlas_t *atlas, arena_t *scratch,
                     uint32_t clear, capture_frame_t *out_frame);

/*
====================
capture_encode_png

Encodes the visible pixels of an image view into a PNG
using uncompressed DEFLATE blocks. Source rows follow
`stride_pixels`, padding is omitted. Numeric 0xRRGGBBAA
colors become RGBA bytes without color-space or alpha conversion.

Borrows the image and its pixels for this call. Source storage
must remain valid and unchanged during encoding. It may occupy
earlier allocations in scratch. Output storage must not overlap
the input or the output pointer/size arguments.

Uses scratch only, without heap allocation or file I/O.
Earlier allocations survive. Output bytes remain valid until
their scratch storage is reset or unwound.

This favors simple capture over compressed file size.
One IDAT chunk is used, limited to INT32_MAX bytes.

Defensive in debug and release. Invalid arguments clear any
provided outputs and consume no scratch storage.
OOM asserts in debug and returns EOOM in release.

Returns:
- `OK`     on success.
- `EINVAL` for invalid arguments, invalid image layout or an image
           exceeding the PNG size limits.
- `EOOM`   when scratch cannot hold the PNG.
====================
*/
res_t capture_encode_png(const image_view_t *image, arena_t *scratch, uint8_t **out_bytes, size_t *out_size);
