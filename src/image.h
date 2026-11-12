#pragma once

/*
====================
image.h

Borrowed views of CPU-accessible image storage.
Used by image encoding, capture and rendering interfaces.

Pixels use numeric 0xRRGGBBAA colors. Native memory byytes
are not necessarily ordered R, G, B, A.

Views describe storage only. They do not allocate, release,
synchronize or convert pixels.
====================
*/

#include "gmath.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================================================== */
/* >TYPES                                                                                         */
/* ============================================================================================== */

/*
====================
image_view_t

Read-only view of borrowed numeric 0xRRGGBBAA pixels.
The origin is top-left, X increases right and Y increases down.

Fields:
- `pixels`:          first visible pixel.
- `size`:            visible width and height in pixels, both positive.
- `stride_pixels`:   distance between row starts, at least `size.x`.
- `capacity_pixels`: accessible uint32_t elements starting at pixels,
                     including any padding between visible rows.

Pixel (x, y) is `pixels[y * stride_pixels + x]` (or `pixels[IDX_2D(x, y, stride_pixels)]`).
Padding after the final visible row is not required.

The owner must keep the storage alive and unchanged while a
consumer reads it. Copying this view does not copy the pixels
or extend their lifetime. A zero-initialized view is invalid.

Consumers must validate dimensions, stride, capacity and
arithmetic bounds before accessing pixels. The capacity is
a caller-provided contract, not proof of an allocations size.

This view specifies channel storage, not color space or
alpha interpretation. Those semantics belong to the producing
and consuming APIs, constructing a view performs no conversion.
====================
*/
typedef struct image_view {
    const uint32_t *pixels;
    vec2i           size;
    size_t          stride_pixels;
    size_t          capacity_pixels;
} image_view_t;

/* ============================================================================================== */
/* >API                                                                                           */
/* ============================================================================================== */

/*
====================
image_view_is_valid

Checks the pixel pointer and image layout without reading pixels.
Requires positive dimensions, non-overlapping rows and sufficient
capacity through the final visible pixel. Final-row padding is
not required.

Rejects capacities whose byte count cannot fit in `size_t`.
Checks row coverage without overflowing stride multiplication.

Defensive in debug and release, invalid descriptions return `false`.
A `true` result validates metadata only. The caller remains responsible
for the actual allocation size, pointer validity and storage lifetime.
====================
*/
static inline bool image_view_is_valid(const image_view_t *view) {
    if (!view || !view->pixels || view->size.x <= 0 || view->size.y <= 0) return false;

    size_t wid = (size_t)view->size.x, hei = (size_t)view->size.y;
    if (view->stride_pixels < wid || view->capacity_pixels < wid) return false;
    if (view->capacity_pixels > SIZE_MAX / sizeof(*view->pixels)) return false;

    /* reserve the final rows visible pixels, then bound the preceding row strides */
    return hei - 1 <= (view->capacity_pixels - wid) / view->stride_pixels;
}
