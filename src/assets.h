#pragma once

/*
====================
assets.h

Texture atlas management, procedural canvas rasterization,
and asset ingestion. Maintains a unified 32-bit RGBA
texture atlas in permanent arena memory with:
- dual CPU/GPU backing for headless CPU software blitting and GPU textures
- pre-allocated UV and pixel bounds table for ECS `sprite2d` components
- built-in procedural fallbacks (ID 0 solid white quad, ID 1 checkerboard)
- text pixel art DSL (ASCII character mapping with 16-color palettes)
- 9-primitive procedural vector canvas (anti-aliased shapes, curves and gradients)
- memory-first and file-based dynaminc PNG/BMP/TGA loaders and spritesheet slicing
- in-place asset hot reloading without atlas re-packing or game restart
- atlas-level color mapping and palette swapping
- disk export APIs (PNG)
====================
*/

#include "arena.h"
#include "core.h"
#include "gmath.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint16_t asset_sprite_id_t;

/* ============================================================================================== */
/* >CONSTANTS                                                                                     */
/* ============================================================================================== */

#define ASSET_ATLAS_DEF_WID     1024U
#define ASSET_ATLAS_DEF_HEI     1024U
#define ASSET_ATLAS_MAX_SPRITES 2048U

#define ASSET_SPRITE_WHITE      ((asset_sprite_id_t)0U)
#define ASSET_SPRITE_CHECKER    ((asset_sprite_id_t)1U)
#define ASSET_SPRITE_USER_START ((asset_sprite_id_t)2U)
#define ASSET_SPRITE_NULL       ((asset_sprite_id_t)0xFFFFU)

#define COLOR_RGBA(r, g, b, a) (((uint32_t)(r) << 24) | ((uint32_t)(g) << 16) | ((uint32_t)(b) << 8) | (uint32_t)(a))
#define COLOR_R(col)           ((uint32_t)((col) >> 24) & 0xFFU)
#define COLOR_G(col)           ((uint32_t)((col) >> 16) & 0xFFU)
#define COLOR_B(col)           ((uint32_t)((col) >> 8)  & 0xFFU)
#define COLOR_A(col)           ((uint32_t)(col)         & 0xFFU)

#define COLOR_CHAN_MAX 0xFFU
#define COLOR_CHAN_MIN 0x00U

#define COLOR_WHITE   0xFFFFFFFFU
#define COLOR_BLACK   0x000000FFU
#define COLOR_CLEAR   0x00000000U
#define COLOR_MAGENTA 0xFF00FFFFU
#define COLOR_RED     0xFF0000FFU
#define COLOR_YELLOW  0xFFFF00FFU
#define COLOR_GREEN   0x00FF00FFU
#define COLOR_CYAN    0x00FFFFFFU
#define COLOR_BLUE    0x0000FFFFU

#define PIVOT_TOP_LEFT      vec2f(0.0f, 0.0f)
#define PIVOT_TOP_CENTER    vec2f(0.5f, 0.0f)
#define PIVOT_TOP_RIGHT     vec2f(1.0f, 0.0f)
#define PIVOT_CENTER_LEFT   vec2f(0.0f, 0.5f)
#define PIVOT_CENTER_CENTER vec2f(0.5f, 0.5f)
#define PIVOT_CENTER        PIVOT_CENTER_CENTER
#define PIVOT_CENTER_RIGHT  vec2f(1.0f, 0.5f)
#define PIVOT_BOTTOM_LEFT   vec2f(0.0f, 1.0f)
#define PIVOT_BOTTOM_CENTER vec2f(0.5f, 1.0f)
#define PIVOT_BOTTOM_RIGHT  vec2f(1.0f, 1.0f)

/* ============================================================================================== */
/* >DATA TYPES                                                                                    */
/* ============================================================================================== */

/*
====================
asset_sprite_t

Public metadata descriptor for a registered
sprite within the texture atlas.

Fields:
- `uv`:    Normalized texture coordinates [u0, v0, u1, v1] within [0.0f, 1.0f].
- `px`:    Integer pixel boundaries [min_x, min_y, max_x, max_y] in atlas memory.
- `pivot`: Normalized origin anchor point (default <0.5f, 0.5f> center).
- `id`:    Unique sprite index corresponding to `sprite2d.id`.
====================
*/
typedef struct asset_sprite {
    aabb2f            uv;
    aabb2i            px;
    vec2f             pivot;
    asset_sprite_id_t id;
    uint8_t          _pad[2];
} asset_sprite_t;

/*
====================
asset_atlas_t

Managed master texture atlas subsystem.
Maintains a contiguous 32-bit RGBA pixel
buffer allocated in permanent arena storage.
Coordinates dynamic two-dimensional bin packing,
maintains the global sprite lookup table, and
manages the CPU-to-GPU sync lifecycle.

Invariants:
- `size`   is immutable after initialization (default: <1024, 1024>)
- `id = 0` (`ASSET_SPRITE_WHITE`)   is permanently reserved for a 1x1 solid white pixel.
- `id = 1` (`ASSET_SPRITE_CHECKER`) is permanently reserved for an 8x8 magenta/black checkerboard.
- Internal pixel memory is retained on CPU for headless software blitting.
- All pixels and metadata reside in the caller-provided permanent arena.

Lifecycle:
- Initialized via `asset_atlas_init`.
- Populated via `asset_load_*`, `asset_bake_*`, or `asset_atlas_alloc_slot`.
- Synchronized to GPU via `asset_atlas_sync_gpu`.
====================
*/
typedef struct asset_atlas {
    uint32_t       *pixels;      /* sz: size.x * size.y * sizeof(uint32_t), perm-alloced   */
    asset_sprite_t *sprites;     /* sz: max_sprites * sizeof(asset_sprite_t), perm-alloced */
    void           *packer_mem;  /* persistent internal rect-packer memory node array      */
    vec2i           size;        /* atlas width (size.x) and height (size.y) in pixels     */
    uint32_t        max_sprites;
    uint32_t        sprite_cnt;
    bool            dirty;       /* true when CPU pixels modified, reset on GPU sync       */
    uint8_t        _pad[7];
} asset_atlas_t;

/*
====================
asset_slot_t

Descriptor for an allocated rectangular
region within the texture atlas bitmap.
Provides direct pixel pointer and stride
for procedural drawing and rasterization.

Fields:
- `atlas`:  Pointer to the parent atlas owner.
- `pixels`: Direct pointer to the top-left 32-bit RGBA pixel in atlas bitmap memory.
- `stride`: Row pitch in pixels (matches total atlas width).
- `px`:     Integer pixel boundaries [min_x, min_y, max_x, max_y] in atlas memory.
- `id`:     Sprite identifier registered for this slot.
====================
*/
typedef struct asset_slot {
    asset_atlas_t    *atlas;
    uint32_t         *pixels;
    uint32_t          stride;
    aabb2i            px;
    asset_sprite_id_t id;
    bool              antialiased;
    uint8_t          _pad;
} asset_slot_t;

/* ============================================================================================== */
/* >LIFECYCLE                                                                                     */
/* ============================================================================================== */

/*
====================
asset_atlas_init

Initializes the master texture atlas within
permanent arena memory. Allocates the 32-bit
RGBA pixel buffer, sprite metadata table, and
2D bin packer state. Automatically registers
and bakes `ASSET_SPRITE_WHITE`   (1x1 solid white quad)
      and `ASSET_SPRITE_CHECKER` (8x8 magenta/black checkerboard fallback).
If `size.x <= 0 || size.y <= 0` defaults to (1024, 1024).
If `max_sprites == 0`,          defaults to `ASSET_ATLAS_MAX_SPRITES` (2048).
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`     on successful initialization.
- `EINVAL` if `atlas` or `perm` is `NULL`.
- `EOOM`   if `perm` lacks memory for pixels, sprite tables, or packing structures.
====================
*/
res_t asset_atlas_init(asset_atlas_t *atlas, arena_t *perm, vec2i size, uint32_t max_sprites);

/*
====================
asset_atlas_reset

Resets the master texture atlas, clearing all user-allocated
sprites and reclaiming all packed 2D pixel space.
Reinitializes the skyline bin packer, zeroes the pixel and
metadata buffers, re-registers and bakes reserved fallbacks
`ASSET_SPRITE_WHITE` (ID 0) and `ASSET_SPRITE_CHECKER` (ID 1),
and sets `dirty = true` to trigger GPU synchronization.
Asserts non-null atlas in debug mode, returns result code in release.

Returns:
- `OK`     on successful reset.
- `EINVAL` if `atlas` is `NULL` or uninitialized.
====================
*/
res_t asset_atlas_reset(asset_atlas_t *atlas);

/*
====================
asset_atlas_alloc_slot

Allocates an uninitialized rectangular slot
of dimensions `size` in the atlas bitmap.
Uses 2D skyline bin packing to find free space,
assigns the next sequential sprite ID, records
metadata, marks `dirty = true`, and returns
an `asset_slot_t` pointing to the top-left pixel.
Asserts valid inputs in debug, returns result code in release.

Returns:
- `OK`     on successful slot allocation.
- `EINVAL` if `atlas` or `out_slot` is `NULL`, or `size_x <= 0 || size.y <= 0`.
- `EFULL`  if the atlas bitmap has no remaining space for `size`, or `sprite_cnt >= max_sprites`.
====================
*/
res_t asset_atlas_alloc_slot(asset_atlas_t *atlas, vec2i size, asset_slot_t *out_slot);

/* ============================================================================================== */
/* >PROCEDURAL VECTOR CANVAS                                                                      */
/* ============================================================================================== */

/*
====================
asset_slot_clear

Fills all pixels in `slot` with solid color `col`.
Early exits if `slot` or `pixels` is `NULL`.
====================
*/
void asset_slot_clear(asset_slot_t *slot, uint32_t col);

/*
====================
asset_slot_set_pixel

Blends a single 32-bit RGBA pixel at local
coordinate `px` inside `slot`.
Uses standard alpha over-compositing.
Ignores coordinates outside slot boundaries.
Early exits if `slot` is `NULL`.
====================
*/
void asset_slot_set_pixel(asset_slot_t *slot, vec2i px, uint32_t col);

/*
====================
asset_slot_draw_rect

Draws a filled rectangle `rect` inside `slot` with alpha blending.
Clamps rectangle to slot boundaries.
Early exits if `slot` is `NULL`.
====================
*/
void asset_slot_draw_rect(asset_slot_t *slot, aabb2i rect, uint32_t col);

/*
====================
asset_slot_draw_rect_outline

Draws an anti-aliased rectangular border with `thickness` in pixels.
Early exits if `slot` is `NULL`.
====================
*/
void asset_slot_draw_rect_outline(asset_slot_t *slot, aabb2i rect, float thickness, uint32_t col);

/*
====================
asset_slot_draw_rect_rounded

Draws an anti-aliased filled rounded rectangle with corner radius `radius` in pixels.
Early exits if `slot` is `NULL`.
====================
*/
void asset_slot_draw_rect_rounded(asset_slot_t *slot, aabb2i rect, float radius, uint32_t col);

/*
====================
asset_slot_draw_circle

Draws an anti-aliased filled circle defined by local `center` and `radius`.
Early exits if `slot` is `NULL`.
====================
*/
void asset_slot_draw_circle(asset_slot_t *slot, vec2f center, float radius, uint32_t col);

/*
====================
asset_slot_draw_circle_outline

Draws an anti-aliased circular outline with `thickness` in pixels.
Early exits if `slot` is `NULL`.
====================
*/
void asset_slot_draw_circle_outline(asset_slot_t *slot, vec2f center, float radius, float thickness, uint32_t col);

/*
====================
asset_slot_draw_triangle

Draws an anti-aliased filled triangle connecting local vertices `p0`, `p1` and `p2`.
Early exits if `slot` is `NULL`.
====================
*/
void asset_slot_draw_triangle(asset_slot_t *slot, vec2f p0, vec2f p1, vec2f p2, uint32_t col);

/*
====================
asset_slot_draw_line

Draws an anti-aliased line segment between `p0` and `p1` with `thickness` in pixels.
Early exits if `slot` is `NULL`.
====================
*/
void asset_slot_draw_line(asset_slot_t *slot, vec2f p0, vec2f p1, float thickness, uint32_t col);

/*
====================
asset_slot_draw_bezier

Draws an anti-aliased quadratic Bezier curve defined by start `p0`,
control `p1`, and end `p2` with `thickness` in pixels.
Early exits if `slot` is `NULL`.
====================
*/
void asset_slot_draw_bezier(asset_slot_t *slot, vec2f p0, vec2f p1, vec2f p2, float thickness, uint32_t col);

/*
====================
asset_slot_draw_gradient_linear

Draws a vertical linear color gradient between `col_top` and `col_bot` inside `rect`.
Early exits if `slot` is `NULL`.
====================
*/
void asset_slot_draw_gradient_linear(asset_slot_t *slot, aabb2i rect, uint32_t col_top, uint32_t col_bot);

/*
====================
asset_slot_draw_gradient_radial

Draws an anti-aliased radical color gradient from `center` out to `radius`.
Early exits if `slot` is `NULL`.
====================
*/
void asset_slot_draw_gradient_radial(asset_slot_t *slot, vec2f center, float radius, uint32_t col_in, uint32_t col_out);

/* ============================================================================================== */
/* >TEXT PIXEL ART                                                                                */
/* ============================================================================================== */

/* built-in PICO-8 16-color palette */
extern const uint32_t g_palette_pico8[16];

/*
====================
asset_bake_text_sprite

Bakes a compact ASCII pixel-art string into a newly allocated atlas slot.
Characters `0`-`9` and `A/a`-`F/f` index into `palette_16` (0-15).
Characters `.`, ` ` and `-` represent transparent pixels.
Newlines and carriage returns are automatically skipped during parsing.
If `palette_16` is `NULL`, defaults to `g_palette_pico8`.
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`     on successful baking.
- `EINVAL` if `atlas`, `ascii`, or `out_id` is `NULL`, or `size.x <= 0 || size.y <= 0`.
- `EFULL`  if atlas lacks space for slot or sprite count exceeds maximum.
====================
*/
res_t asset_bake_text_sprite(asset_atlas_t *atlas, const char *ascii, vec2i size, const uint32_t *palette_16, asset_sprite_id_t *out_id);

/*
====================
asset_bake_text_animation

Bakes an array of `frame_cnt` ASCII text frames into the atlas.
Allocates consecutive sprite IDs starting at `out_base_id` so that frame `i`
can be indexed as `*out_base_id + i`.
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`     on successful baking of all frames.
- `EINVAL` if parameters are `NULL` or counts/dimensions are zero.
- `EFULL`  if atlas lacks space for all animation frames.
====================
*/
res_t asset_bake_text_animation(asset_atlas_t *atlas, const char **frames, uint32_t frame_cnt, vec2i size, const uint32_t *palette_16, asset_sprite_id_t *out_base_id);

/* ============================================================================================== */
/* >IMAGE LOADERS / SPRITESHEET LOGIC                                                             */
/* ============================================================================================== */

/*
====================
asset_load_image_mem

Decodes an image from memory buffer `buf` (PNG, BMP, TGA) of size `size` via `stb_image`,
allocates an atlas slot, copies pixels, updates sprite metadata, and frees decoded memory.
On success sets `*out_id` to new sprite ID.
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`      on successful image loading and atlas packing.
- `EINVAL`  if `atlas`, `buf`, or `out_id` is `NULL`, or `size == 0`.
- `ENOTFND` if image decoding fails (invalid or corrupt format).
- `EFULL`   if atlas has insufficient space for decoded dimensions.
====================
*/
res_t asset_load_image_mem(asset_atlas_t *atlas, const void *buf, size_t size, asset_sprite_id_t *out_id);

/*
====================
asset_load_image_file

Reads and decodes an image file from filesystem path `path` into the texture atlas.
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`      on successful image loading and atlas packing.
- `EINVAL`  if `atlas`, `path` or `out_id` is `NULL`.
- `ENOTFND` if file cannot be opened or image decoding fails.
- `EFULL`   if atlas has insufficient space for decoded dimensions.
====================
*/
res_t asset_load_image_file(asset_atlas_t *atlas, const char *path, asset_sprite_id_t *out_id);

/*
====================
asset_register_grid

Slices an existing atlas sprite `src_id` (such as a loaded spritesheet) into a uniform
grid of sub-sprites of dimensions `cell_size`.
Registers new sprite metadata entries pointing to the existing pixel memory without copying.
Stores allocated sub-sprite IDs into `out_ids` up to `max_out`, writing total to `out_cnt`.
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`     on successful grid slicing.
- `EINVAL` if parameters are `NULL`, `source_id` is invalid, or cell sizes are zero.
- `EFULL`  if total grid sub-sprites would exceed `max_sprites`.
====================
*/
res_t asset_register_grid(asset_atlas_t *atlas, asset_sprite_id_t src_id, vec2i cell_size, asset_sprite_id_t *out_ids, uint32_t max_out, uint32_t *out_cnt);

/* ============================================================================================== */
/* >HOT RELOADING                                                                                 */
/* ============================================================================================== */

/*
====================
asset_reload_image_file

Re-reads an image from `path` and updates existing sprite `id` in-place.
Decoded dimensions must match the sprite's existing pixel rectangle `px`.
Marks `dirty = true` to trigger GPU synchronization on the next tick without re-packing.
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`      on successful in-place reload.
- `EINVAL`  if parameters are `NULL`, `id` is unallocated, or dimensions do not match.
- `ENOTFND` if file cannot be opened or decoded.
====================
*/
res_t asset_reload_image_file(asset_atlas_t *atlas, asset_sprite_id_t id, const char *path);

/*
====================
asset_reload_image_mem

Re-decodes image memory from `buf` and updates existing sprite `id` in-place.
Decoded dimensions must match the sprite's existing pixel rectangle `px`.
Marks `dirty = true` to trigger GPU synchronization on the next tick without re-packing.
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`      on successful in-place reload.
- `EINVAL`  if parameters are `NULL`, `id` is unallocated, or dimensions do not match.
- `ENOTFND` if decoding fails.
====================
*/
res_t asset_reload_image_mem(asset_atlas_t *atlas, asset_sprite_id_t id, const void *buf, size_t size);

/* ============================================================================================== */
/* >PALETTE SWAPPING                                                                              */
/* ============================================================================================== */

/*
====================
asset_atlas_remap_palette

Scans the entire atlas bitmap and replaces any pixel matching `src_palette[k]` with
`dst_palette[k]` for all `k < count`. Sets `dirty = true` for GPU synchronization.
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`     on successful palette remap.
- `EINVAL` if `atlas`, `src_palette`, or `dst_palette` is `NULL`, or `count == 0`.
====================
*/
res_t asset_atlas_remap_palette(asset_atlas_t *atlas, const uint32_t *src_palette, const uint32_t *dst_palette, uint32_t count);

/*
====================
asset_atlas_remap_color

Replaces all occurences of `src_col` across the atlas bitmap with `dst_col`.
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`     on successful color replacement.
- `EINVAL` if `atlas` is `NULL`.
====================
*/
res_t asset_atlas_remap_color(asset_atlas_t *atlas, uint32_t src_col, uint32_t dst_col);

/* ============================================================================================== */
/* >DISK EXPORT                                                                                   */
/* ============================================================================================== */

/*
====================
asset_export_sprite_png

Exports an individual sprite cropped to its pixel boundaries as an RGBA PNG file to `path`.
If `overwrite` is `false` and `path` already exists on disk,
the export safely aborts without modifying the existing file.
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`     on successful file write.
- `EINVAL` if `atlas` or `path` is `NULL`, or `id` is unallocated.
- `EIO`    if filesystem write fails, or if file exists and `overwrite` is `false`.
====================
*/
res_t asset_export_sprite_png(const asset_atlas_t *atlas, asset_sprite_id_t id, const char *path,
                              bool overwrite, arena_t *scratch);

/*
====================
asset_export_atlas_png

Exports the entire master atlas bitmap as an RGBA PNG file to `path`.
If `overwrite` is `false` and `path` already exists on disk, the export safely aborts.
Asserts non-null pointers in debug, returns result code in release.

Returns:
- `OK`     on successful file write.
- `EINVAL` if `atlas` or `path` is `NULL`.
- `EIO`    if filesystem write fails, or if file exists and `overwrite` is `false`.
====================
*/
res_t asset_export_atlas_png(const asset_atlas_t *atlas, const char *path, bool overwrite, arena_t *scratch);

/* ============================================================================================== */
/* >MUTATIONS                                                                                     */
/* ============================================================================================== */

/*
====================
asset_pivot_from_px

Converts a local pixel coordinate within a sprite of `sprite_sz`
into normalized [0.0f, 1.0f] pivot coordinates.
Guards division by zero, falling back to center <0.5f, 0.5f> (PIVOT_CENTER).
Supports sub-pixel anchor positioning.
====================
*/
static inline vec2f asset_pivot_from_px(vec2i sprite_sz, vec2f px_off) {
    return (sprite_sz.x <= 0 || sprite_sz.y <= 0) ?
           PIVOT_CENTER                           :
           vec2f(px_off.x / (float)sprite_sz.x, px_off.y / (float)sprite_sz.y);
}

/*
====================
asset_sprite_set_pivot

Sets the normalized pivot point for sprite `id`
(e.g., <0.5f, 0.5f> (PIVOT_CENTER) for center,
<0.5f, 1.0f> (PIVOT_BOTTOM_CENTER) for bottom-center,
<0.0f, 0.0f> (PIVOT_TOP_LEFT) for top-left).
Asserts `atlas` is not `NULL` and `id` is allocated in debug mode.

Returns:
- `OK`     on successful pivot update.
- `EINVAL` if `atlas` is `NULL` or `id >= atlas->sprite_cnt`.
====================
*/
res_t asset_sprite_set_pivot(asset_atlas_t *atlas, asset_sprite_id_t id, vec2f norm_pivot);

/*
====================
asset_set_sprite_pivot_px

Sets sprite pivot using local pixel coordinates
relative to the sprite's top-left corner.
Normalizes the coordinates against the sprite's actual pixel dimensions.
Supports sub-pixel anchor positioning.
Asserts `atlas` is not `NULL` and `id` is allocated in debug mode.

Returns:
- `OK`     on successful pivot update.
- `EINVAL` if `atlas` is `NULL` or `id >= atlas->sprite_cnt`.
====================
*/
res_t asset_set_sprite_pivot_px(asset_atlas_t *atlas, asset_sprite_id_t id, vec2f px_off);

/*
====================
asset_slot_set_antialiased

Enables or disables sub-pixel antialiasing for subsequent
vector drawing operations on `slot` (default: `true`).
====================
*/
static inline void asset_slot_set_antialiased(asset_slot_t *slot, bool enabled) {
    if (slot) slot->antialiased = enabled;
}

/*
====================
asset_slot_set_pivot

Sets normalized pivot point for the sprite associated with `slot`.
Guards `NULL` slot or `NULL` atlas pointer.

Returns:
- `EINVAL` if `slot` or `slot->atlas` are NULL.
- result of `asset_set_sprite_pivot` call otherwise.
====================
*/
static inline res_t asset_slot_set_pivot(asset_slot_t *slot, vec2f norm_pivot) {
    if (!slot || !slot->atlas) return EINVAL;
    return asset_sprite_set_pivot(slot->atlas, slot->id, norm_pivot);
}

/* ============================================================================================== */
/* >QUERIES                                                                                       */
/* ============================================================================================== */

/*
====================
asset_get_sprite

Queries sprite metadata for `id`.
Asserts `atlas` is valid in debug mode.

Returns:
- the pointer to metadata on successful query.
- `NULL` if `id` is unallocated or out of bounds.
====================
*/
const asset_sprite_t *asset_get_sprite(const asset_atlas_t *atlas, asset_sprite_id_t id);

/*
====================
asset_get_uv

Returns normalized UV coordinates [u0, v0, u1, v1] for `id`.
Asserts `atlas` is valid in debug mode.

Returns:
- the UV coordinates for `id` on successful query.
- the UV coordinates of `ASSET_SPRITE_CHECKER` if `id` is unallocated or invalid.
====================
*/
aabb2f asset_get_uv(const asset_atlas_t *atlas, asset_sprite_id_t id);

/*
====================
asset_get_pixel

Returns 32-bit RGBA pixel at global atlas coordinate `pos`.
Asserts `atlas` is valid and `pos` is in bounds in debug mode.

Returns:
- the color for provided `pos` if `pos` is in bounds.
- 0 if coordinate is out of bounds or `atlas` is `NULL`.
====================
*/
uint32_t asset_get_pixel(const asset_atlas_t *atlas, vec2i pos);

/* ============================================================================================== */
/* >GPU SYNCHRONIZATION                                                                           */
/* ============================================================================================== */

/*
====================
asset_atlas_sync_gpu

Synchronizes modified CPU atlas pixel memory to the GPU texture image.
If `atlas->dirty` is `true`, updates the underlying GPU texture and clears `dirty`.
Asserts `atlas` is not `NULL`. Safe to call every frame, whenever needed.

Returns:
- `OK`     on successful synchronization or if atlas was not dirty.
- `EINVAL` if `atlas` is `NULL` or uninitialized.
====================
*/
res_t asset_atlas_sync_gpu(asset_atlas_t *atlas);
