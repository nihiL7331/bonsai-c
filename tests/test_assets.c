#include "assets.h"
#include "arena.h"
#include "test.h"
#include "stb_image_write.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

extern unsigned char *stbi_write_png_to_mem(const unsigned char *pixels, int stride_bytes, int x, int y, int n, int *out_len);

#define EPS 0.001f

static inline bool feq(float a, float b) {
    return fabsf(a - b) <= EPS;
}

int main(void) {
    uint8_t arena_buf[MB(1)];
    arena_t perm;

    TEST_CASE("atlas_init", "initializes atlas with ID0 and ID1 defaults") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        res_t r = asset_atlas_init(&atlas, &perm, vec2i(128, 128), 64);
        TEST_ASSERT(r == OK,                                    "expected OK, got %d",         r);
        TEST_ASSERT(atlas.size.x == 128 && atlas.size.y == 128, "expected 128x128, got %dx%d", atlas.size.x, atlas.size.y);
        TEST_ASSERT(atlas.max_sprites == 64,                    "expected 64 max sprites, got %" PRIu32, atlas.max_sprites);
        TEST_ASSERT(atlas.sprite_cnt == 2,                      "expected 2 baked sprites, got %" PRIu32, atlas.sprite_cnt);
        TEST_ASSERT(atlas.dirty,                                "atlas should be marked dirty after init");

        /* test ID 0 (solid white 1x1) */
        const asset_sprite_t *s0 = asset_get_sprite(&atlas, ASSET_SPRITE_WHITE);
        TEST_ASSERT(s0,                                                  "sprite 0 should exist");
        TEST_ASSERT(s0->px.min_x == 0 && s0->px.min_y == 0,              "s0 origin (0, 0)");
        TEST_ASSERT(s0->px.max_x == 1 && s0->px.max_y == 1,              "s0 1x1 dimensions");
        TEST_ASSERT(asset_get_pixel(&atlas, vec2i(0, 0)) == COLOR_WHITE, "s0 pixel must be COLOR_WHITE");

        /* test ID 1 (checkerboard 8x8) */
        const asset_sprite_t *s1 = asset_get_sprite(&atlas, ASSET_SPRITE_CHECKER);
        TEST_ASSERT(s1 != NULL, "sprite 1 should exist");
        vec2i s1_sz = aabb_size(s1->px);
        TEST_ASSERT(s1_sz.x == 8 && s1_sz.y == 8, "s1 must be 8x8, got %dx%d", s1_sz.x, s1_sz.y);

        /* verify checkerboard pattern at (0,0) and (4,0) */
        uint32_t c_0_0 = asset_get_pixel(&atlas, vec2i(s1->px.min_x, s1->px.min_y));
        uint32_t c_4_0 = asset_get_pixel(&atlas, vec2i(s1->px.min_x + 4, s1->px.min_y));
        TEST_ASSERT(c_0_0 == COLOR_MAGENTA, "checker (0,0) must be MAGENTA, got 0x%08X", c_0_0);
        TEST_ASSERT(c_4_0 == COLOR_BLACK,   "checker (4,0) must be BLACK, got 0x%08X", c_4_0);
    } TEST_CASE_END();

    TEST_CASE("atlas_alloc_slot", "packs non-overlapping slots and respects limits") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(128, 128), 32);

        asset_slot_t slots[8];
        vec2i sizes[] = {
            vec2i(16, 16),
            vec2i(32, 16),
            vec2i(24, 24),
            vec2i(8,  32),
            vec2i(16, 8),
            vec2i(48, 16),
            vec2i(16, 16),
            vec2i(32, 32),
        };

        for (int i = 0; i < 8; ++i) {
            res_t r = asset_atlas_alloc_slot(&atlas, sizes[i], &slots[i]);
            TEST_ASSERT(r == OK,                                   "slot %d alloc failed with %d", i, r);
            TEST_ASSERT(slots[i].id == (asset_sprite_id_t)(i + 2), "expected ID %d, got %" PRIu32, i + 2, slots[i].id);

            vec2i actual_sz = aabb_size(slots[i].px);
            TEST_ASSERT(actual_sz.x == sizes[i].x && actual_sz.y == sizes[i].y,
                        "slot %d size mismatch: exp %dx%d, got %dx%d",
                        i, sizes[i].x, sizes[i].y, actual_sz.x, actual_sz.y);

            /* verify slot is within atlas bounds */
            TEST_ASSERT(slots[i].px.min_x >= 0 && slots[i].px.max_x <= atlas.size.x, "slot %d X out of bounds", i);
            TEST_ASSERT(slots[i].px.min_y >= 0 && slots[i].px.max_y <= atlas.size.y, "slot %d Y out of bounds", i);

            /* verify no pixel overlap with previous slots */
            for (int j = 0; j < i; ++j) {
                bool overlap = aabb_valid(aabb_clamp(slots[i].px, slots[j].px));
                TEST_ASSERT(!overlap, "slot %d overlaps with slot %d", i, j);
            }
        }

        /* test atlas overflow (allocating beyond available dimensions returns EFULL) */
        asset_slot_t overflow_slot;
        res_t r_overflow = asset_atlas_alloc_slot(&atlas, vec2i(100, 100), &overflow_slot);
        TEST_ASSERT(r_overflow == EFULL, "expected EFULL when atlas cannot fit slot, got %d", r_overflow);
    } TEST_CASE_END();

    TEST_CASE("atlas_reset", "clears user sprites, reclaims space") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(64, 64), 16);

        /* allocate user slots that fill up the atlas */
        asset_slot_t s1, s2;
        asset_atlas_alloc_slot(&atlas, vec2i(32, 32), &s1);
        asset_slot_clear(&s1, COLOR_RED);
        asset_atlas_alloc_slot(&atlas, vec2i(32, 32), &s2);
        asset_slot_clear(&s2, COLOR_BLUE);

        TEST_ASSERT(atlas.sprite_cnt == 4, "expected 4 sprites before reset (ID 0..3)");
        TEST_ASSERT(asset_get_sprite(&atlas, 2) != NULL, "sprite 2 must exist before reset");

        /* large allocation should now fail due to lack of space */
        asset_slot_t s_big;
        res_t r_big = asset_atlas_alloc_slot(&atlas, vec2i(60, 60), &s_big);
        TEST_ASSERT(r_big == EFULL, "atlas should be full before reset");

        /* reset atlas */
        res_t r_reset = asset_atlas_reset(&atlas);
        TEST_ASSERT(r_reset == OK, "atlas reset failed: %d", r_reset);
        TEST_ASSERT(atlas.sprite_cnt == ASSET_SPRITE_USER_START, "sprite_cnt must reset to 2, got %" PRIu32, atlas.sprite_cnt);
        TEST_ASSERT(atlas.dirty == true, "atlas must be dirty after reset");

        /* ID 0 and ID 1 must be properly preserved and repainted */
        const asset_sprite_t *s0 = asset_get_sprite(&atlas, ASSET_SPRITE_WHITE);
        TEST_ASSERT(s0 != NULL, "ID 0 must exist after reset");
        TEST_ASSERT(asset_get_pixel(&atlas, vec2i(0, 0)) == COLOR_WHITE, "ID 0 pixel must be WHITE");

        const asset_sprite_t *s1_check = asset_get_sprite(&atlas, ASSET_SPRITE_CHECKER);
        TEST_ASSERT(s1_check != NULL, "ID 1 must exist after reset");
        TEST_ASSERT(asset_get_pixel(&atlas, s1_check->px.min) == COLOR_MAGENTA, "ID 1 origin must be MAGENTA");

        /* user sprites must no longer exist */
        TEST_ASSERT(asset_get_sprite(&atlas, 2) == NULL, "sprite 2 must be unallocated after reset");

        /* 2D packing space must be reclaimed: allocating 55x55 should now succeed */
        asset_slot_t s_reclaim;
        res_t r_reclaim = asset_atlas_alloc_slot(&atlas, vec2i(55, 55), &s_reclaim);
        TEST_ASSERT(r_reclaim == OK, "reclaimed space allocation must succeed, got %d", r_reclaim);
        TEST_ASSERT(s_reclaim.id == ASSET_SPRITE_USER_START, "first slot after reset must get ID 2");
    } TEST_CASE_END();

    TEST_CASE("pivots_and_uv", "updates normalized and pixel pivots correctly") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(100, 100), 16);

        asset_slot_t slot;
        asset_atlas_alloc_slot(&atlas, vec2i(20, 40), &slot);

        /* default pivot should be center (0.5, 0.5) */
        const asset_sprite_t *spr = asset_get_sprite(&atlas, slot.id);
        TEST_ASSERT(feq(spr->pivot.x, 0.5f) && feq(spr->pivot.y, 0.5f), "default pivot must be 0.5, 0.5");

        /* test normalized pivot update */
        asset_sprite_set_pivot(&atlas, slot.id, vec2f(0, 1));
        TEST_ASSERT(feq(spr->pivot.x, 0.0f) && feq(spr->pivot.y, 1.0f), "updated norm pivot (0, 1)");

        /* test slot-based pivot update helper */
        asset_slot_set_pivot(&slot, PIVOT_TOP_LEFT);
        TEST_ASSERT(feq(spr->pivot.x, 0.0f) && feq(spr->pivot.y, 0.0f), "slot set pivot top-left");

        /* test pixel pivot update (5px, 10px on a 20x40 sprite -> (0.25, 0.25)) */
        asset_set_sprite_pivot_px(&atlas, slot.id, vec2f(5, 10));
        TEST_ASSERT(feq(spr->pivot.x, 0.25f) && feq(spr->pivot.y, 0.25f), "pixel pivot exp (0.25, 0.25), got (%f, %f)", spr->pivot.x, spr->pivot.y);

        /* test asset_pivot_from_px fallback on zero/negative dimensions */
        vec2f p_fallback = asset_pivot_from_px(vec2i(0, 0), vec2f(10, 10));
        TEST_ASSERT(feq(p_fallback.x, 0.5f) && feq(p_fallback.y, 0.5f), "zero dimension falls back to center pivot");

        /* test UV coordinates */
        aabb2f uv = asset_get_uv(&atlas, slot.id);
        float exp_u0 = (float)slot.px.min_x / 100.0f;
        float exp_v0 = (float)slot.px.min_y / 100.0f;
        float exp_u1 = (float)slot.px.max_x / 100.0f;
        float exp_v1 = (float)slot.px.max_y / 100.0f;
        TEST_ASSERT(feq(uv.min.x, exp_u0) && feq(uv.min.y, exp_v0), "UV min mismatch");
        TEST_ASSERT(feq(uv.max.x, exp_u1) && feq(uv.max.y, exp_v1), "UV max mismatch");

        /* unallocated sprite ID within max_sprites returns checkerboard UV */
        aabb2f uv_invalid = asset_get_uv(&atlas, atlas.sprite_cnt + 1);
        const asset_sprite_t *checker = asset_get_sprite(&atlas, ASSET_SPRITE_CHECKER);
        TEST_ASSERT(feq(uv_invalid.min.x, checker->uv.min.x), "unallocated ID returns checker UV");

        /* test GPU synchronization resets dirty flag */
        TEST_ASSERT(atlas.dirty == true, "atlas must be dirty before sync");
        res_t r_sync = asset_atlas_sync_gpu(&atlas);
        TEST_ASSERT(r_sync == OK, "gpu sync returned OK");
        TEST_ASSERT(atlas.dirty == false, "atlas must be clean after sync");
    } TEST_CASE_END();

    TEST_CASE("canvas_rect", "draws solid rect and outlined rect") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(64, 64), 16);

        asset_slot_t slot;
        asset_atlas_alloc_slot(&atlas, vec2i(32, 32), &slot);
        asset_slot_clear(&slot, COLOR_CLEAR);

        /* draw filled solid red rect at [4, 4 -> 12, 12] */
        asset_slot_draw_rect(&slot, aabb2i(4, 4, 12, 12), COLOR_RED);

        vec2i inside_pos  = vec2i(slot.px.min_x + 6, slot.px.min_y + 6);
        vec2i outside_pos = vec2i(slot.px.min_x + 2, slot.px.min_y + 2);

        TEST_ASSERT(asset_get_pixel(&atlas, inside_pos) == COLOR_RED, "inside pixel must be red");
        TEST_ASSERT(asset_get_pixel(&atlas, outside_pos) == COLOR_CLEAR, "outside pixel must be clear");

        /* draw green rect outline at [16, 16 -> 28, 28] with thickness 2 */
        asset_slot_draw_rect_outline(&slot, aabb2i(16, 16, 28, 28), 2.0f, COLOR_GREEN);

        /* center of outline should be unpainted (hollow) */
        vec2i center_pos = vec2i(slot.px.min_x + 22, slot.px.min_y + 22);
        TEST_ASSERT(asset_get_pixel(&atlas, center_pos) == COLOR_CLEAR, "center of rect outline should be hollow");

        /* edge of outline should have non-zero green component */
        vec2i edge_pos = vec2i(slot.px.min_x + 16, slot.px.min_y + 22);
        uint32_t edge_col = asset_get_pixel(&atlas, edge_pos);
        TEST_ASSERT(COLOR_G(edge_col) > 0, "outline edge must have green");
    } TEST_CASE_END();

    TEST_CASE("canvas_circle", "draws anti-aliased circle and outline") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(64, 64), 16);

        asset_slot_t slot;
        asset_atlas_alloc_slot(&atlas, vec2i(32, 32), &slot);
        asset_slot_clear(&slot, COLOR_CLEAR);

        /* draw solid circle at center (16, 16) with radius 8 */
        asset_slot_draw_circle(&slot, vec2f(16, 16), 8.0f, COLOR_BLUE);

        vec2i center = vec2i(slot.px.min_x + 16, slot.px.min_y + 16);
        vec2i inner  = vec2i(slot.px.min_x + 16, slot.px.min_y + 20); /* dist = 4 < 8 */
        vec2i outer  = vec2i(slot.px.min_x + 16, slot.px.min_y + 28); /* dist = 12 > 8 */

        TEST_ASSERT(asset_get_pixel(&atlas, center) == COLOR_BLUE, "circle center must be solid blue");
        TEST_ASSERT(asset_get_pixel(&atlas, inner) == COLOR_BLUE,  "circle inner must be solid blue");
        TEST_ASSERT(asset_get_pixel(&atlas, outer) == COLOR_CLEAR, "circle outer must be untouched");

        /* draw circle outline at center (16, 16) with radius 12, thickness 2 */
        asset_slot_draw_circle_outline(&slot, vec2f(16, 16), 12.0f, 2.0f, COLOR_YELLOW);
        vec2i ring = vec2i(slot.px.min_x + 16, slot.px.min_y + 28); /* dist = 12, on ring */
        uint32_t ring_col = asset_get_pixel(&atlas, ring);
        TEST_ASSERT(COLOR_R(ring_col) > 0 && COLOR_G(ring_col) > 0, "ring pixel must be yellow");
    } TEST_CASE_END();

    TEST_CASE("canvas_gradients", "interpolates linear and radial colors") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(64, 64), 16);

        asset_slot_t slot;
        asset_atlas_alloc_slot(&atlas, vec2i(16, 16), &slot);
        asset_slot_clear(&slot, COLOR_CLEAR);

        /* linear gradient from red (top) to blue (bottom) */
        asset_slot_draw_gradient_linear(&slot, aabb2i(0, 0, 16, 16), COLOR_RED, COLOR_BLUE);

        vec2i top_px = vec2i(slot.px.min_x + 8, slot.px.min_y + 0);
        vec2i bot_px = vec2i(slot.px.min_x + 8, slot.px.min_y + 15);
        vec2i mid_px = vec2i(slot.px.min_x + 8, slot.px.min_y + 7);

        uint32_t c_top = asset_get_pixel(&atlas, top_px);
        uint32_t c_bot = asset_get_pixel(&atlas, bot_px);
        uint32_t c_mid = asset_get_pixel(&atlas, mid_px);

        TEST_ASSERT(COLOR_R(c_top) > 200 && COLOR_B(c_top) < 50, "top must be predominantly red");
        TEST_ASSERT(COLOR_B(c_bot) > 200 && COLOR_R(c_bot) < 50, "bottom must be predominantly blue");
        TEST_ASSERT(COLOR_R(c_mid) > 50 && COLOR_B(c_mid) > 50,  "mid must have both red and blue components");

        /* radial gradient from white (center) to clear (outer) with radius 7 at (8.5, 8.5) */
        asset_slot_t slot_rad;
        asset_atlas_alloc_slot(&atlas, vec2i(16, 16), &slot_rad);
        asset_slot_clear(&slot_rad, COLOR_CLEAR);
        asset_slot_draw_gradient_radial(&slot_rad, vec2f(8.5f, 8.5f), 7.0f, COLOR_WHITE, COLOR_CLEAR);

        vec2i rad_center = vec2i(slot_rad.px.min_x + 8, slot_rad.px.min_y + 8);
        vec2i rad_mid    = vec2i(slot_rad.px.min_x + 8, slot_rad.px.min_y + 11); /* dist 3 < 7 */
        vec2i rad_outer  = vec2i(slot_rad.px.min_x + 8, slot_rad.px.min_y + 15); /* dist 6.5 on edge */

        uint32_t c_rad_center = asset_get_pixel(&atlas, rad_center);
        uint32_t c_rad_mid    = asset_get_pixel(&atlas, rad_mid);
        uint32_t c_rad_outer  = asset_get_pixel(&atlas, rad_outer);

        TEST_ASSERT(COLOR_A(c_rad_center) == 255,                        "radial center pixel at exact origin must be opaque white, got alpha %" PRIu32, COLOR_A(c_rad_center));
        TEST_ASSERT(COLOR_A(c_rad_mid) > 50 && COLOR_A(c_rad_mid) < 240, "radial mid must be partially transparent");
        TEST_ASSERT(COLOR_A(c_rad_outer) < 50,                           "radial outer must be near transparent");
    } TEST_CASE_END();

    TEST_CASE("canvas_shapes", "draws rounded rect, triangle, line, and bezier") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(128, 128), 16);

        asset_slot_t slot;
        asset_atlas_alloc_slot(&atlas, vec2i(64, 64), &slot);
        asset_slot_clear(&slot, COLOR_CLEAR);

        /* draw rounded rect [0, 0 -> 20, 20] with radius 6 */
        asset_slot_draw_rect_rounded(&slot, aabb2i(0, 0, 20, 20), 6.0f, COLOR_MAGENTA);
        vec2i r_center = vec2i(slot.px.min_x + 10, slot.px.min_y + 10);
        vec2i r_corner = vec2i(slot.px.min_x + 0,  slot.px.min_y + 0); /* clipped by radius */
        TEST_ASSERT(asset_get_pixel(&atlas, r_center) == COLOR_MAGENTA, "rounded rect center must be magenta");
        TEST_ASSERT(asset_get_pixel(&atlas, r_corner) == COLOR_CLEAR,   "rounded rect sharp corner must be clear");

        /* draw triangle with vertices at (30, 10), (50, 10), (40, 30) */
        asset_slot_draw_triangle(&slot, vec2f(30, 10), vec2f(50, 10), vec2f(40, 30), COLOR_CYAN);
        vec2i tri_centroid = vec2i(slot.px.min_x + 40, slot.px.min_y + 16);
        vec2i tri_outside  = vec2i(slot.px.min_x + 30, slot.px.min_y + 28);
        TEST_ASSERT(asset_get_pixel(&atlas, tri_centroid) == COLOR_CYAN, "triangle centroid must be cyan");
        TEST_ASSERT(asset_get_pixel(&atlas, tri_outside) == COLOR_CLEAR, "point outside triangle must be clear");

        /* draw line from (0, 40) to (30, 40) with thickness 2 */
        asset_slot_draw_line(&slot, vec2f(0, 40), vec2f(30, 40), 2.0f, COLOR_WHITE);
        vec2i line_mid = vec2i(slot.px.min_x + 15, slot.px.min_y + 40);
        TEST_ASSERT(asset_get_pixel(&atlas, line_mid) == COLOR_WHITE, "line mid must be white");

        /* draw quadratic bezier */
        asset_slot_draw_bezier(&slot, vec2f(0, 50), vec2f(15, 60), vec2f(30, 50), 2.0f, COLOR_YELLOW);
        vec2i bez_mid = vec2i(slot.px.min_x + 15, slot.px.min_y + 55);
        uint32_t bez_col = asset_get_pixel(&atlas, bez_mid);
        TEST_ASSERT(COLOR_A(bez_col) > 0, "bezier curve midpoint must be drawn");
    } TEST_CASE_END();

    TEST_CASE("antialiasing_toggle", "verifies 1-bit thresholding vs anti-aliasing") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(64, 64), 16);

        asset_slot_t slot_aa;
        asset_atlas_alloc_slot(&atlas, vec2i(32, 32), &slot_aa);
        asset_slot_clear(&slot_aa, COLOR_CLEAR);
        slot_aa.antialiased = true;
        asset_slot_draw_circle(&slot_aa, vec2f(16, 16), 8.0f, COLOR_WHITE);

        asset_slot_t slot_1bit;
        asset_atlas_alloc_slot(&atlas, vec2i(32, 32), &slot_1bit);
        asset_slot_clear(&slot_1bit, COLOR_CLEAR);
        asset_slot_set_antialiased(&slot_1bit, false);
        asset_slot_draw_circle(&slot_1bit, vec2f(16, 16), 8.0f, COLOR_WHITE);

        /* scan all pixels of 1-bit slot: alpha must be ONLY 0 or 255 */
        bool only_binary_alpha = true;
        for (int y = 0; y < 32; ++y) {
            for (int x = 0; x < 32; ++x) {
                uint32_t c = asset_get_pixel(&atlas, vec2i(slot_1bit.px.min_x + x, slot_1bit.px.min_y + y));
                uint32_t a = COLOR_A(c);
                if (a != 0 && a != 255) {
                    only_binary_alpha = false;
                    break;
                }
            }
        }
        TEST_ASSERT(only_binary_alpha, "1-bit mode must contain only binary alpha (0 or 255)");
    } TEST_CASE_END();

    TEST_CASE("text_pixel_sprite", "bakes ASCII string with PICO-8 palette") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(64, 64), 16);

        const char *heart =
            "..8.8..\n"
            ".88888.\n"
            ".88888.\n"
            "..888..\n"
            "...8...\n";

        asset_sprite_id_t heart_id;
        res_t r = asset_bake_text_sprite(&atlas, heart, vec2i(7, 5), NULL, &heart_id);
        TEST_ASSERT(r == OK, "baking text sprite failed: %d", r);

        const asset_sprite_t *spr = asset_get_sprite(&atlas, heart_id);
        TEST_ASSERT(spr != NULL, "heart sprite must exist");
        vec2i sz = aabb_size(spr->px);
        TEST_ASSERT(sz.x == 7 && sz.y == 5, "heart size exp 7x5, got %dx%d", sz.x, sz.y);

        /* pixel (0, 0) is '.', should be clear */
        vec2i p_0_0 = vec2i(spr->px.min_x + 0, spr->px.min_y + 0);
        TEST_ASSERT(asset_get_pixel(&atlas, p_0_0) == COLOR_CLEAR, "pixel (0,0) must be transparent");

        /* pixel (2, 0) is '8' (red in PICO-8: index 8) */
        vec2i p_2_0 = vec2i(spr->px.min_x + 2, spr->px.min_y + 0);
        TEST_ASSERT(asset_get_pixel(&atlas, p_2_0) == g_palette_pico8[8], "pixel (2,0) must match PICO-8 red");
    } TEST_CASE_END();

    TEST_CASE("text_pixel_anim", "bakes multiple frames into contiguous IDs") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(64, 64), 16);

        const char *frames[3] = {
            "88\n88\n", /* frame 0: 2x2 red */
            "99\n99\n", /* frame 1: 2x2 orange */
            "AA\nAA\n", /* frame 2: 2x2 yellow */
        };

        asset_sprite_id_t base_id;
        res_t r = asset_bake_text_animation(&atlas, frames, 3, vec2i(2, 2), NULL, &base_id);
        TEST_ASSERT(r == OK, "animation baking failed: %d", r);

        /* verify consecutive frame IDs */
        for (uint32_t i = 0; i < 3; ++i) {
            asset_sprite_id_t f_id = base_id + i;
            const asset_sprite_t *f_spr = asset_get_sprite(&atlas, f_id);
            TEST_ASSERT(f_spr != NULL, "frame %" PRIu32 " must exist", i);

            uint32_t p0 = asset_get_pixel(&atlas, vec2i(f_spr->px.min_x, f_spr->px.min_y));
            uint32_t exp_col = (i == 0) ? g_palette_pico8[8] :
                               (i == 1) ? g_palette_pico8[9] :
                                          g_palette_pico8[10];
            TEST_ASSERT(p0 == exp_col, "frame %" PRIu32 " color mismatch", i);
        }
    } TEST_CASE_END();

    TEST_CASE("image_load_mem", "decodes dynamic in-memory PNG and validates pixels") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(64, 64), 16);

        /* 4x2 test image with distinct RGBA colors */
        uint32_t raw_pixels[8] = {
            COLOR_RED,    COLOR_GREEN,   COLOR_YELLOW,  COLOR_WHITE,
            COLOR_MAGENTA, COLOR_BLACK,  COLOR_RED,     COLOR_BLUE,
        };

        int png_len = 0;
        unsigned char *png_data = stbi_write_png_to_mem(
            (const unsigned char *)raw_pixels,
            4 * (int)sizeof(uint32_t),
            4, 2, 4,
            &png_len
        );
        TEST_ASSERT(png_data != NULL && png_len > 0, "PNG encoding to memory failed");

        asset_sprite_id_t img_id;
        res_t r = asset_load_image_mem(&atlas, png_data, (size_t)png_len, &img_id);
        TEST_ASSERT(r == OK, "asset_load_image_mem failed: %d", r);

        const asset_sprite_t *spr = asset_get_sprite(&atlas, img_id);
        TEST_ASSERT(spr != NULL, "loaded sprite must exist");
        vec2i sz = aabb_size(spr->px);
        TEST_ASSERT(sz.x == 4 && sz.y == 2, "sprite size mismatch: exp 4x2, got %dx%d", sz.x, sz.y);

        /* check all 8 pixels */
        for (int32_t y = 0; y < 2; ++y) {
            for (int32_t x = 0; x < 4; ++x) {
                vec2i pos = vec2i(spr->px.min_x + x, spr->px.min_y + y);
                uint32_t c = asset_get_pixel(&atlas, pos);
                uint32_t exp = raw_pixels[y * 4 + x];
                TEST_ASSERT(c == exp, "pixel (%d,%d) exp 0x%08X, got 0x%08X", x, y, exp, c);
            }
        }

        /* test invalid arguments */
        uint8_t garbage[16] = {1, 2, 3, 4, 5, 6, 7, 8};
        asset_sprite_id_t bad_id;
        TEST_ASSERT(asset_load_image_mem(&atlas, garbage, sizeof(garbage), &bad_id) == ENOTFND, "corrupt buffer returns ENOTFND");

        free(png_data);
    } TEST_CASE_END();

    TEST_CASE("register_grid", "slices spritesheet into sub-sprites") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(128, 128), 32);

        /* create a 32x16 parent sprite */
        asset_slot_t slot;
        asset_atlas_alloc_slot(&atlas, vec2i(32, 16), &slot);

        /* fill 4 cells of 16x8 with distinct colors */
        asset_slot_draw_rect(&slot, aabb2i(0,  0, 16,  8), COLOR_RED);
        asset_slot_draw_rect(&slot, aabb2i(16, 0, 32,  8), COLOR_GREEN);
        asset_slot_draw_rect(&slot, aabb2i(0,  8, 16, 16), COLOR_BLUE);
        asset_slot_draw_rect(&slot, aabb2i(16, 8, 32, 16), COLOR_YELLOW);

        asset_sprite_id_t sub_ids[4];
        uint32_t out_cnt = 0;
        res_t r = asset_register_grid(&atlas, slot.id, vec2i(16, 8), sub_ids, 4, &out_cnt);
        TEST_ASSERT(r == OK, "register_grid failed: %d", r);
        TEST_ASSERT(out_cnt == 4, "expected 4 sub-sprites, got %" PRIu32, out_cnt);

        /* verify sub-sprite dimensions and top-left pixel colors */
        uint32_t exp_colors[4] = { COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW };
        for (uint32_t i = 0; i < 4; ++i) {
            const asset_sprite_t *sub = asset_get_sprite(&atlas, sub_ids[i]);
            TEST_ASSERT(sub != NULL, "sub-sprite %" PRIu32 " must exist", i);

            vec2i sub_sz = aabb_size(sub->px);
            TEST_ASSERT(sub_sz.x == 16 && sub_sz.y == 8, "sub-sprite %" PRIu32 " size exp 16x8, got %dx%d", i, sub_sz.x, sub_sz.y);

            uint32_t c = asset_get_pixel(&atlas, sub->px.min);
            TEST_ASSERT(c == exp_colors[i], "sub-sprite %" PRIu32 " pixel exp 0x%08X, got 0x%08X", i, exp_colors[i], c);
        }

        /* test invalid cell size (larger than source) */
        uint32_t dummy_cnt = 0;
        asset_sprite_id_t dummy_id;
        res_t r_inv = asset_register_grid(&atlas, slot.id, vec2i(64, 64), &dummy_id, 1, &dummy_cnt);
        TEST_ASSERT(r_inv == EINVAL, "cell size larger than source must return EINVAL");
    } TEST_CASE_END();

    TEST_CASE("hot_reload_mem", "updates sprite pixels in-place") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(64, 64), 16);

        uint32_t original[4] = { COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED };
        int len1 = 0;
        unsigned char *png1 = stbi_write_png_to_mem((const unsigned char *)original, 2 * (int)sizeof(uint32_t), 2, 2, 4, &len1);

        asset_sprite_id_t id;
        asset_load_image_mem(&atlas, png1, (size_t)len1, &id);
        free(png1);

        const asset_sprite_t *spr_before = asset_get_sprite(&atlas, id);
        aabb2i px_before = spr_before->px;

        /* reload with blue pixels */
        uint32_t replacement[4] = { COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE };
        int len2 = 0;
        unsigned char *png2 = stbi_write_png_to_mem((const unsigned char *)replacement, 2 * (int)sizeof(uint32_t), 2, 2, 4, &len2);

        atlas.dirty = false;
        res_t r = asset_reload_image_mem(&atlas, id, png2, (size_t)len2);
        free(png2);

        TEST_ASSERT(r == OK, "reload failed: %d", r);
        TEST_ASSERT(atlas.dirty == true, "reload must mark atlas dirty");

        const asset_sprite_t *spr_after = asset_get_sprite(&atlas, id);
        TEST_ASSERT(spr_after->px.min_x == px_before.min_x && spr_after->px.min_y == px_before.min_y, "rect must not move");
        TEST_ASSERT(asset_get_pixel(&atlas, spr_after->px.min) == COLOR_BLUE, "pixel must be updated to blue");

        /* reload with mismatched dimensions (4x4 instead of 2x2) must fail */
        uint32_t wrong_size[16] = {0};
        int len3 = 0;
        unsigned char *png3 = stbi_write_png_to_mem((const unsigned char *)wrong_size, 4 * (int)sizeof(uint32_t), 4, 4, 4, &len3);
        res_t r_bad_sz = asset_reload_image_mem(&atlas, id, png3, (size_t)len3);
        free(png3);
        TEST_ASSERT(r_bad_sz == EINVAL, "mismatched dimension reload must return EINVAL");
    } TEST_CASE_END();

    TEST_CASE("palette_remap", "replaces colors and palettes across atlas") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(32, 32), 16);

        asset_slot_t slot;
        asset_atlas_alloc_slot(&atlas, vec2i(16, 16), &slot);

        /* draw red and green halves */
        asset_slot_draw_rect(&slot, aabb2i(0, 0, 8, 16), COLOR_RED);
        asset_slot_draw_rect(&slot, aabb2i(8, 0, 16, 16), COLOR_GREEN);

        vec2i p_red   = slot.px.min;
        vec2i p_green = vec2i(slot.px.min_x + 10, slot.px.min_y);

        TEST_ASSERT(asset_get_pixel(&atlas, p_red) == COLOR_RED, "initial red check");
        TEST_ASSERT(asset_get_pixel(&atlas, p_green) == COLOR_GREEN, "initial green check");

        /* single color remap: red -> yellow */
        atlas.dirty = false;
        res_t r_col = asset_atlas_remap_color(&atlas, COLOR_RED, COLOR_YELLOW);
        TEST_ASSERT(r_col == OK, "remap_color returned OK");
        TEST_ASSERT(atlas.dirty == true, "remap_color set dirty");
        TEST_ASSERT(asset_get_pixel(&atlas, p_red) == COLOR_YELLOW, "red must become yellow");
        TEST_ASSERT(asset_get_pixel(&atlas, p_green) == COLOR_GREEN, "green must stay green");

        /* palette remap: yellow -> magenta, green -> cyan */
        uint32_t src_pal[2] = { COLOR_YELLOW, COLOR_GREEN };
        uint32_t dst_pal[2] = { COLOR_MAGENTA, COLOR_CYAN };
        atlas.dirty = false;
        res_t r_pal = asset_atlas_remap_palette(&atlas, src_pal, dst_pal, 2);
        TEST_ASSERT(r_pal == OK, "remap_palette returned OK");
        TEST_ASSERT(atlas.dirty == true, "remap_palette set dirty");
        TEST_ASSERT(asset_get_pixel(&atlas, p_red) == COLOR_MAGENTA, "yellow must become magenta");
        TEST_ASSERT(asset_get_pixel(&atlas, p_green) == COLOR_CYAN, "green must become cyan");
    } TEST_CASE_END();

    TEST_CASE("disk_io", "exports sprite/atlas and loads from file") {
        arena_init(&perm, arena_buf, sizeof(arena_buf));

        asset_atlas_t atlas;
        asset_atlas_init(&atlas, &perm, vec2i(32, 32), 16);

        asset_slot_t slot;
        asset_atlas_alloc_slot(&atlas, vec2i(4, 2), &slot);

        /* row 0: red, row 1: green */
        asset_slot_draw_rect(&slot, aabb2i(0, 0, 4, 1), COLOR_RED);
        asset_slot_draw_rect(&slot, aabb2i(0, 1, 4, 2), COLOR_GREEN);

        const char *tmp_sprite_path = "/tmp/test_bonsai_sprite.png";
        const char *tmp_atlas_path  = "/tmp/test_bonsai_atlas.png";
        unlink(tmp_sprite_path);
        unlink(tmp_atlas_path);

        /* export sprite to PNG */
        res_t r_exp = asset_export_sprite_png(&atlas, slot.id, tmp_sprite_path, true);
        TEST_ASSERT(r_exp == OK, "export_sprite_png failed: %d", r_exp);
        TEST_ASSERT(access(tmp_sprite_path, F_OK) == 0, "exported sprite file must exist");

        /* export without overwrite should return EIO when file already exists */
        res_t r_no_ow = asset_export_sprite_png(&atlas, slot.id, tmp_sprite_path, false);
        TEST_ASSERT(r_no_ow == EIO, "overwrite=false on existing file must return EIO, got %d", r_no_ow);

        /* export master atlas to PNG */
        res_t r_atlas_exp = asset_export_atlas_png(&atlas, tmp_atlas_path, true);
        TEST_ASSERT(r_atlas_exp == OK, "export_atlas_png failed: %d", r_atlas_exp);
        TEST_ASSERT(access(tmp_atlas_path, F_OK) == 0, "exported atlas file must exist");

        /* test loading exported non-square sprite back into atlas */
        asset_sprite_id_t loaded_id;
        res_t r_load = asset_load_image_file(&atlas, tmp_sprite_path, &loaded_id);
        TEST_ASSERT(r_load == OK, "asset_load_image_file failed: %d", r_load);

        const asset_sprite_t *loaded_spr = asset_get_sprite(&atlas, loaded_id);
        TEST_ASSERT(loaded_spr != NULL, "loaded sprite must exist");
        vec2i loaded_sz = aabb_size(loaded_spr->px);
        TEST_ASSERT(loaded_sz.x == 4 && loaded_sz.y == 2, "loaded size mismatch: exp 4x2, got %dx%d", loaded_sz.x, loaded_sz.y);
        TEST_ASSERT(asset_get_pixel(&atlas, vec2i(loaded_spr->px.min_x, loaded_spr->px.min_y + 0)) == COLOR_RED,   "row 0 must be red");
        TEST_ASSERT(asset_get_pixel(&atlas, vec2i(loaded_spr->px.min_x, loaded_spr->px.min_y + 1)) == COLOR_GREEN, "row 1 must be green");

        /* test reload from file */
        res_t r_reload = asset_reload_image_file(&atlas, loaded_id, tmp_sprite_path);
        TEST_ASSERT(r_reload == OK, "asset_reload_image_file failed: %d", r_reload);

        /* cleanup temporary files */
        unlink(tmp_sprite_path);
        unlink(tmp_atlas_path);
    } TEST_CASE_END();

    TEST_END();
}
