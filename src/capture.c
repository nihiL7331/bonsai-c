#include "capture.h"
#include "backend/cpu.h"

#include <limits.h>
#include <string.h>

/* ============================================================================================== */
/* >CONSTANTS AND TYPES                                                                           */
/* ============================================================================================== */

#define CAPTURE_PNG_TYPE_SZ        4U
#define CAPTURE_PNG_IHDR_SZ        13U
#define CAPTURE_PNG_CHAN_CNT       4U
#define CAPTURE_PNG_BIT_DEPTH      8U
#define CAPTURE_PNG_RGBA_TYPE      6U
#define CAPTURE_PNG_COMP_DEFLATE   0U
#define CAPTURE_PNG_FILTER_ADAPT   0U
#define CAPTURE_PNG_FILTER_NONE    0U
#define CAPTURE_PNG_INTERLACE_NONE 0U
#define CAPTURE_PNG_CHUNK_MAX      INT32_MAX

/* it writes only IHDR, IDAT, and IEND                                */
/* every chunk has a length, type, and CRC in addition to its payload */
#define CAPTURE_PNG_CHUNK_CNT      3U
#define CAPTURE_PNG_CHUNK_OVERHEAD (2U * sizeof(uint32_t) + CAPTURE_PNG_TYPE_SZ)

/* stored DEFLATE blocks contain a control byte, LEN, and 16-bit complement */
#define CAPTURE_DEFLATE_BLOCK_MAX UINT16_MAX
#define CAPTURE_DEFLATE_HEADER_SZ (1U + 2U * sizeof(uint16_t))
#define CAPTURE_DEFLATE_MORE      0x00U
#define CAPTURE_DEFLATE_FINAL     0x01U

/* zlib header: DEFLATE with 32kb window, no dictionary, valid FCHECK bits */
#define CAPTURE_ZLIB_CMF      0x78U
#define CAPTURE_ZLIB_FLG      0x01U
#define CAPTURE_ZLIB_OVERHEAD (2U + sizeof(uint32_t))
#define CAPTURE_ADLER32_MOD   65521U

/* IEEE CRC-32 polynomial used by PNG chunks */
#define CAPTURE_CRC32_POLYNOM 0xEDB88320U

/* PNG signature: high-bit marker, PNG, CR/LF, DOS EOF marker, LF */
static const uint8_t s_capture_png_sign[] = {
    0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'
};

res_t capture_render(const render_batch_t *batch, const asset_atlas_t *atlas, arena_t *scratch,
                     uint32_t clear, capture_frame_t *out_frame) {
    if (!out_frame) return EINVAL;
    *out_frame = (capture_frame_t){0};

    if (!batch || !scratch || !scratch->buf || scratch->off > scratch->cap ||
        batch->viewport.x <= 0 || batch->viewport.y <= 0 ||
        batch->light_cnt > RENDER_MAX_LIGHTS ||
        (batch->cmd3d_cnt && !batch->cmds3d) || (batch->sprite2d_cnt && !batch->sprites2d)) return EINVAL;

    if ((batch->cmd3d_cnt || batch->sprite2d_cnt) &&
        (!atlas || !atlas->pixels || atlas->size.x <= 0 || atlas->size.y <= 0)) return EINVAL;

    size_t wid = (size_t)batch->viewport.x, hei = (size_t)batch->viewport.y;
    size_t cnt = wid * hei;
    if (wid > SIZE_MAX / hei || cnt > SIZE_MAX / sizeof(uint32_t)) return EINVAL;

    arena_temp_t    mark  = arena_temp_begin(scratch);
    capture_frame_t frame = { .size = batch->viewport };

    frame.pixels = arena_alloc(scratch, cnt * sizeof(*frame.pixels));
    if (!frame.pixels) { arena_temp_end(mark); return EOOM; }

    if (batch->cmd3d_cnt) {
        frame.depth = arena_alloc(scratch, cnt * sizeof(*frame.depth));
        if (!frame.depth) { arena_temp_end(mark); return EOOM; }
    }

    backend_cpu_target_t target = {
        .pixels                = frame.pixels,
        .depth                 = frame.depth,
        .size                  = frame.size,
        .stride_pixels         = wid,
        .capacity_pixels       = cnt,
        .depth_stride_pixels   = frame.depth ? wid : 0,
        .depth_capacity_pixels = frame.depth ? cnt : 0,
    };

    res_t res = backend_cpu_render(batch, atlas, &target, clear);
    if (res != OK) { arena_temp_end(mark); return res; }

    *out_frame = frame;
    return OK;
}

/* ============================================================================================== */
/* >PNG ENCODING                                                                                  */
/* ============================================================================================== */

static inline void _capture_write_u32_be(uint8_t *out, uint32_t val) {
    for (size_t i = 0; i < sizeof(val); ++i) out[i] = (uint8_t)(val >> ((sizeof(val) - 1 - i) * CHAR_BIT));
}

static inline void _capture_write_u16_le(uint8_t *out, uint16_t val) {
    for (size_t i = 0; i < sizeof(val); ++i) out[i] = (uint8_t)(val >> (i * CHAR_BIT));
}

/* writes a PNG chunk, including CRC, and returns the byte following it */
static uint8_t *_capture_write_png_chunk(uint8_t *out, const char type[CAPTURE_PNG_TYPE_SZ], const uint8_t *data, uint32_t sz) {
    _capture_write_u32_be(out, sz);

    uint8_t *type_out = out + sizeof(uint32_t);
    memcpy(type_out, type, CAPTURE_PNG_TYPE_SZ);

    uint8_t *payload_out = type_out + CAPTURE_PNG_TYPE_SZ;
    if (sz && data != payload_out) memcpy(payload_out, data, sz);

    uint32_t crc = UINT32_MAX;

    for (size_t i = 0; i < (size_t)sz + CAPTURE_PNG_TYPE_SZ; ++i) {
        crc ^= type_out[i];

        for (uint32_t bit = 0; bit < CHAR_BIT; ++bit)
            crc = (crc >> 1) ^ (CAPTURE_CRC32_POLYNOM & (-(crc & 1U)));
    }

    _capture_write_u32_be(payload_out + sz, ~crc);
    return out + CAPTURE_PNG_CHUNK_OVERHEAD + sz;
}

res_t capture_encode_png(const image_view_t *image, arena_t *scratch, uint8_t **out_bytes, size_t *out_size) {
    if (out_bytes) *out_bytes = NULL;
    if (out_size)  *out_size  = 0;

    if (!out_bytes || !out_size || !image_view_is_valid(image) ||
        !scratch || !scratch->buf || scratch->off > scratch->cap) return EINVAL;

    size_t wid = (size_t)image->size.x, hei = (size_t)image->size.y;
    if    (wid > (SIZE_MAX - 1) / CAPTURE_PNG_CHAN_CNT) return EINVAL;

    /* each scanline starts with one filter byte, followed by RGBA chan bytes */
    size_t row_bytes = wid * CAPTURE_PNG_CHAN_CNT + 1;
    size_t raw_sz    = row_bytes * hei;
    if (hei > SIZE_MAX / row_bytes || raw_sz > CAPTURE_PNG_CHUNK_MAX) return EINVAL;

    size_t block_cnt  = (raw_sz - 1) / CAPTURE_DEFLATE_BLOCK_MAX + 1;
    size_t payload_sz = raw_sz + block_cnt * CAPTURE_DEFLATE_HEADER_SZ + CAPTURE_ZLIB_OVERHEAD;
    if    (payload_sz > CAPTURE_PNG_CHUNK_MAX) return EINVAL;

    size_t png_sz = sizeof(s_capture_png_sign) + CAPTURE_PNG_IHDR_SZ +
                    CAPTURE_PNG_CHUNK_CNT * CAPTURE_PNG_CHUNK_OVERHEAD + payload_sz;

    uint8_t *png = arena_alloc(scratch, png_sz);
    if    (!png) return EOOM;

    memcpy(png, s_capture_png_sign, sizeof(s_capture_png_sign));

    uint8_t  ihdr[CAPTURE_PNG_IHDR_SZ];
    uint8_t *field = ihdr;

    _capture_write_u32_be(field, (uint32_t)wid);
    field += sizeof(uint32_t);
    _capture_write_u32_be(field, (uint32_t)hei);
    field += sizeof(uint32_t);

    *field++ = CAPTURE_PNG_BIT_DEPTH;
    *field++ = CAPTURE_PNG_RGBA_TYPE;
    *field++ = CAPTURE_PNG_COMP_DEFLATE;
    *field++ = CAPTURE_PNG_FILTER_ADAPT;
    *field   = CAPTURE_PNG_INTERLACE_NONE;

    uint8_t *chunk = _capture_write_png_chunk(png + sizeof(s_capture_png_sign), "IHDR", ihdr, sizeof(ihdr));
    uint8_t *dst   = chunk + sizeof(uint32_t) + CAPTURE_PNG_TYPE_SZ;

    *dst++ = CAPTURE_ZLIB_CMF;
    *dst++ = CAPTURE_ZLIB_FLG;

    /* adler-32 starts at (a = 1, b = 0) and inclues filter bytes as well as pixel bytes */
    uint32_t adler_a = 1, adler_b = 0;
    size_t   pos     = 0;

    while (pos < raw_sz) {
        size_t len = MIN(raw_sz - pos, CAPTURE_DEFLATE_BLOCK_MAX);

        *dst++ = pos + len == raw_sz ? CAPTURE_DEFLATE_FINAL : CAPTURE_DEFLATE_MORE;

        _capture_write_u16_le(dst, (uint16_t)len);
        dst += sizeof(uint16_t);
        _capture_write_u16_le(dst, (uint16_t)~(uint16_t)len);
        dst += sizeof(uint16_t);

        for (size_t i = 0; i < len; ++i, ++pos) {
            size_t  row_off = pos % row_bytes;
            uint8_t byte    = CAPTURE_PNG_FILTER_NONE;

            if (row_off) {
                size_t px    = (pos / row_bytes) * wid + (row_off - 1) / CAPTURE_PNG_CHAN_CNT;
                size_t chan  = (row_off - 1) % CAPTURE_PNG_CHAN_CNT;
                size_t shift = (CAPTURE_PNG_CHAN_CNT - 1 - chan) * CHAR_BIT;

                byte = (uint8_t)(image->pixels[px] >> shift);
            }

            *dst++ = byte;
            adler_a = (adler_a + byte)    % CAPTURE_ADLER32_MOD;
            adler_b = (adler_b + adler_a) % CAPTURE_ADLER32_MOD;
        }
    }

    _capture_write_u32_be(dst, (adler_b << (sizeof(uint16_t) * CHAR_BIT)) | adler_a);

    uint8_t *payload = chunk + sizeof(uint32_t) + CAPTURE_PNG_TYPE_SZ;
    chunk = _capture_write_png_chunk(chunk, "IDAT", payload, (uint32_t)payload_sz);
    chunk = _capture_write_png_chunk(chunk, "IEND", NULL,    0);

    *out_bytes = png;
    *out_size  = png_sz;

    return OK;
}
