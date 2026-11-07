#pragma once

/*
====================
arena.h

Contiguous linear bump allocator aligned to 64-byte cache lines.
Provides O(1) allocation without fragmentation, strict arena ownership,
and scoped checkpoint unwinds via `arena_temp_t`.
====================
*/

#include "core.h"

#include <stdbool.h>
#include <stddef.h>

/* ============================================================================================== */
/* >TYPES                                                                                         */
/* ============================================================================================== */

/*
====================
arena_t

Linear bump allocator state over an externally owned memory buffer `buf`.
Backing memory is passed via `arena_init`. Allocations are aligned to 64-byte
cache lines. Offsets monotonically increase until explicitly reset or unwound.
====================
*/
typedef struct arena {
    void  *buf;
    size_t cap;
    size_t off;
} arena_t;

/*
====================
arena_temp_t

RAII-style rollback checkpoint for temporary scratch allocations.
Stores the previous allocation offset of an arena. Passing this struct to
`arena_temp_end` restores the offset, reclaiming all intervening allocations.
====================
*/
typedef struct arena_temp {
    arena_t *arena;
    size_t   prev_off;
} arena_temp_t;

/* ============================================================================================== */
/* >LIFECYCLE                                                                                     */
/* ============================================================================================== */

/*
====================
arena_reset

Rewinds the arena allocation offset `a->off` back to 0.
Invalidates all previously allocated memory without zeroing or releasing
the underlying buffer.
====================
*/
static inline void arena_reset(arena_t *a) {
    ASSERT(a, "Attempted reset on NULL arena");
    a->off = 0;
}

/*
====================
arena_init

Initializes a bump allocator over a user-supplied backing buffer.
Aligns `buf` upwards to `CACHE_LINE_SZ` (64 bytes) and deducts alignment
padding from usable capacity. Does not take buffer ownership.

Returns:
- `OK`     on successful initialization with usable capacity.
- `EINVAL` if `a` or `buf` are NULL.
- `WEMPTY` if `cap` is smaller than the required cache-line alignment padding.
====================
*/
static inline res_t arena_init(arena_t *a, void *buf, size_t cap) {
    ASSERT(a,   "Attempted init on NULL arena (buf: %p, cap: %zu)",          buf, cap);
    ASSERT(buf, "Attempted init arena with NULL buffer (a: %p, cap: %zu)",   a,   cap);
    if (!a || !buf) return EINVAL;

    uintptr_t raw     = (uintptr_t)buf;
    uintptr_t aligned = (uintptr_t)ALIGN_LINE_PTR(buf);
    size_t pad        = (size_t)(aligned - raw);

    a->buf = (void *)aligned;
    a->cap = (cap > pad) ? (cap - pad) : 0;
    arena_reset(a);

    return a->cap ? OK : WEMPTY;
}

/* ============================================================================================== */
/* >ALLOCATION                                                                                    */
/* ============================================================================================== */

/*
====================
arena_alloc

Bumps the arena offset to allocate `sz` bytes.
Both offset and size are rounded up to `CACHE_LINE_SZ` (64 bytes).

Returns:
- a 64-byte cache-line aligned pointer to the allocated block on success,
- NULL if capacity `a->cap` is exceeded.
====================
*/
static inline void *arena_alloc(arena_t *a, size_t sz) {
    ASSERT(a, "Attempted allocation on NULL arena (sz: %zu)", sz);

    size_t aligned_off = ALIGN_LINE_INT(a->off);
    size_t aligned_sz  = ALIGN_LINE_INT(sz);

    ASSERT(aligned_off + aligned_sz <= a->cap, "Arena ran out of memory (a: %p, sz: %zu)", a, sz);
    if (aligned_off + aligned_sz > a->cap) return NULL;

    void *ret_ptr = (void *)((uintptr_t)a->buf + aligned_off);
    a->off        = aligned_off + aligned_sz;
    return ret_ptr;
}

/*
====================
arena_has

Checks if the half-open range `[ptr, ptr + sz)` resides entirely
within the currently active (allocated) region of the arena.

Returns:
- `true`, if fully enclosed within active bounds.
- `false` otherwise.
====================
*/
static inline bool arena_has(arena_t *a, void *ptr, size_t sz) {
    ASSERT(a,   "Attempted has check on NULL arena (ptr: %p, sz: %zu)",   ptr, sz);
    ASSERT(ptr, "Attempted has check on NULL ptr   (arena: %p, sz: %zu)", a,   sz);
    if (!a || !ptr) return false;
    return (uintptr_t)ptr       >= (uintptr_t)(a->buf) &&
          ((uintptr_t)ptr + sz) <= ((uintptr_t)a->buf + a->off);
}

/* ============================================================================================== */
/* >SCRATCH UNWINDING                                                                             */
/* ============================================================================================== */

/*
====================
arena_temp_begin

Captures an allocation checkpoint on `a` for scoped scratch memory.
Pair with `arena_temp_end` in strict LIFO order to rewind temporary
allocations.

Returns an `arena_temp_t` checkpoint handle.
====================
*/
static inline arena_temp_t arena_temp_begin(arena_t *a) {
    ASSERT(a, "Attempted temp begin on NULL arena");
    return (arena_temp_t){ .arena = a, .prev_off = a->off };
}

/*
====================
arena_temp_end

Rewinds the parent arena's allocation offset back to the
checkpoint saved in `a_tmp`. Reclaims all memory allocated
since the matching `arena_temp_begin`, reverting `a_tmp.arena->off`.
====================
*/
static inline void arena_temp_end(arena_temp_t a_tmp) {
    ASSERT(a_tmp.arena,                        "Attempted temp end on NULL arena");
    ASSERT(a_tmp.arena->off >= a_tmp.prev_off, "Arena corruption detected during temp unwind");
    a_tmp.arena->off = a_tmp.prev_off;
}
