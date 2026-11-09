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

#ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif /* !WIN32_LEAN_AND_MEAN */
    #include <windows.h>
#else  /* !defined(_WIN32) */
    #include <sys/mman.h>
    #ifndef MAP_ANONYMOUS
        #define MAP_ANONYMOUS MAP_ANON
    #endif /* !defined(MAP_ANONYMOUS) */
    #ifndef MAP_NORESERVE
        #define MAP_NORESERVE 0
    #endif /* !defined(MAP_NORESERVE) */
#endif /* !defined(_WIN32) */

#define ARENA_DEF_VM_CAP GB(1)

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
    ASSERT(buf, "Attempted init arena with NULL buffer (a: %p, cap: %zu)",   (void *)a,   cap);
    if (!a || !buf) return EINVAL;

    uintptr_t raw     = (uintptr_t)buf;
    uintptr_t aligned = (uintptr_t)ALIGN_LINE_PTR(buf);
    size_t pad        = (size_t)(aligned - raw);

    a->buf = (void *)aligned;
    a->cap = (cap > pad) ? (cap - pad) : 0;
    arena_reset(a);

    return a->cap ? OK : WEMPTY;
}

/*
====================
arena_init_vm

Initializes a demand-paged virtual-memory arena of capacity `cap`.
Reserves contiguous virtual address space via OS `mmap` on POSIX
or `VirtualAlloc` on Windows.
If `cap == 0`, defaults to `ARENA_DEF_VM_CAP` (1 GB).
Physical 4 KB (or 16 KB on macOS ARM) RAM pages are faulted in
by the OS kernel only when written.
Pair with `arena_free_vm` to release the virtual address range.

Returns:
- `OK`     on successful virtual memory reservation.
- `EINVAL` if `a` is `NULL`.
- `EOOM`   if OS virtual memory reservation fails.
====================
*/
static inline res_t arena_init_vm(arena_t *a, size_t cap) {
    ASSERT(a, "Attempted VM init on NULL arena (cap: %zu)", cap);
    if   (!a) return EINVAL;

    size_t aligned_cap = ALIGN_LINE_INT(cap ? cap : ARENA_DEF_VM_CAP);
#ifdef _WIN32
    void *buf = VirtualAlloc(NULL, aligned_cap, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    if  (!buf) return EOOM;
#else  /* !defined(_WIN32) */
    void  *buf = mmap(NULL, aligned_cap, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    if    (buf == MAP_FAILED) return EOOM;
#endif /* !defined(_WIN32) */

    a->buf = buf;
    a->cap = aligned_cap;
    arena_reset(a);

    return OK;
}

/*
====================
arena_free_vm

Releases a demand-paged virtual-memory arena previously initialized
via `arena_init_vm` back to the OS (`munmap` / `VirtualFree`) and zeroes `*a`.

Returns:
- `OK`     on successful unmapping.
- `EINVAL` if `a` is `NULL`, uninitialized, or OS release fails.
====================
*/
static inline res_t arena_free_vm(arena_t *a) {
    ASSERT(a, "Attempted VM free on NULL arena");
    if   (!a || !a->buf || !a->cap) return EINVAL;

#ifdef _WIN32
    bool ok = (VirtualFree(a->buf, 0, MEM_RELEASE) != 0);
#else  /* !defined(_WIN32) */
    bool ok = (munmap(a->buf, a->cap) == 0);
#endif /* !defined(_WIN32) */
    *a = (arena_t){0};

    return ok ? OK : EINVAL;
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

    ASSERT(aligned_off + aligned_sz <= a->cap, "Arena ran out of memory (a: %p, sz: %zu)", (void *)a, sz);
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
    ASSERT(a,   "Attempted has check on NULL arena (ptr: %p, sz: %zu)",   ptr,         sz);
    ASSERT(ptr, "Attempted has check on NULL ptr   (arena: %p, sz: %zu)", (void *)a,   sz);
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
