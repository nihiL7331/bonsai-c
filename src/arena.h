#pragma once

#include "core.h"
#include <stddef.h>

typedef struct arena {
   void *buf;
   size_t cap;
   size_t off;
} arena_t;

typedef struct arena_temp {
    arena_t *arena;
    size_t prev_off;
} arena_temp_t;

static inline void arena_reset(arena_t *a) {
    ASSERT(a, "Attempted reset on NULL arena");
    a->off = 0;
}

static inline res_t arena_init(arena_t *a, void *buf, size_t cap) {
    ASSERT(a,   "Attempted init on NULL arena (buf: %p, cap: %zu)",          buf, cap);
    ASSERT(buf, "Attempted init arena with NULL buffer (a: %p, cap: %zu)",   a,   cap);
    uintptr_t raw     = (uintptr_t)buf;
    uintptr_t aligned = (uintptr_t)ALIGN_LINE_PTR(buf);
    size_t pad        = (size_t)(aligned - raw);

    a->buf = (void *)aligned;
    a->cap = (cap > pad) ? (cap - pad) : 0;
    arena_reset(a);

    return a->cap ? OK : WEMPTY;
}

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

static inline arena_temp_t arena_temp_begin(arena_t *a) {
    ASSERT(a, "Attempted temp begin on NULL arena");
    return (arena_temp_t){ .arena = a, .prev_off = a->off };
}

static inline void arena_temp_end(arena_temp_t a_tmp) {
    ASSERT(a_tmp.arena,                        "Attempted temp end on NULL arena");
    ASSERT(a_tmp.arena->off >= a_tmp.prev_off, "Arena corruption detected during temp unwind");
    a_tmp.arena->off = a_tmp.prev_off;
}
