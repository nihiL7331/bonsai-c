#pragma once

/*
====================
core.h

Foundational engine utilities: 64-byte cache-line alignment, vector/matrix
printf formatters, branch-free math macros, diagnostic logging,
debug-only assertions, fatal panics, and the global `res_t` status code enum.
====================
*/

#include <math.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#if defined(__GNUC__) || defined(__clang__)
    #define TRAP() __builtin_trap()
#else //  !defined(__GNUC__) && !defined(__clang__)
    #define TRAP() (*(volatile int *)0 = 0)
#endif // !defined(__GNUC__) && !defined(__clang__)

/* ============================================================================================== */
/* >CACHE LINE HELPERS                                                                            */
/* ============================================================================================== */

#define CACHE_LINE_SZ       64

/*
====================
ALIGN_LINE_PTR

Rounds a memory pointer upwards to the nearest 64-byte
cache-line bounadry (`CACHE_LINE_SZ`). Used by arena allocator 
to prevent false sharing and ensure optimal SIMD vector alignment.
====================
*/
#define ALIGN_LINE_PTR(ptr) ((void *)(((uintptr_t)(ptr) + (CACHE_LINE_SZ - 1)) & ~((uintptr_t)CACHE_LINE_SZ - 1)))

/*
====================
ALIGN_LINE_INT

Rounds an integer size or byte offset upwards to the
nearest 64-byte cache-line multiple (`CACHE_LINE_SZ`).
====================
*/
#define ALIGN_LINE_INT(x)   (((uintptr_t)(x) + (CACHE_LINE_SZ - 1)) & ~((uintptr_t)CACHE_LINE_SZ - 1))

/* ============================================================================================== */
/* >FORMATTING HELPERS                                                                            */
/* ============================================================================================== */

/* usage:                                                                 */
/*      if v = (vec2f){ 1.0f, 2.0f }                                      */
/*      "this failed, some vec2: " FMT_VEC2F "", ARG_VEC2F(v)             */
/* expands to:                                                            */
/*      "this failed, some vec2: (1.00, 2.00)"                            */

#define FMT_VEC2F   "(%.2f, %.2f)"
#define ARG_VEC2F(v) (v).x, (v).y

#define FMT_VEC2I   "(%" PRId32 ", %" PRId32 ")"
#define ARG_VEC2I(v) (v).x,       (v).y

#define FMT_VEC3F   "(%.2f, %.2f, %.2f)"
#define ARG_VEC3F(v) (v).x, (v).y, (v).z

#define FMT_VEC3I   "(%" PRId32 ", %" PRId32 ", %" PRId32 ")"
#define ARG_VEC3I(v) (v).x,       (v).y,       (v).z

#define FMT_VEC4F   "(%.2f, %.2f, %.2f, %.2f)"
#define ARG_VEC4F(v) (v).x, (v).y, (v).z, (v).w

#define FMT_VEC4I   "(%" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ")"
#define ARG_VEC4I(v) (v).x,       (v).y,       (v).z,       (v).w

#define FMT_MAT4F    "[" FMT_VEC4F "; " FMT_VEC4F "; " FMT_VEC4F "; " FMT_VEC4F "]"
#define ARG_MAT4F(m) ARG_VEC4F((m).col[0]), ARG_VEC4F((m).col[1]), ARG_VEC4F((m).col[2]), ARG_VEC4F((m).col[3])

#define FMT_AABB2F    "[" FMT_VEC2F " -> " FMT_VEC2F "]"
#define ARG_AABB2F(b) ARG_VEC2F((b).min), ARG_VEC2F((b).max)

#define FMT_AABB2I    "[" FMT_VEC2I " -> " FMT_VEC2I "]"
#define ARG_AABB2I(b) ARG_VEC2I((b).min), ARG_VEC2I((b).max)

#define FMT_AABB3F    "[" FMT_VEC3F " -> " FMT_VEC3F "]"
#define ARG_AABB3F(b) ARG_VEC3F((b).min), ARG_VEC3F((b).max)

#define FMT_AABB3I    "[" FMT_VEC3I " -> " FMT_VEC3I "]"
#define ARG_AABB3I(b) ARG_VEC3I((b).min), ARG_VEC3I((b).max)

#define FMT_QUAT    FMT_VEC4F
#define ARG_QUAT(q) ARG_VEC4F(q)

/* ============================================================================================== */
/* >GENERIC HELPERS                                                                               */
/* ============================================================================================== */

#define ARR_CNT(arr) (sizeof(arr) / sizeof(*arr))
#define UNUSED(x)    ((void)x)
#define NOOP         do { } while (0)

#define _MIN2(a, b)       (((a) < (b)) ? (a) : (b))
#define _MIN3(a, b, c)    _MIN2(_MIN2(a, b), c)
#define _MIN4(a, b, c, d) _MIN2(_MIN3(a, b, c), d)
#define _MIN_SELECT(_1, _2, _3, _4, NAME, ...) NAME

/*
====================
MIN

Variadic minimum macro supporting 2 to 4 arguments of comparable types.
Evaluated via macro overload selection (`_MIN_SELECT`).
====================
*/
#define MIN(...) _MIN_SELECT(__VA_ARGS__, _MIN4, _MIN3, _MIN2, 0)(__VA_ARGS__)

#define _MAX2(a, b)       (((a) > (b)) ? (a) : (b))
#define _MAX3(a, b, c)    _MAX2(_MAX2(a, b), c)
#define _MAX4(a, b, c, d) _MAX2(_MAX3(a, b, c), d)
#define _MAX_SELECT(_1, _2, _3, _4, NAME, ...) NAME

/*
====================
MAX

Variadic maximum macro supporting 2 to 4 arguments of comparable types.
Evaluated via macro overload selection (`_MAX_SELECT`).
====================
*/
#define MAX(...) _MAX_SELECT(__VA_ARGS__, _MAX4, _MAX3, _MAX2, 0)(__VA_ARGS__)

/*
====================
CLAMP

Clamps `x` within the inclusive range `[lo, hi]`.
====================
*/
#define CLAMP(x, lo, hi) (((x) < (lo) ? (lo) : (((x) > (hi)) ? (hi) : (x))))

/*
====================
ABS

Type-generic absolute value operator.
Dispatches to `fabsf`, `fabs`, `abs`, `labs` or `llabs` based on arg type.
Expands `int8/16_t` to `int` via `(x) + 0` casting and uses `abs`.
====================
*/
#define ABS(x) _Generic((x) + 0, float: fabsf, double: fabs, int: abs, long: labs, long long: llabs)((x))

/*
====================
IS_POW2

Evaluates whether integer `x` is a non-zero power of 2 without branching.
====================
*/
#define IS_POW2(x) ((x) && !(((x) & ((x) - 1))))

/*
BIT

Generates an unsigned 64-bit integer bitmask with bit `n` set.
*/
#define BIT(n) (1ULL << (n))

#define KB(x) ((size_t)(x) * 1024ULL)
#define MB(x) ((size_t)(x) * 1024ULL * 1024ULL)
#define GB(x) ((size_t)(x) * 1024ULL * 1024ULL * 1024ULL)

/* ============================================================================================== */
/* >LOGGING & PROGRAM LIFECYCLE                                                                   */
/* ============================================================================================== */

static inline int _core_use_color(void) {
    static int cached = -1;
    if (cached < 0) {
        const char *no_color = getenv("NO_COLOR");
        cached               = isatty(STDOUT_FILENO) && !(no_color && *no_color);
    }
    return cached;
}

#define LOG_TC(code) (_core_use_color() ? (code) : "")
#define LOG_CLR_RST  "\033[0m"
#define LOG_CLR_RED  "\033[1;31m"
#define LOG_CLR_GRN  "\033[1;32m"
#define LOG_CLR_YLW  "\033[1;33m"

static inline void _log(const char *color, const char *prefix, FILE *s,
                        const char *file, int line, const char *func, const char *fmt, ...) {
    // TODO: log into MCP logs
    fprintf(s, "%s[%s]%s %s:%d in %s(): ", LOG_TC(color), prefix, LOG_TC(LOG_CLR_RST), file, line, func);
    va_list args;
    va_start(args, fmt);
    vfprintf(s, fmt, args);
    va_end(args);
    fprintf(s, "\n");
}

/*
====================
LOG_WARN

Emits a formatted diagnostic warning to `stderr`
with file, line, and function context.
Active in debug builds, compiles out to `NOOP` in release.
====================
*/
#ifdef DEBUG
#define LOG_WARN(...) _log(LOG_CLR_YLW, "WARN", stderr, __FILE__, __LINE__, __func__, __VA_ARGS__)
#else  // !DEBUG
#define LOG_WARN(...) NOOP
#endif // !DEBUG

/*
====================
LOG_ERR

Emits a formatted error log to `stderr`
with file, line, and function context.
Active across all build configurations (debug/release).
====================
*/
#define LOG_ERR(...)  _log(LOG_CLR_RED, "ERR ", stderr, __FILE__, __LINE__, __func__, __VA_ARGS__)

/*
====================
ASSERT

In debug builds, evaluates `cond`. If false,
logs failure details via `LOG_ERR` and triggers `TRAP()`.
In release builds, compiles out to `NOOP`.
Hot paths assume contracts hold.
====================
*/
#ifdef DEBUG
#define ASSERT(cond, ...)                                                                          \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            LOG_ERR("ASSERT: (" #cond ") " __VA_ARGS__);                                           \
            TRAP();                                                                                \
        }                                                                                          \
    } while (0)
#else  // !DEBUG
#define ASSERT(cond, ...) NOOP
#endif // !DEBUG

/*
====================
PANIC

Unrecoverable fatal error handler.
Logs the formatted error message via `LOG_ERR`
and halts execution via `TRAP()` in all build
configurations (debug and release).
====================
*/
#define PANIC(...)                                                                                 \
    do {                                                                                           \
        LOG_ERR("PANIC: " __VA_ARGS__);                                                            \
        TRAP();                                                                                    \
    } while (0)

/*
====================
res_t

Global result and error status code enumeration.
Functions returning `res_t` adhere to defensive
lifecycle and boundary return conventions.

Values:
- `OK`:       Operation completed successfully.
- `EOOM`:     Out of memory, capacity exceeded.
- `EINVAL`:   Invalid parameter (e.g., unexpected NULL ptr or out-of-range value).
- `ENOTFND`:  Requested entity, component, or asset was not found.
- `EFULL`:    Static capacity or fixed-size collection is full.
- `ESCHLOCK`: Concurrency or ring-buffer contention failure.
- `ESCRTOUT`: Script execution timed out or aborted.
- `ERLLBCK`:  State rollback or frame transaction verification failed.
- `EIO`:      I/O or serialization failure.
- `WEMPTY`:   Empty collection, zero elements, or zero usable capacity.
====================
*/
typedef enum res {
    OK,
    EOOM,
    EINVAL,
    ENOTFND,
    EFULL,
    ESCHLOCK,
    ESCRTOUT,
    ERLLBCK,
    EIO,
    WEMPTY,
} res_t;
