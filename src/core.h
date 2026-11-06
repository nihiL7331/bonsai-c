#pragma once

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
#define ALIGN_LINE_PTR(ptr) ((void *)(((uintptr_t)(ptr) + (CACHE_LINE_SZ - 1)) & ~((uintptr_t)CACHE_LINE_SZ - 1)))
#define ALIGN_LINE_INT(x)   (((uintptr_t)(x) + (CACHE_LINE_SZ - 1)) & ~((uintptr_t)CACHE_LINE_SZ - 1))

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
/* variadic 1-4 args math minimum */
#define MIN(...) _MIN_SELECT(__VA_ARGS__, _MIN4, _MIN3, _MIN2, 0)(__VA_ARGS__)

#define _MAX2(a, b)       (((a) > (b)) ? (a) : (b))
#define _MAX3(a, b, c)    _MAX2(_MAX2(a, b), c)
#define _MAX4(a, b, c, d) _MAX2(_MAX3(a, b, c), d)
#define _MAX_SELECT(_1, _2, _3, _4, NAME, ...) NAME
/* variadic 1-4 args math maximum */
#define MAX(...) _MAX_SELECT(__VA_ARGS__, _MAX4, _MAX3, _MAX2, 0)(__VA_ARGS__)

#define CLAMP(x, lo, hi) (((x) < (lo) ? (lo) : (((x) > (hi)) ? (hi) : (x))))

#define IS_POW2(x) ((x) && !(((x) & ((x) - 1))))

#define BIT(n) (1ULL << (n))

#define KB(x) ((size_t)(x) * 1024ULL)
#define MB(x) ((size_t)(x) * 1024ULL * 1024ULL)
#define GB(x) ((size_t)(x) * 1024ULL * 1024ULL * 1024ULL)

/* ============================================================================================== */
/* >LOGGING & PROGRAM LIFECYCLE                                                                   */
/* ============================================================================================== */

static inline int test_use_color(void) {
    static int cached = -1;
    if (cached < 0) {
        const char *no_color = getenv("NO_COLOR");
        cached               = isatty(STDOUT_FILENO) && !(no_color && *no_color);
    }
    return cached;
}

#define LOG_TC(code) (test_use_color() ? (code) : "")
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

#ifdef DEBUG
#define LOG_WARN(...) _log(LOG_CLR_YLW, "WARN", stderr, __FILE__, __LINE__, __func__, __VA_ARGS__)
#else  // !DEBUG
#define LOG_WARN(...) NOOP
#endif // !DEBUG
#define LOG_ERR(...)  _log(LOG_CLR_RED, "ERR ", stderr, __FILE__, __LINE__, __func__, __VA_ARGS__)

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

#define PANIC(...)                                                                                 \
    do {                                                                                           \
        LOG_ERR("PANIC: " __VA_ARGS__);                                                            \
        TRAP();                                                                                    \
    } while (0)

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
