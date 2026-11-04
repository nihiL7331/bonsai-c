#pragma once

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#if defined(__GNUC__) || defined(__clang__)
    #define TRAP() __builtin_trap()
#else //  !defined(__GNUC__) && !defined(__clang__)
    #define TRAP() (*(volatile int *)0 = 0)
#endif // !defined(__GNUC__) && !defined(__clang__)

#define CACHE_LINE_SZ       64
#define ALIGN_LINE_PTR(ptr) ((void *)(((uintptr_t)(ptr) + (CACHE_LINE_SZ - 1)) & ~((uintptr_t)CACHE_LINE_SZ - 1)))
#define ALIGN_LINE_INT(x)   (((uintptr_t)(x) + (CACHE_LINE_SZ - 1)) & ~((uintptr_t)CACHE_LINE_SZ - 1))

#define ARR_CNT(arr)        (sizeof(arr) / sizeof(*arr))
#define UNUSED(x)           ((void)x)

#define _MIN2(a, b)       (((a) < (b)) ? (a) : (b))
#define _MIN3(a, b, c)    _MIN2(_MIN2(a, b), c)
#define _MIN4(a, b, c, d) _MIN2(_MIN3(a, b, c), d)

#define _MIN_SELECT(_1, _2, _3, _4, NAME, ...) NAME
#define MIN(...) _MIN_SELECT(__VA_ARGS__, _MIN4, _MIN3, _MIN2, 0)(__VA_ARGS__)

#define _MAX2(a, b)       (((a) > (b)) ? (a) : (b))
#define _MAX3(a, b, c)    _MAX2(_MAX2(a, b), c)
#define _MAX4(a, b, c, d) _MAX2(_MAX3(a, b, c), d)

#define _MAX_SELECT(_1, _2, _3, _4, NAME, ...) NAME
#define MAX(...) _MAX_SELECT(__VA_ARGS__, _MAX4, _MAX3, _MAX2, 0)(__VA_ARGS__)

static inline void log_err(const char *file, int line, const char *func, const char *fmt, ...) {
    // TODO: log into MCP logs
    fprintf(stderr, "%s:%d in %s(): ", file, line, func);
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fprintf(stderr, "\n");
}

#ifdef DEBUG
#define ASSERT(cond, ...)                                                                          \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            log_err(__FILE__, __LINE__, __func__, "ASSERT: (" #cond ") " __VA_ARGS__);             \
            TRAP();                                                                                \
        }                                                                                          \
    } while (0)
#else  // !DEBUG
#define ASSERT(cond, ...) do { } while (0)
#endif // !DEBUG

#define PANIC(...)                                                                                 \
    do {                                                                                           \
        log_err(__FILE__, __LINE__, __func__, "PANIC: " __VA_ARGS__);                              \
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
