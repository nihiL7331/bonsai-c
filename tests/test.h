#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static uint32_t s_tests_run    = 0;
static uint32_t s_tests_failed = 0;

#define TEST_CASE_SCOPE_LEN 20
#define TEST_CASE_DESC_LEN  50

static inline int test_use_color(void) {
    static int cached = -1;
    if (cached < 0) {
        const char *no_color = getenv("NO_COLOR");
        cached = isatty(STDOUT_FILENO) && !(no_color && *no_color);
    }
    return cached;
}

#define TEST_ESC "\033"
#define TEST_RED TEST_ESC "[1;31m"
#define TEST_GRN TEST_ESC "[1;32m"
#define TEST_RST TEST_ESC "[0m"

#define TC(code) (test_use_color() ? (code) : "")

#define TEST_END()                                                                                 \
    do {                                                                                           \
        printf("%" PRIu32 " tests, %" PRIu32 " failed\n", s_tests_run, s_tests_failed);            \
        if (s_tests_failed) printf("%sfail%s\n\n", TC(TEST_RED), TC(TEST_RST));                    \
        else                printf("%spass%s\n\n", TC(TEST_GRN), TC(TEST_RST));                    \
        return s_tests_failed ? 1 : 0;                                                             \
    } while (0)

#define TEST_CASE(scope, desc)                                                                     \
    do {                                                                                           \
        printf("\t%s:%*s %-*s... ",                                                                \
               (scope), TEST_CASE_SCOPE_LEN - (int)strlen(scope), "", TEST_CASE_DESC_LEN, (desc)); \
        fflush(stdout);                                                                            \
        s_tests_run++;                                                                             \
        bool _ok = true;

#define TEST_ASSERT(cond, ...)                                                                     \
        do {                                                                                       \
            if (!(cond)) {                                                                         \
                if (_ok) {                                                                         \
                    printf("%sfail%s\n", TC(TEST_RED), TC(TEST_RST));                              \
                    _ok = false;                                                                   \
                    s_tests_failed++;                                                              \
                }                                                                                  \
                printf("\t%s-> %s:%d: (%s)%s ",                                                    \
                        TC(TEST_RED), __FILE__, __LINE__, #cond, TC(TEST_RST));                    \
                printf("" __VA_ARGS__);                                                            \
                printf("\n");                                                                      \
            }                                                                                      \
        } while (0)

#define TEST_CASE_END() if (_ok) printf("%sok%s\n", TC(TEST_GRN), TC(TEST_RST));                   \
    } while (0)
