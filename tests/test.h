#pragma once

#include "core.h"

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

#define TEST_END()                                                                                 \
    do {                                                                                           \
        printf("%" PRIu32 " tests, %" PRIu32 " failed\n", s_tests_run, s_tests_failed);            \
        if (s_tests_failed) printf("%sfail%s\n\n", LOG_TC(LOG_CLR_RED), LOG_TC(LOG_CLR_RST));      \
        else                printf("%spass%s\n\n", LOG_TC(LOG_CLR_GRN), LOG_TC(LOG_CLR_RST));      \
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
                    printf("%sfail%s\n", LOG_TC(LOG_CLR_RED), LOG_TC(LOG_CLR_RST));                \
                    _ok = false;                                                                   \
                    s_tests_failed++;                                                              \
                }                                                                                  \
                printf("\t%s-> %s:%d: (%s)%s ",                                                    \
                        LOG_TC(LOG_CLR_RED), __FILE__, __LINE__, #cond, LOG_TC(LOG_CLR_RST));      \
                printf("" __VA_ARGS__);                                                            \
                printf("\n");                                                                      \
            }                                                                                      \
        } while (0)

#define TEST_CASE_END() if (_ok) printf("%sok%s\n", LOG_TC(LOG_CLR_GRN), LOG_TC(LOG_CLR_RST));     \
    } while (0)
