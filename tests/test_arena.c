#include "arena.h"
#include "test.h"

#include <string.h>
#include <stdint.h>

int main(void) {
    TEST_CASE("arena_init", "alings buffer to CACHE_LINE_SZ") {
        uint8_t buf[CACHE_LINE_SZ * 16];
        arena_t a;
        res_t   res = arena_init(&a, buf, sizeof(buf));

        TEST_ASSERT(a.buf,                               "buf is NULL");
        TEST_ASSERT(res == OK,                           "expected OK return val, got %d",  res);
        TEST_ASSERT(!((uintptr_t)a.buf % CACHE_LINE_SZ), "buf not 64-byte aligned: %p",     a.buf);
        TEST_ASSERT(a.off == 0,                          "init off not equal 0, got %zu",   a.off);
        TEST_ASSERT(a.cap <= sizeof(buf),                "exp buf size (%zu) >= cap (%zu)", sizeof(buf), a.cap);
    } TEST_CASE_END();

    TEST_CASE("arena_alloc", "aligns allocs to CACHE_LINE_SZ") {
        size_t szs[] = {
            1,
            CACHE_LINE_SZ / 4,
            CACHE_LINE_SZ - 1,
            CACHE_LINE_SZ,
            CACHE_LINE_SZ + 1,
            2 * CACHE_LINE_SZ - 1,
            2 * CACHE_LINE_SZ,
            4 * CACHE_LINE_SZ
        };

        uint8_t buf[CACHE_LINE_SZ * 4 * ARR_CNT(szs)];
        arena_t a;
        arena_init(&a, buf, sizeof(buf));

        for (size_t i = 0; i < ARR_CNT(szs); ++i) {
            size_t sz = szs[i];
            void  *p  = arena_alloc(&a, sz);
            TEST_ASSERT(p,                               "alloc of %zuB failed",                sz);
            TEST_ASSERT(!((uintptr_t)p % CACHE_LINE_SZ), "ptr (%p) for sz %zu not 64B aligned", p,     sz);
            TEST_ASSERT(!(a.off % CACHE_LINE_SZ),        "off %zu not 64B aligned",             a.off);
        }
    } TEST_CASE_END();

    TEST_CASE("arena_alloc", "ensures allocs dont overlap") {
        _Alignas(CACHE_LINE_SZ) uint8_t buf[CACHE_LINE_SZ * 2];
        arena_t a;
        arena_init(&a, buf, sizeof(buf));

        uint8_t *p1   = arena_alloc(&a, CACHE_LINE_SZ);
        uint8_t *p2   = arena_alloc(&a, CACHE_LINE_SZ);
        size_t   diff = p2 - p1;

        TEST_ASSERT(p1,                    "alloc p1 failed");
        TEST_ASSERT(p2,                    "alloc p2 failed");
        TEST_ASSERT(diff >= CACHE_LINE_SZ, "ptrs overlap/too close, diff=%td", p2 - p1);

        for (size_t i = 0; i < CACHE_LINE_SZ; ++i) {
            memset(p1 + i, 0xA, 1);
            memset(p2 + i, 0xB, 1);
        }

        for (size_t i = 0; i < CACHE_LINE_SZ; ++i) {
            TEST_ASSERT(p1[i] == 0xA, "p1[%zu] corrupt: 0x%02X != 0xA", i, p1[i]);
            TEST_ASSERT(p2[i] == 0xB, "p2[%zu] corrupt: 0x%02X != 0xB", i, p2[i]);
        }
    } TEST_CASE_END();

    TEST_CASE("arena_alloc", "fills up exactly to a->cap") {
        _Alignas(CACHE_LINE_SZ) uint8_t buffer[CACHE_LINE_SZ * 3];

        arena_t a;
        arena_init(&a, buffer, sizeof(buffer));

        void *p1 = arena_alloc(&a, CACHE_LINE_SZ);
        void *p2 = arena_alloc(&a, CACHE_LINE_SZ);
        void *p3 = arena_alloc(&a, CACHE_LINE_SZ);

        TEST_ASSERT(p1,             "alloc p1 failed");
        TEST_ASSERT(p2,             "alloc p2 failed");
        TEST_ASSERT(p3,             "alloc p3 failed");
        TEST_ASSERT(a.off == a.cap, "exp off (%zu) == cap (%zu)", a.off, a.cap);
    } TEST_CASE_END();

    TEST_CASE("arena_reset", "rewinds offset properly") {
        uint8_t buf[CACHE_LINE_SZ * 8];
        arena_t a;
        arena_init(&a, buf, sizeof(buf));

        arena_alloc(&a, CACHE_LINE_SZ * 2);
        TEST_ASSERT(a.off, "exp off >0");

        arena_reset(&a);
        TEST_ASSERT(!a.off, "exp off =0");

        void *p = arena_alloc(&a, CACHE_LINE_SZ);
        TEST_ASSERT(p == a.buf, "post-reset first alloc exp at base");
    } TEST_CASE_END();

    TEST_CASE("arena_temp", "unwinds scratch properly") {
        uint8_t buf[CACHE_LINE_SZ * 8];
        arena_t a;
        arena_init(&a, buf, sizeof(buf));

        arena_alloc(&a, CACHE_LINE_SZ);
        size_t base_off = a.off;

        arena_temp_t t1 = arena_temp_begin(&a);
        void *s1 = arena_alloc(&a, CACHE_LINE_SZ * 2);
        TEST_ASSERT(s1, "scope 1 alloc failed");
        TEST_ASSERT(a.off > base_off, "off didnt advance in scope 1");

        arena_temp_t t2 = arena_temp_begin(&a);
        void *s2 = arena_alloc(&a, CACHE_LINE_SZ * 4);
        TEST_ASSERT(s2 != NULL, "scope 2 alloc failed");
        arena_temp_end(t2);

        size_t exp_off = base_off + CACHE_LINE_SZ * 2;
        TEST_ASSERT(a.off == exp_off, "scope 2 rewind failed: exp off (%zu) == base_off + 2 * LINE_SZ (%zu)", a.off, exp_off);

        arena_temp_end(t1);
        TEST_ASSERT(a.off == base_off, "scope 1 rewind failed: exp off (%zu) == base_off (%zu)", a.off, base_off);

        void *nxt = arena_alloc(&a, 64);
        TEST_ASSERT(nxt == s1, "realloced addr (%p) doesnt match rewound addr (%p)", nxt, s1);
    } TEST_CASE_END();

    TEST_END();
}
