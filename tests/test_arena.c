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

    TEST_CASE("arena_vm_def", "reserves def VM arena and demand-pages writes") {
        arena_t a = {0};
        res_t   r = arena_init_vm(&a, 0);

        TEST_ASSERT(r     == OK,               "arena_init_vm(0) failed with %d", r);
        TEST_ASSERT(a.buf != NULL,             "VM buf is NULL");
        TEST_ASSERT(a.cap == ARENA_DEF_VM_CAP, "exp cap %zu, got %zu", (size_t)ARENA_DEF_VM_CAP, a.cap);
        TEST_ASSERT(a.off == 0,                "exp initial off 0, got %zu", a.off);
        TEST_ASSERT(!((uintptr_t)a.buf % CACHE_LINE_SZ), "VM buf not 64B aligned: %p", a.buf);

        uint32_t *blk1 = (uint32_t *)arena_alloc(&a, MB(4));
        uint32_t *blk2 = (uint32_t *)arena_alloc(&a, MB(8));
        TEST_ASSERT(blk1 != NULL && blk2 != NULL, "VM arena multi-MB alloc failed");
        TEST_ASSERT(arena_has(&a, blk1, MB(4)),   "arena_has failed for blk1");
        TEST_ASSERT(arena_has(&a, blk2, MB(8)),   "arena_has failed for blk2");

        /* touch first and last pages to trigger demand paging */
        blk1[0]                           = 0xDEADBEEFU;
        blk2[(MB(8) / sizeof(*blk2)) - 1] = 0xCAFEBABEU;
        TEST_ASSERT(blk1[0] == 0xDEADBEEFU,                           "blk1 write/read mismatch");
        TEST_ASSERT(blk2[(MB(8) / sizeof(*blk2)) - 1] == 0xCAFEBABEU, "blk2 write/read mismatch");

        res_t fr = arena_free_vm(&a);
        TEST_ASSERT(fr    == OK,   "arena_free_vm failed with %d", fr);
        TEST_ASSERT(a.buf == NULL, "buf not cleared after free");
        TEST_ASSERT(a.cap == 0,    "cap not cleared after free");
        TEST_ASSERT(a.off == 0,    "off not cleared after free");
    } TEST_CASE_END();

    TEST_CASE("arena_vm_custom", "reserves explicit VM capacity and aligns up") {
        arena_t a = {0};
        res_t   r = arena_init_vm(&a, 100);

        TEST_ASSERT(r     == OK,                  "arena_init_vm(100) failed");
        TEST_ASSERT(a.cap == ALIGN_LINE_INT(100), "exp aligned cap %zu, got %zu", ALIGN_LINE_INT(100), a.cap);

        void *p1 = arena_alloc(&a, 64);
        void *p2 = arena_alloc(&a, 64);
        TEST_ASSERT(p1 != NULL && p2 != NULL, "expected 2x64B allocs to succeed");
        TEST_ASSERT(a.off == a.cap,           "expected arena to be full");

        TEST_ASSERT(arena_free_vm(&a) == OK,  "arena_free_vm failed");
        TEST_ASSERT(arena_free_vm(&a) == EINVAL, "double free on zeroed arena must return EINVAL");
    } TEST_CASE_END();

    TEST_CASE("arena_has_bounds", "tracks active ranges through unwind and reset") {
        _Alignas(CACHE_LINE_SZ) uint8_t buf[CACHE_LINE_SZ * 4];
        arena_t a;
        TEST_ASSERT(arena_init(&a, buf, sizeof(buf)) == OK, "arena initialized");
        uint8_t *persistent = arena_alloc(&a, CACHE_LINE_SZ);
        arena_temp_t mark = arena_temp_begin(&a);
        uint8_t *temporary = arena_alloc(&a, CACHE_LINE_SZ);

        TEST_ASSERT(arena_has(&a, persistent, CACHE_LINE_SZ * 2), "active range spans both allocations");
        TEST_ASSERT(!arena_has(&a, temporary, CACHE_LINE_SZ + 1), "range past active end rejected");
        TEST_ASSERT(!arena_has(&a, buf + CACHE_LINE_SZ * 3, 1), "unused capacity rejected");
        arena_temp_end(mark);
        TEST_ASSERT(arena_has(&a, persistent, CACHE_LINE_SZ), "earlier allocation survives unwind");
        TEST_ASSERT(!arena_has(&a, temporary, 1), "temporary allocation invalidated");
        arena_reset(&a);
        TEST_ASSERT(!arena_has(&a, persistent, 1), "reset invalidates earlier allocation");
    } TEST_CASE_END();

    TEST_END();
}
