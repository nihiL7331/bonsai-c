#include "core.h"
#include "reflect.h"
#include "test.h"

#include <stddef.h>
#include <string.h>
#include <stdbool.h>

int main(void) {
    TEST_CASE("reflect_comp_cnt", "validates count bounds and enum consistency") {
        TEST_ASSERT(reflect_comp_cnt() == COMP_CNT, "reflect_comp_cnt (%u) != COMP_COUNT (%u)", reflect_comp_cnt(), COMP_CNT);
        TEST_ASSERT(reflect_comp_cnt() >= 1,        "exp registry have >=1 comp, got %u",       COMP_CNT);
        TEST_ASSERT(reflect_comp_cnt() <= 64,       "COMP_COUNT (%u) exceeds 64b mask cap",     COMP_CNT);
    } TEST_CASE_END();

    TEST_CASE("comp_id", "verifies all component ids are bounded and unique") {
        uint64_t seen_mask = 0x0000000000000000;

        for (uint32_t i = 0; i < reflect_comp_cnt(); ++i) {
            const comp_meta_t *comp = reflect_comp_by_id((comp_id_t)i);

            TEST_ASSERT(comp,                              "meta for ID %u is NULL",    i);
            TEST_ASSERT(comp->id < COMP_CNT,               "ID (%d) > COMP_COUNT (%u)", comp->id, COMP_CNT);
            TEST_ASSERT(!(seen_mask & (1ULL << comp->id)), "dupe comp ID: %d for %s",   comp->id, comp->name);

            seen_mask |= (1ULL << comp->id);
        }
    } TEST_CASE_END();

    TEST_CASE("reflect_lookup", "verifies round-trip integrity for all components") {
        for (uint32_t i = 0; i < reflect_comp_cnt(); ++i) {
            const comp_meta_t *comp = reflect_comp_by_id((comp_id_t)i);

            TEST_ASSERT(comp,                                     "meta for ID %u is NULL",                       i);
            TEST_ASSERT(comp->id == (comp_id_t)i,                 "ID mismatch on comp %u",                       i);
            TEST_ASSERT(comp->name && comp->name[0],              "comp %u invalid name",                         i);
            TEST_ASSERT(comp->size      > 0,                      "comp '%s' has 0 size",                         comp->name);
            TEST_ASSERT(comp->field_cnt > 0,                      "comp '%s' has 0 fields",                       comp->name);
            TEST_ASSERT(comp->def_val,                            "comp '%s' has NULL def value",                 comp->name);
            TEST_ASSERT(comp == reflect_comp_by_name(comp->name), "name lookup for '%s' did not match ID lookup", comp->name);
        }
    } TEST_CASE_END();

    TEST_CASE("reflect_lookup", "returns NULL on unknown/invalid names") {
        TEST_ASSERT(!reflect_comp_by_name("nonexistent"),      "exp NULL for unknown name");
        TEST_ASSERT(!reflect_comp_by_name(""),                 "exp NULL for \"\" name");
        TEST_ASSERT(!reflect_comp_by_name("TRANS2D"),          "exp NULL for mismatched case");
        TEST_ASSERT(!reflect_comp_by_name("trans2"),           "exp NULL for substr");
        TEST_ASSERT(!reflect_comp_by_name("trans2dDEADBEEF"),  "suffix 'data' exp to return NULL");
        TEST_ASSERT(reflect_comp_by_name("trans2d\0DEADBEEF"), "suffix '\\0-term+data' lookup fail");
    } TEST_CASE_END();

    TEST_CASE("layout_invariants", "verifies offsets/no overlap") {
        for (uint32_t i = 0; i < reflect_comp_cnt(); ++i) {
            const comp_meta_t *comp     = reflect_comp_by_id((comp_id_t)i);
            uint16_t           prev_end = 0;

            for (uint16_t f = 0; f < comp->field_cnt; ++f) {
                const field_meta_t *field     = &comp->fields[f];
                uint16_t            field_end = field->off + field->size;

                TEST_ASSERT(field->off >= prev_end,   "comp '%s' field '%s' overlaps with previous field (off %u < prev_end %u)", comp->name, field->name, field->off, prev_end);
                TEST_ASSERT(field_end  <= comp->size, "comp '%s' field '%s' exceeds struct size (end %u > size %u)",              comp->name, field->name, field_end,  comp->size);

                prev_end = field_end;
            }

            TEST_ASSERT(comp->size <= CACHE_LINE_SZ, "comp '%s' size (%uB) exceeds CACHE_LINE_SZ limit", comp->name, comp->size);
        }
    } TEST_CASE_END();

    TEST_CASE("feature_types", "validates types handling") {
        static const size_t type_elem_szs[TYPE_CNT] = {
            [TYPE_F32]    = sizeof(float),
            [TYPE_F64]    = sizeof(double),
            [TYPE_I8]     = sizeof(int8_t),
            [TYPE_I16]    = sizeof(int16_t),
            [TYPE_I32]    = sizeof(int32_t),
            [TYPE_I64]    = sizeof(int64_t),
            [TYPE_U8]     = sizeof(uint8_t),
            [TYPE_U16]    = sizeof(uint16_t),
            [TYPE_U32]    = sizeof(uint32_t),
            [TYPE_U64]    = sizeof(uint64_t),
            [TYPE_BOOL]   = sizeof(bool),
            [TYPE_STR]    = sizeof(char),
            [TYPE_ENTITY] = sizeof(entity_handle_t),
        };
        _Static_assert(ARR_CNT(type_elem_szs) == TYPE_CNT, "type_elem_szs arr is out of sync with reflect_type_t");
        for (size_t i = 0; i < TYPE_CNT; ++i) TEST_ASSERT(type_elem_szs[i], "reflect_type_t enum %zu has no size defined in type_elem_szs arr", i);

        for (uint32_t i = 0; i < reflect_comp_cnt(); ++i) {
            const comp_meta_t *comp = reflect_comp_by_id((comp_id_t)i);

            for (uint16_t f = 0; f < comp->field_cnt; ++f) {
                const field_meta_t *field = &comp->fields[f];

                TEST_ASSERT(field->type  < TYPE_CNT,                                "comp '%s' field '%s' has out-of-bounds type enum %d",          comp->name, field->name, field->type);
                TEST_ASSERT(field->cnt  >= 1,                                       "comp '%s' field '%s' cnt <=0",                                 comp->name, field->name);
                TEST_ASSERT(field->size == field->cnt * type_elem_szs[field->type], "comp '%s' field '%s' size (%uB) != cnt (%u) * elem_sz (%zuB)", comp->name, field->name, field->size, field->cnt, type_elem_szs[field->type]);
            }
        }
    } TEST_CASE_END();

    TEST_CASE("reflect_defaults", "matches concrete public defaults and field offsets") {
        const comp_meta_t *meta = reflect_comp_by_id(COMP_trans3d);
        TEST_ASSERT(meta != NULL && meta->size == sizeof(trans3d), "trans3d metadata matches public type");
        if (meta && meta->def_val) {
            trans3d value;
            memcpy(&value, meta->def_val, sizeof(value));
            TEST_ASSERT(value.qw == 0.0f && value.qx == 0.0f && value.qy == 0.0f && value.qz == 0.0f,
                        "default quaternion matches documented zero sentinel");
            TEST_ASSERT(value.scale_x == 1.0f && value.scale_y == 1.0f && value.scale_z == 1.0f,
                        "default 3D scale is one");
        }
        meta = reflect_comp_by_id(COMP_tag);
        TEST_ASSERT(meta != NULL && meta->field_cnt == 1, "tag exposes one field");
        if (meta && meta->field_cnt == 1) {
            const field_meta_t *field = &meta->fields[0];
            TEST_ASSERT(strcmp(field->name, "name") == 0 && field->type == TYPE_STR, "tag name is a string field");
            TEST_ASSERT(field->off == offsetof(tag, name) && field->size == sizeof(((tag *)0)->name),
                        "tag string metadata matches C layout");
            TEST_ASSERT(field->cnt == 32, "tag string capacity is 32 bytes");
        }
    } TEST_CASE_END();

    TEST_END();
}
