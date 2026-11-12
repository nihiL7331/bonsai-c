/*
====================
reflect.c

Single-source reflection engine implementation.
Initializes component descriptors lazily on first access
and provides fast O(1) ID lookups and O(N) name resolution.
====================
*/

#include "reflect.h"
#include "core.h"

#include <string.h>

#define _RTYPE_float           TYPE_F32
#define _RTYPE_double          TYPE_F64
#define _RTYPE_int8_t          TYPE_I8
#define _RTYPE_int16_t         TYPE_I16
#define _RTYPE_int32_t         TYPE_I32
#define _RTYPE_int64_t         TYPE_I64
#define _RTYPE_uint8_t         TYPE_U8
#define _RTYPE_uint16_t        TYPE_U16
#define _RTYPE_uint32_t        TYPE_U32
#define _RTYPE_uint64_t        TYPE_U64
#define _RTYPE_bool            TYPE_BOOL
#define _RTYPE__Bool           TYPE_BOOL
#define _RTYPE_char            TYPE_STR
#define _RTYPE_entity_handle_t TYPE_ENTITY
#define _RTYPE_vec2f           TYPE_VEC2F
#define _RTYPE_vec3f           TYPE_VEC3F
#define _RTYPE_vec4f           TYPE_VEC4F
#define _RTYPE_vec2i           TYPE_VEC2I
#define _RTYPE_vec3i           TYPE_VEC3I
#define _RTYPE_vec4i           TYPE_VEC4I
#define _RTYPE_aabb2f          TYPE_AABB2F
#define _RTYPE_aabb3f          TYPE_AABB3F
#define _RTYPE_aabb2i          TYPE_AABB2I
#define _RTYPE_aabb3i          TYPE_AABB3I
#define _RTYPE_mat3f           TYPE_MAT3F
#define _RTYPE_mat4f           TYPE_MAT4F
#define _RTYPE_quatf           TYPE_QUATF

/* ============================================================================================== */
/* >DEF STRUCT DEF GEN                                                                            */
/* ============================================================================================== */

/*
    expands

        COMP(collider2d)
            FIELD(float,    half_w,     0.0f)
            FIELD(float,    half_h,     0.0f)
            FIELD(float,    off_x,      0.0f)
            FIELD(float,    off_y,      0.0f)
            FIELD(uint32_t, mask,       0x00000001)
            FIELD(bool,     trigger,    false)
            FIELD(uint8_t,  _pad,    3, {0})
        COMP_END(collider2d)

    to

        const collider2d collider2d_def = {
            .half_w  = 0.0f,
            .half_h  = 0.0f,
            .off_x   = 0.0f,
            .off_y   = 0.0f,
            .mask    = 0x00000001,
            .trigger = false,
            ._pad    = {0},
        };
*/

#define REQUIRES(...)
#define EXCLUDES(...)
#define COMP(NAME)               const NAME NAME##_def = {
#define FIELD(T, NAME,      ...) .NAME = __VA_ARGS__,
#define ARRAY(T, NAME, CNT, ...) .NAME = __VA_ARGS__,
#define COMP_END(NAME)           };

#include "components.def"

#undef COMP_END
#undef ARRAY
#undef FIELD
#undef COMP
#undef EXCLUDES
#undef REQUIRES

/* ============================================================================================== */
/* >FIELD META GEN                                                                                */
/* ============================================================================================== */

/*
    expands

        COMP(collider2d)
            FIELD(float,    half_w,     0.0f)
            FIELD(float,    half_h,     0.0f)
            FIELD(float,    off_x,      0.0f)
            FIELD(float,    off_y,      0.0f)
            FIELD(uint32_t, mask,       0x00000001)
            FIELD(bool,     trigger,    false)
            FIELD(uint8_t,  _pad,    3, {0})
        COMP_END(collider2d)

    to

        static const field_meta_t *s_get_collider2d_fields(uint16_t *out_cnt) {
            typedef collider2d _curr_comp_t;
            static const field_meta_t fields[] = {
                { half_w,  float,    _RTYPE_float,    (uint16_t)offsetof(_curr_comp_t, half_w),  (uint16_t)sizeof(((_curr_comp_t *)0)->half_w),  1 },
                { half_h,  float,    _RTYPE_float,    (uint16_t)offsetof(_curr_comp_t, half_h),  (uint16_t)sizeof(((_curr_comp_t *)0)->half_h),  1 },
                { off_x,   float,    _RTYPE_float,    (uint16_t)offsetof(_curr_comp_t, off_x),   (uint16_t)sizeof(((_curr_comp_t *)0)->off_x),   1 },
                { off_y,   float,    _RTYPE_float,    (uint16_t)offsetof(_curr_comp_t, off_y),   (uint16_t)sizeof(((_curr_comp_t *)0)->off_y),   1 },
                { mask,    uint32_t, _RTYPE_uint32_t, (uint16_t)offsetof(_curr_comp_t, mask),    (uint16_t)sizeof(((_curr_comp_t *)0)->mask),    1 },
                { trigger, bool,     _RTYPE_bool,     (uint16_t)offsetof(_curr_comp_t, trigger), (uint16_t)sizeof(((_curr_comp_t *)0)->trigger), 1 },
                { _pad,    uint8_t,  _RTYPE_uint8_t,  (uint16_t)offsetof(_curr_comp_t, _pad),    (uint16_t)sizeof(((_curr_comp_t *)0)->_pad),    3 },
            };
            *out_cnt = (uint16_t)(sizeof(fields) / sizeof(*fields));
            return fields;
        }
*/

#define REQUIRES(...)
#define EXCLUDES(...)
#define COMP(NAME)                                                                                 \
    static const field_meta_t *s_get_##NAME##_fields(uint16_t *out_cnt) {                          \
        typedef NAME _curr_comp_t;                                                                 \
        static const field_meta_t fields[] = {
#define _FIELD_META_3(T, NAME,      DEF) { #NAME, #T, _RTYPE_##T, (uint16_t)offsetof(_curr_comp_t, NAME), (uint16_t)sizeof(((_curr_comp_t *)0)->NAME), 1     },
#define _FIELD_META_4(T, NAME, CNT, DEF) { #NAME, #T, _RTYPE_##T, (uint16_t)offsetof(_curr_comp_t, NAME), (uint16_t)sizeof(((_curr_comp_t *)0)->NAME), (CNT) },
#define FIELD(T, NAME, ...)      _FIELD_META_3(T, NAME,      0)
#define ARRAY(T, NAME, CNT, ...) _FIELD_META_4(T, NAME, CNT, 0)
#define COMP_END(NAME)                                                                             \
    };                                                                                             \
        *out_cnt = (uint16_t)(sizeof(fields) / sizeof(*fields));                                   \
        return fields;                                                                             \
    }

#include "components.def"

#undef COMP_END
#undef ARRAY
#undef FIELD
#undef _FIELD_META_4
#undef _FIELD_META_3
#undef COMP
#undef EXCLUDES
#undef REQUIRES

/* ============================================================================================== */
/* >API                                                                                           */
/* ============================================================================================== */

static comp_meta_t s_comp_reg[COMP_CNT];
static bool        s_reg_init   = false;
static res_t       s_reg_result = EINVAL;

/* builds prerequisites and reserve dependencies */
static res_t _reflect_constraints_build(void) {
    for (uint32_t i = 0; i < COMP_CNT; ++i)
        s_comp_reg[i].required_by_mask = 0;

    /* compute closure for prereqs */
    for (uint32_t k = 0; k < COMP_CNT; ++k)
        for (uint32_t i = 0; i < COMP_CNT; ++i)
            if (s_comp_reg[i].requires_mask & BIT(k)) s_comp_reg[i].requires_mask |= s_comp_reg[k].requires_mask;

    for (uint32_t i = 0; i < COMP_CNT; ++i) {
        comp_meta_t *comp = &s_comp_reg[i];

        if (comp->requires_mask & BIT(i)) return EINVAL;

        uint64_t grp = BIT(i) | comp->requires_mask;

        for (uint32_t j = 0; j < COMP_CNT; ++j) {
            if ((grp & BIT(j)) && (s_comp_reg[j].excludes_mask & grp)) return EINVAL;
            if (comp->requires_mask & BIT(j))                          s_comp_reg[j].required_by_mask |= BIT(i);
        }
    }

    return OK;
}

static void _reflect_init_lazy(void) {
    if (s_reg_init) return;

#define REQUIRES(...)
#define EXCLUDES(...)
#define COMP(NAME)                                                                                 \
    do {                                                                                           \
        uint16_t cnt = 0;                                                                          \
        const field_meta_t *fields = s_get_##NAME##_fields(&cnt);                                  \
        s_comp_reg[COMP_##NAME] = (comp_meta_t){                                                   \
            .name      = #NAME,                                                                    \
            .fields    = fields,                                                                   \
            .def_val   = &NAME##_def,                                                              \
            .id        = COMP_##NAME,                                                              \
            .size      = (uint16_t)sizeof(NAME),                                                   \
            .field_cnt = cnt,                                                                      \
        };                                                                                         \
    } while (0);
#define FIELD(...)
#define ARRAY(...)
#define COMP_END(NAME)

#include "components.def"

#undef COMP_END
#undef ARRAY
#undef FIELD
#undef COMP
#undef EXCLUDES
#undef REQUIRES

#define REQUIRES(COMPONENT, PREREQUISITE)                                                          \
    s_comp_reg[COMP_##COMPONENT].requires_mask |= BIT(COMP_##PREREQUISITE);
#define EXCLUDES(A, B)                                                                             \
    do {                                                                                           \
        s_comp_reg[COMP_##A].excludes_mask |= BIT(COMP_##B);                                       \
        s_comp_reg[COMP_##B].excludes_mask |= BIT(COMP_##A);                                       \
    } while (0);
#define COMP(NAME)
#define FIELD(...)
#define ARRAY(...)
#define COMP_END(NAME)

#include "components.def"

#undef COMP_END
#undef ARRAY
#undef FIELD
#undef COMP
#undef EXCLUDES
#undef REQUIRES

    s_reg_result = _reflect_constraints_build();
    s_reg_init   = true;
}

res_t reflect_validate_schema(void) {
    _reflect_init_lazy();
    return s_reg_result;
}

const comp_meta_t *reflect_comp_by_id(comp_id_t id) {
    ASSERT(id < COMP_CNT, "Attempted to grab component by out-of-range ID");
    if (id >= COMP_CNT) return NULL;

    _reflect_init_lazy();
    if (s_reg_result != OK) return NULL;

    return &s_comp_reg[id];
}

const comp_meta_t *reflect_comp_by_name(const char *name) {
    ASSERT(name, "Attempted to grab component by NULL name");
    if (!name) return NULL;

    _reflect_init_lazy();
    if (s_reg_result != OK) return NULL;

    for (size_t i = 0; i < COMP_CNT; ++i)
        if (strcmp(s_comp_reg[i].name, name) == 0) return &s_comp_reg[i];

    return NULL;
}

uint32_t reflect_comp_cnt(void) {
    return COMP_CNT;
}
