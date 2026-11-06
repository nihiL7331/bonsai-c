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
            ._pad    = ({0}),
        };
*/

#define COMP(NAME)                      const NAME NAME##_def = {
#define _FIELD_DEF_3(T, NAME,      DEF) .NAME = (DEF),
#define _FIELD_DEF_4(T, NAME, CNT, DEF) .NAME = DEF,
#define FIELD(...)                      _FIELD_SELECT(__VA_ARGS__, _FIELD_DEF_4, _FIELD_DEF_3, 0)(__VA_ARGS__)
#define COMP_END(NAME)                  };

#include "components.def"

#undef COMP_END
#undef FIELD
#undef _FIELD_DEF_4
#undef _FIELD_DEF_3
#undef COMP

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

#define COMP(NAME)                                                                                 \
    static const field_meta_t *s_get_##NAME##_fields(uint16_t *out_cnt) {                          \
        typedef NAME _curr_comp_t;                                                                 \
        static const field_meta_t fields[] = {
#define _FIELD_META_3(T, NAME,      DEF) { #NAME, #T, _RTYPE_##T, (uint16_t)offsetof(_curr_comp_t, NAME), (uint16_t)sizeof(((_curr_comp_t *)0)->NAME), 1     },
#define _FIELD_META_4(T, NAME, CNT, DEF) { #NAME, #T, _RTYPE_##T, (uint16_t)offsetof(_curr_comp_t, NAME), (uint16_t)sizeof(((_curr_comp_t *)0)->NAME), (CNT) },
#define FIELD(...)                       _FIELD_SELECT(__VA_ARGS__, _FIELD_META_4, _FIELD_META_3, 0)(__VA_ARGS__)
#define COMP_END(NAME)                                                                             \
    };                                                                                             \
        *out_cnt = (uint16_t)(sizeof(fields) / sizeof(*fields));                                   \
        return fields;                                                                             \
    }

#include "components.def"

#undef COMP_END
#undef FIELD
#undef _FIELD_META_4
#undef _FIELD_META_3
#undef COMP

/* ============================================================================================== */
/* >API                                                                                           */
/* ============================================================================================== */

static comp_meta_t s_comp_reg[COMP_CNT];
static bool        s_reg_init = false;

static void _reflect_init_lazy(void) {
    if (s_reg_init) return;
    s_reg_init = true;

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
#define COMP_END(NAME)

#include "components.def"

#undef COMP_END
#undef FIELD
#undef COMP
}

const comp_meta_t *reflect_comp_by_id(comp_id_t id) {
    ASSERT(id < COMP_CNT, "Attempted to grab component by out-of-range ID");
    if (id >= COMP_CNT) return NULL;
    reflect_init_lazy();

    return &s_comp_reg[id];
}

const comp_meta_t *reflect_comp_by_name(const char *name) {
    ASSERT(name, "Attempted to grab component by NULL name");
    if (!name) return NULL;
    reflect_init_lazy();

    for (size_t i = 0; i < COMP_CNT; ++i)
        if (strcmp(s_comp_reg[i].name, name) == 0) return &s_comp_reg[i];

    return NULL;
}

uint32_t reflect_comp_cnt(void) {
    return COMP_CNT;
}
