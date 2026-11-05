#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define _FIELD_SELECT(_1, _2, _3, _4, NAME, ...) NAME

/* ============================================================================================== */
/* >TYPEDEF STRUCT GEN                                                                            */
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

        typedef struct collider2d {
            float    half_w;
            float    half_h;
            float    off_x;
            float    off_y;
            uint32_t mask;
            bool     trigger;
            uint8_t _pad[3];
        } collider2d;
*/

#define COMP(NAME)                       typedef struct NAME {
#define _FIELD_DECL_3(T, NAME,      DEF) T NAME;
#define _FIELD_DECL_4(T, NAME, CNT, DEF) T NAME[CNT];
#define FIELD(...)                       _FIELD_SELECT(__VA_ARGS__, _FIELD_DECL_4, _FIELD_DECL_3, 0)(__VA_ARGS__)
#define COMP_END(NAME)                   } NAME;

#include "components.def"

#undef COMP_END
#undef FIELD
#undef _FIELD_DECL_4
#undef _FIELD_DECL_3
#undef COMP

/* ============================================================================================== */
/* >COMP ID ENUM GEN                                                                              */
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

        COMP_collider2d,
*/

typedef enum comp_id {
#define COMP(NAME) COMP_##NAME,
#define FIELD(...)
#define COMP_END(NAME)

#include "components.def"

#undef COMP_END
#undef FIELD
#undef COMP
    COMP_CNT,
} comp_id_t;

/* ============================================================================================== */
/* >DEF STRUCT DECL GEN                                                                           */
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

        extern const collider2d collider2d_def;
*/

#define COMP(NAME) extern const NAME NAME##_def;
#define FIELD(...)
#define COMP_END(NAME)

#include "components.def"

#undef COMP_END
#undef FIELD
#undef COMP

/* ============================================================================================== */
/* >REFLECT META STRUCTS                                                                          */
/* ============================================================================================== */

typedef enum reflect_type {
    TYPE_F32,
    TYPE_F64,
    TYPE_I8,
    TYPE_I16,
    TYPE_I32,
    TYPE_I64,
    TYPE_U8,
    TYPE_U16,
    TYPE_U32,
    TYPE_U64,
    TYPE_BOOL,
    TYPE_STR,
    TYPE_ENTITY,
    TYPE_CNT,
} reflect_type_t;

typedef struct field_meta {
    const char    *name;
    const char    *type_name;
    reflect_type_t type;
    uint16_t       off;
    uint16_t       size;
    uint16_t       cnt;
} field_meta_t;

typedef struct comp_meta {
    const char          *name;
    const field_meta_t *fields;
    const void         *def_val;
    comp_id_t           id;
    uint16_t            size;
    uint16_t            field_cnt;
} comp_meta_t;

/* ============================================================================================== */
/* >API                                                                                           */
/* ============================================================================================== */

const comp_meta_t *reflect_comp_by_id(comp_id_t id);       /* given a components ID,   return its struct */
const comp_meta_t *reflect_comp_by_name(const char *name); /* given a components name, return its struct */
uint32_t           reflect_comp_cnt(void);                 /* return # of available components           */
