#pragma once

/*
====================
reflect.h

Single-source reflection metadata subsystem for the ECS.
Expands `components.def` via a hierarchical preprocessor DSL to generate:
- C component struct definitions
- component enumeration IDs
- default component prototype instances
- runtime field and component reflection descriptors
====================
*/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
#define FIELD(...)                       VA_JOIN(_FIELD_DECL_, VA_NARGS(__VA_ARGS__))(__VA_ARGS__)
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

/*
====================
comp_id_t

Enumerated component identifier index.
Used for sparse-set pool indexing and 64-bit component bitmasks.
Terminated by `COMP_CNT` representing total registered component count.
====================
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

/*
====================
reflect_type_t

Primitive data type enumeration for reflection
inspection and script marshaling.
====================
*/
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

/*
====================
field_meta_t

Field reflection metadata descriptor.
Stores:
- field name `name`      (string identifier,     e.g. "x")
- type name  `type_name` (C type representation, e.g. "float")
- type tag   `type`      (primitive enum `reflect_type_t`)
- offset     `off`       (byte offset within component struct via `offsetof`)
- byte size  `size`      (total byte size of field via `sizeof`)
- count      `cnt`       (element count: 1 for scalars, >1 for fixed-size arrays)
====================
*/
typedef struct field_meta {
    const char    *name;
    const char    *type_name;
    reflect_type_t type;
    uint16_t       off;
    uint16_t       size;
    uint16_t       cnt;
} field_meta_t;

/*
====================
comp_meta_t

Component reflection metadata descriptor.
Stores:
- component name `name`      (string identifier, e.g. "trans2d")
- field list     `fields`    (pointer to array of `field_meta_t` descriptors)
- default value  `def_val`   (pointer to read-only default struct instance)
- component ID   `id`        (numeric identifier `comp_id_t`)
- struct size    `size`      (total byte size via `sizeof`)
- field count    `field_cnt` (number of entries in `fields` array)
====================
*/
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

/*
====================
reflect_comp_by_id

Retrieves component reflection metadata by `id` ID.
Lazy-initializes registry on first call.

Returns:
- pointer to `comp_meta_t` descriptor if ID is valid.
- `NULL` otherwise.
====================
*/
const comp_meta_t *reflect_comp_by_id(comp_id_t id);

/*
====================
reflect_comp_by_name

Searches for component reflection metadata by `name` string.

Returns:
- pointer to `comp_meta_t` descriptor if component was found.
- `NULL` otherwise.
====================
*/
const comp_meta_t *reflect_comp_by_name(const char *name);

/*
====================
reflect_comp_cnt

Retrieves the total number of registered ECS components.

Returns component count `COMP_CNT`.
====================
*/
uint32_t reflect_comp_cnt(void);
