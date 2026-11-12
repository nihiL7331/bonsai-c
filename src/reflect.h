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

#include "core.h"
#include "gmath.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint64_t    entity_handle_t;
#define ENTITY_NULL ((entity_handle_t)0ULL)

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

#define REQUIRES(...)
#define EXCLUDES(...)
#define COMP(NAME)                       typedef struct NAME {
#define FIELD(T, NAME, ...)              T NAME;
#define ARRAY(T, NAME, CNT, ...)         T NAME[CNT];
#define COMP_END(NAME)                   } NAME;

#include "components.def"

#undef COMP_END
#undef ARRAY
#undef FIELD
#undef COMP
#undef EXCLUDES
#undef REQUIRES

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
#define REQUIRES(...)
#define EXCLUDES(...)
#define COMP(NAME) COMP_##NAME,
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
    COMP_CNT,
} comp_id_t;

_Static_assert(COMP_CNT <= 64, "Component masks support at most 64 types");

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

#define REQUIRES(...)
#define EXCLUDES(...)
#define COMP(NAME) extern const NAME NAME##_def;
#define FIELD(T, NAME, ...)
#define ARRAY(T, NAME, CNT, ...)
#define COMP_END(NAME)

#include "components.def"

#undef COMP_END
#undef ARRAY
#undef FIELD
#undef COMP
#undef EXCLUDES
#undef REQUIRES

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
    TYPE_VEC2F,
    TYPE_VEC3F,
    TYPE_VEC4F,
    TYPE_VEC2I,
    TYPE_VEC3I,
    TYPE_VEC4I,
    TYPE_AABB2F,
    TYPE_AABB3F,
    TYPE_AABB2I,
    TYPE_AABB3I,
    TYPE_MAT3F,
    TYPE_MAT4F,
    TYPE_QUATF,
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
- component name  `name`             (string identifier, e.g. "trans2d")
- field list      `fields`           (pointer to array of `field_meta_t` descriptors)
- default value   `def_val`          (pointer to read-only default struct instance)
- dependency mask `requires_mask`    (direct and transitive prerequisite component bits)
- banned mask     `excludes_mask`    (symmetrically excluded component bits)
- dependent mask  `required_by_mask` (components depending directly or transitively on this component)
- component ID    `id`               (numeric identifier `comp_id_t`)
- struct size     `size`             (total byte size via `sizeof`)
- field count     `field_cnt`        (number of entries in `fields` array)
====================
*/
typedef struct comp_meta {
    const char          *name;
    const field_meta_t *fields;
    const void         *def_val;
    uint64_t            requires_mask;
    uint64_t            excludes_mask;
    uint64_t            required_by_mask;
    comp_id_t           id;
    uint16_t            size;
    uint16_t            field_cnt;
} comp_meta_t;

/* ============================================================================================== */
/* >COMPONENT CONSTRUCTORS                                                                        */
/* ============================================================================================== */

#define trans2d(POSITION, ROTATION, SCALE) ((trans2d){ POSITION, ROTATION, SCALE })
#define trans3d(POSITION, ROTATION, SCALE) ((trans3d){ POSITION, ROTATION, SCALE })
#define sprite2d(ID, LAYER, FLIP_MASK, COLOR) ((sprite2d){ ID, LAYER, FLIP_MASK, COLOR })
#define mesh3d(MESH_ID, TEX_ID, COLOR) ((mesh3d){ MESH_ID, TEX_ID, COLOR })
#define light3d(TYPE, COLOR, INTENSITY, RANGE, CONE_IN, CONE_OUT) ((light3d){ TYPE, {0}, COLOR, INTENSITY, RANGE, CONE_IN, CONE_OUT })
#define collider2d(HALF_EXTENTS, OFFSET, MASK, TRIGGER, SHAPE) ((collider2d){ HALF_EXTENTS, OFFSET, MASK, TRIGGER, SHAPE, {0} })
#define collider3d(HALF_EXTENTS, OFFSET, MASK, TRIGGER, SHAPE) ((collider3d){ HALF_EXTENTS, OFFSET, MASK, TRIGGER, SHAPE, {0} })
#define rb2d(VELOCITY, ANGULAR_VELOCITY, INV_MASS, DRAG) ((rb2d){ VELOCITY, ANGULAR_VELOCITY, INV_MASS, DRAG })
#define rb3d(VELOCITY, ANGULAR_VELOCITY, INV_MASS, DRAG, ANG_DRAG) ((rb3d){ VELOCITY, ANGULAR_VELOCITY, INV_MASS, DRAG, ANG_DRAG })
#define tag(NAME) ((tag){ NAME })
#define hierarchy(PARENT, FIRST_CHILD, PREV_SIBLING, NEXT_SIBLING, CHILD_CNT) ((hierarchy){ PARENT, FIRST_CHILD, PREV_SIBLING, NEXT_SIBLING, CHILD_CNT })

/* ============================================================================================== */
/* >API                                                                                           */
/* ============================================================================================== */

/*
====================
reflect_validate_schema

Initializes and validates the compiled component schema.
Boundary validation returns an error in both debug and release.
The result is cached because the schema is immutable.

Returns:
- `OK`     if dependencies and exclusions are consistent.
- `EINVAL` if prerequisites contain a cycle, a component excludes itself,
           or a component's required group contains mutually exclusive members.
====================
*/
res_t reflect_validate_schema(void);

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
