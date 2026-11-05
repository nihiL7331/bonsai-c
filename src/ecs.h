#pragma once

#include "arena.h"
#include "reflect.h"

#include <stdbool.h>
#include <stdint.h>

typedef uint64_t entity_handle_t; /* [ 32b gen | 16b flags | 16b idx ] */

#define ECS_MAX_ENTITIES  65535U

#define ECS_NULL_IDX      0xFFFFU
#define ENTITY_NULL       ((entity_handle_t)0ULL)
#define ENTITY_FLAG_DEAD  (1U << 0)

#define ENTITY_GEN_MASK   0xFFFFFFFF00000000ULL
#define ENTITY_FLAGS_MASK 0x00000000FFFF0000ULL
#define ENTITY_IDX_MASK   0x000000000000FFFFULL

#define entity_gen(h)   (uint32_t)(((h) & ENTITY_GEN_MASK)   >> 32)
#define entity_flags(h) (uint16_t)(((h) & ENTITY_FLAGS_MASK) >> 16)
#define entity_idx(h)   (uint16_t)( (h) & ENTITY_IDX_MASK)

#define entity_handle(gen, flags, idx) ((entity_handle_t)                                          \
    ((((entity_handle_t)(gen)   << 32) & ENTITY_GEN_MASK)   |                                      \
     (((entity_handle_t)(flags) << 16) & ENTITY_FLAGS_MASK) |                                      \
     (((entity_handle_t)(idx)        ) & ENTITY_IDX_MASK)))

#define ECS_CMD_INIT_CAP 256UL

/* ============================================================================================== */
/* >STRUCTS                                                                                       */
/* ============================================================================================== */

typedef enum ecs_cmd_type {
    ECS_CMD_DESTROY_ENT,
    ECS_CMD_DELETE_COMP,
} ecs_cmd_type_t;

typedef struct ecs_cmd {
    ecs_cmd_type_t  type;
    entity_handle_t ent;
    comp_id_t       comp; /* for ECS_CMD_DELETE_COMP */
} ecs_cmd_t;

typedef struct ecs_pool {
    void    *dense;
    uint32_t cnt;
    uint16_t elem_sz;
    uint16_t sparse[ECS_MAX_ENTITIES + 1];   /* ent_idx -> dense_idx, ECS_NULL_IDX inc. */
    uint16_t entities[ECS_MAX_ENTITIES + 1]; /* dense_idx -> ent_idx                    */
    bool     dirty;
} ecs_pool_t;

typedef struct entity_mgr {
    uint64_t comp_mask[ECS_MAX_ENTITIES + 1]; /* (comp_mask[i] >> k) & 1 => ent i owns comp k */
    uint32_t gens[ECS_MAX_ENTITIES + 1];      /* 0=inactive                                   */
    uint32_t free_hd;
    uint32_t alive_cnt;
    uint16_t free_list[ECS_MAX_ENTITIES + 1];
    uint16_t flags[ECS_MAX_ENTITIES + 1];
} entity_mgr_t;

typedef struct ecs_world {
    arena_t      *perm;    /* permanent lifetime */
    arena_t      *backup;  /* mirror of 'perm'   */
    arena_t      *scratch; /* per-frame          */
    entity_mgr_t mgr;
    ecs_pool_t   pools[COMP_CNT];
    ecs_cmd_t   *cmd_buf; /* scratch-alloced */
    uint32_t     cmd_cnt;
    uint32_t     cmd_cap;
} ecs_world_t;

typedef struct ecs_view_row {
    entity_handle_t ent;
    uint16_t        dense_idxs[4];
} ecs_view_row_t;

typedef struct ecs_view {
    ecs_view_row_t *rows;
    uint32_t        cnt;
} ecs_view_t;

/* ============================================================================================== */
/* >API                                                                                           */
/* ============================================================================================== */

ecs_world_t *ecs_world_create(arena_t *perm, arena_t *backup, arena_t *scratch);
void         ecs_set_world(ecs_world_t *world);
ecs_world_t *ecs_get_world(void);

entity_handle_t ecs_spawn(void);                 /* immediately reserves living handle with inc gen */
entity_handle_t ecs_find_tag(const char *tag);   /* returns first found entity with matching tag    */
void            ecs_destroy(entity_handle_t e);  /* marks dead immediately, defers cleanup to sync  */
bool            ecs_is_alive(entity_handle_t e); /* verifies gen and ensures not marked for death   */
void            ecs_sync(void);                  /* flushes deferred dels & pool cleanup            */

/* NOTE: querying by component works for up to 4 components / mask */
ecs_view_t ecs_query_filter(uint64_t with_mask, uint64_t without_mask); /* returns dense idxs for entities matching 'with_mask' excluding 'without_mask' */
#define    _ecs_query_mask1(with)                            ecs_query_filter((with), 0ULL)
#define    _ecs_query_mask2(with, without)                   ecs_query_filter((with), (without))
#define    _ecs_query_mask_select(_1, _2, NAME, ...)         NAME
/* variadic 1-2 param query-by-mask:                           */
/* - if 1 arg  provided acts like ecs_query_filter(arg1, 0),   */
/* - if 2 args provided acts like ecs_query_filter(arg1, arg2) */
#define    ecs_query_mask(...)                               _ecs_query_mask_select(__VA_ARGS__, _ecs_query_mask2, _ecs_query_mask1, 0)(__VA_ARGS__)

#define    _ecs_query_comp1(t1)                              ecs_query_mask(1ULL << COMP_##t1)
#define    _ecs_query_comp2(t1, t2)                          ecs_query_mask((1ULL << COMP_##t1) | (1ULL << COMP_##t2))
#define    _ecs_query_comp3(t1, t2, t3)                      ecs_query_mask((1ULL << COMP_##t1) | (1ULL << COMP_##t2) | (1ULL << COMP_##t3))
#define    _ecs_query_comp4(t1, t2, t3, t4)                  ecs_query_mask((1ULL << COMP_##t1) | (1ULL << COMP_##t2) | (1ULL << COMP_##t3) | (1ULL << COMP_##t4))
#define    _ecs_query_comp_select(_1, _2, _3, _4, NAME, ...) NAME
/* variadic 1-4 param query-by-component, returns dense idxs for entities matching */
#define    ecs_query_comp(...)                               _ecs_query_comp_select(__VA_ARGS__, _ecs_query_comp4, _ecs_query_comp3, _ecs_query_comp2, _ecs_query_comp1, 0)(__VA_ARGS__)

void *ecs_get_raw(entity_handle_t e, comp_id_t id);                  /* returns component ptr (NULL if missing/dead)    */
bool  ecs_has_raw(entity_handle_t e, comp_id_t id);                  /* true if living entity owns component 'id'       */
void *ecs_add_raw(entity_handle_t e, comp_id_t id, const void *val); /* attaches immediately, if val=NULL uses defaults */
void  ecs_del_raw(entity_handle_t e, comp_id_t id);                  /* queues deferred component removal for sync      */

#define ecs_get(entity_handle, type)      ((type *)ecs_get_raw((entity_handle), COMP_##type))
#define ecs_has(entity_handle, type)               ecs_has_raw((entity_handle), COMP_##type)
#define ecs_add(entity_handle, type, val) ((type *)ecs_add_raw((entity_handle), COMP_##type, (val)))
#define ecs_del(entity_handle, type)               ecs_del_raw((entity_handle), COMP_##type)
