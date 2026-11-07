#pragma once

/*
====================
bonsai.h

Primary umbrella header for bonsai-c.
Exposes engine version metadata and aggregates all public core subsystems.
====================
*/

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================================== */
/* >VERSION                                                                                       */
/* ============================================================================================== */

#define BONSAI_VERSION_MAJOR 0
#define BONSAI_VERSION_MINOR 1
#define BONSAI_VERSION_PATCH 0

#define _BONSAI_STR_IMPL(x)  #x
#define _BONSAI_STR(x)       _BONSAI_STR_IMPL(x)

/*
====================
BONSAI_VERSION_STR

Semantic version string of the engine in `MAJOR.MINOR.PATCH` format.
====================
*/
#define BONSAI_VERSION_STR _BONSAI_STR(BONSAI_VERSION_MAJOR) "." _BONSAI_STR(BONSAI_VERSION_MINOR) "." _BONSAI_STR(BONSAI_VERSION_PATCH)

/* ============================================================================================== */
/* >CORE SUBSYSTEMS                                                                               */
/* ============================================================================================== */

#include "core.h"
#include "arena.h"
#include "gmath.h"
#include "reflect.h"
#include "ecs.h"
#include "spatial.h"

#ifdef __cplusplus
}
#endif
