/**
 * @file    resources.h
 * @brief   Public interface of resources.c (BARR-C 4.2.a: one header per
 *          source file).
 */
#ifndef RESOURCES_H
#define RESOURCES_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
#include "car.h"

IMPORT void resources_report (void); /* boot table: what really exists */

IMPORT void resources_usage (
    res_usage_t * p_res); /* ~100 kernel calls: once a second */

IMPORT ER resources_start (void); /* the two idle-time meters */

#endif /* RESOURCES_H */

/*** end of file ***/
