/**
 * @file    app_main.h
 * @brief   Public interface of app_main.c (BARR-C 4.2.a: one header per
 *          source file).
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h> /* Fixed-width integer types (BARR-C 5.2). */
#include "app.h"

IMPORT ID app_main_task_start (FP entry, PRI pri, uint32_t prc,
                               const char * p_name);

#endif /* APP_MAIN_H */

/*** end of file ***/
