/**
 * @file subsystem4_pipeline.h
 * @brief Three-stage Buddy 4 pipeline, ISR hooks, and emergency event signaling API.
 *        Complies with BARR-C:2018.
 */
#ifndef SUBSYSTEM4_PIPELINE_H
#define SUBSYSTEM4_PIPELINE_H

#include "subsystem4.h"

/**
 * @brief Hardware interrupt notification callback.
 *        Called exclusively from the GPIO edge ISR.
 */
void Subsystem4_Notify_Data_Ready_From_ISR(void);

/**
 * @brief Initializes GP3 as an input with pull-down and registers the rising-edge IRQ callback.
 */
ER Subsystem4_Init_Interrupts(void);

/**
 * @brief Creates message buffers, semaphores, event flags, and starts pipeline tasks.
 */
ER Subsystem4_Start_Pipeline_Tasks(void);

/**
 * @brief Returns the RTOS event-flag ID used for immediate 20 ms SUDDEN_IMPACT signaling.
 */
ID Subsystem4_Get_System_Event_Flag_ID(void);

#endif /* SUBSYSTEM4_PIPELINE_H */