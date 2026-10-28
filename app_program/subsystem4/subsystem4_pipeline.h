/**
 * @file subsystem4_pipeline.h
 * @brief Three-stage Buddy 4 pipeline and emergency event signaling API.
 */
#ifndef SUBSYSTEM4_PIPELINE_H
#define SUBSYSTEM4_PIPELINE_H

#include "subsystem4.h"

/* May be called from a verified ISR; only signals the acquisition semaphore. */
void Subsystem4_Notify_Data_Ready_From_ISR(void);

/* Creates message buffers/semaphore/eventflag and starts the three pipeline tasks. */
ER Subsystem4_Start_Pipeline_Tasks(void);

/* Returns the RTOS event-flag ID used for immediate 20 ms SUDDEN_IMPACT signaling. */
ID Subsystem4_Get_System_Event_Flag_ID(void);

#endif /* SUBSYSTEM4_PIPELINE_H */