/**
 * @file subsystem4_pipeline.h
 * @brief Three-stage Buddy 4 pipeline API.
 *
 * ISR integration note:
 * Call Subsystem4_Notify_Data_Ready_From_ISR() only from the confirmed
 * GY-511 data-ready GPIO interrupt handler. Do not enable this hook until
 * the DRDY pin and μT-Kernel ISR-callable API are verified for the project.
 */
#ifndef SUBSYSTEM4_PIPELINE_H
#define SUBSYSTEM4_PIPELINE_H

#include "subsystem4.h"

/* May be called from a verified ISR; only signals the acquisition semaphore. */
void Subsystem4_Notify_Data_Ready_From_ISR(void);

/* Creates message buffers/semaphore and starts the three pipeline tasks. */
ER Subsystem4_Start_Pipeline_Tasks(void);

#endif /* SUBSYSTEM4_PIPELINE_H */
