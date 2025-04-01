/*******************************************************************************
 *
 *  system_settings.h - header file for system settings of the task scheduler
 *                      declarations used in TIDA-00449
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 ******************************************************************************/

#ifndef SYSTEM_SETTINGS_H_
#define SYSTEM_SETTINGS_H_
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <msp430.h>

typedef float float32_t;

typedef enum {
    FALSE,
    TRUE
}enum_logic_t;

#define CPU_FREQ_MHZ                    (16)

#define MAX_GLOBAL_TIMING_COUNT         ((uint32_t)(0x9A7EC800))        // 2,592,000,000 (30 days)
#define INTERVAL_1MS                    ((uint16_t)(1U))
#define INTERVAL_2MS                    ((uint16_t)(2U))
#define INTERVAL_10MS                   ((uint16_t)(10U))
#define INTERVAL_100MS                  ((uint16_t)(100U))
#define INTERVAL_1000MS                 ((uint16_t)(1000U))

#define MAX_TASK_NUMBERS                ((uint8_t)(3U))                // Number of tasks running

#define delay_1us(v)                     __delay_cycles(CPU_FREQ_MHZ*v)
#define delay_1ms(v)                     __delay_cycles(CPU_FREQ_MHZ*1000*v)

extern uint8_t Flag_1MS;
extern uint32_t Global_Timing_Counter;

//calibration data
extern uint8_t setting_FECU_Side;

#endif /* SYSTEM_SETTINGS_H_ */
