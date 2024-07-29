#ifndef __PID_H
#define __PID_H

#include "ti_msp_dl_config.h"

#define LIMIT(data, low, high)                                                 \
    do {                                                                       \
        data = data < low ? low : data;                                        \
        data = data > high ? high : data;                                      \
    } while (0)

#define RESET 0	
#define SET   1

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    uint16_t imax;

    float last_error, integrator;
    float last_derivative;
    uint32_t last_time;

    uint8_t NaN;
} PID;

void PID_Init(PID *pid);

int16_t PID_Caculate(PID *pid, float error);

#endif