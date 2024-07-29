#include "ti_msp_dl_config.h"

#include "pid.h"

float RC = 1 / (2 * 3.14 * 20);

extern uint32_t ms;

void PID_Init(PID *pid) {
    pid->NaN = RESET;
    pid->last_time = 0;
    pid->last_error = 0.;
    pid->integrator = 0.;
    pid->last_derivative = 0.;
}

int16_t PID_Caculate(PID *pid, float error) {
    float output = 0;
    uint32_t now = ms;
    float dt = (float)(now - pid->last_time) / 1000;

    if (pid->last_time == 0 || dt > 1) {
        pid->integrator = dt = 0;
        pid->NaN = SET;
    }
    pid->last_time = now;

    output += error * pid->Kp;

    if (pid->Kd && dt) {
        float derivative;
        if (pid->NaN == SET) {
            derivative = 0;
            pid->last_derivative = 0;
            pid->NaN = RESET;
        } else {
            derivative = (error - pid->last_error) / dt;
        }

        derivative = pid->last_derivative +
                     (dt / (RC + dt)) * (derivative - pid->last_derivative);
        pid->last_error = error;
        pid->last_derivative = derivative;

        output += pid->Kd * derivative;
    }

    if (pid->Ki && dt) {
        pid->integrator += error * pid->Ki * dt;
        LIMIT(pid->integrator, -pid->imax, pid->imax);

        output += pid->integrator;
    }

    return (int32_t)output;
}