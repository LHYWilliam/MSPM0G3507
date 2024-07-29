#ifndef __DMP_H
#define __DMP_H

#include "inv_mpu.h"

#define MPU6050

#define ACCEL_ON (0x01)
#define GYRO_ON (0x02)
#define DEFAULT_MPU_HZ (200)

#define q30 1073741824.0f

static inline unsigned short inv_row_2_scale(const signed char *row) {
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;
    return b;
}

static inline unsigned short
inv_orientation_matrix_to_scalar(const signed char *mtx) {
    unsigned short scalar;

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

static inline void run_self_test(void) {
    int result;
    char test_packet[4] = {0};
    long gyro[3], accel[3];
    unsigned char i = 0;

    result = mpu_run_self_test(gyro, accel);

    if (result == 0x7) {
        for (i = 0; i < 3; i++) {
            gyro[i] = (long)(gyro[i] * 32.8f);
            accel[i] *= 2048.f;
            accel[i] = accel[i] >> 16;
            gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);
        mpu_set_accel_bias_6050_reg(accel);
    }
}

void DMP_Init();
void DMP_GetData(float *pitch, float *roll, float *yaw);

#endif