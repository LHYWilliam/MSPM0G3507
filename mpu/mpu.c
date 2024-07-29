#include <stdint.h>

#include "mpu.h"
#include "mpui2c.h"

void MPU_Init() {
    MPUI2C_Init();

    uint8_t PWR_MGMT_1 = 0x01;
    uint8_t PWR_MGMT_2 = 0x00;

    uint8_t SMPLRT_DIV = 0x09;

    uint8_t CONFIG = 0x06;
    uint8_t GYRO_CONFIG = 0x18;
    uint8_t ACCEL_CONFIG = 0x18;

    MPU_Send(MPU6050_PWR_MGMT_1, 1, &PWR_MGMT_1);
    MPU_Send(MPU6050_PWR_MGMT_2, 1, &PWR_MGMT_2);

    MPU_Send(MPU6050_SMPLRT_DIV, 1, &SMPLRT_DIV);

    MPU_Send(MPU6050_CONFIG, 1, &CONFIG);
    MPU_Send(MPU6050_ACCEL_CONFIG, 1, &ACCEL_CONFIG);
    MPU_Send(MPU6050_GYRO_CONFIG, 1, &GYRO_CONFIG);
}

void MPU_AdaptOffset(uint16_t times, int16_t *xacc_offset, int16_t *yacc_offset,
                     int16_t *xgyro_offset, int16_t *ygyro_offset,
                     int16_t *zgyro_offset) {
    int16_t xacc, yacc, zacc, xgyro, ygyro, zgyro;
    int64_t xacc_sum = 0, yacc_sum = 0, xgyro_sum = 0, ygyro_sum = 0,
            zgyro_sum = 0;

    for (uint16_t i = 0; i < times; i++) {
        MPU_GetData(&xacc, &yacc, &zacc, &xgyro, &ygyro, &zgyro);
        xacc_sum += xacc;
        yacc_sum += yacc;
        xgyro_sum += xgyro;
        ygyro_sum += ygyro;
        zgyro_sum += zgyro;
    }

    *xacc_offset = (int16_t)(xacc_sum / times);
    *yacc_offset = (int16_t)(yacc_sum / times);
    *xgyro_offset = (int16_t)(xgyro_sum / times);
    *ygyro_offset = (int16_t)(ygyro_sum / times);
    *zgyro_offset = (int16_t)(zgyro_sum / times);
}

void MPU_GetData(int16_t *xacc, int16_t *yacc, int16_t *zacc, int16_t *xgyro,
                 int16_t *ygyro, int16_t *zgyro) {
    uint8_t acc[6], gyro[6];
    MPU_Receieve(MPU6050_ACCEL_XOUT_H, 6, acc);
    MPU_Receieve(MPU6050_GYRO_XOUT_H, 6, gyro);

    *xacc = (acc[0] << 8) | acc[1];
    *yacc = (acc[2] << 8) | acc[3];
    *zacc = (acc[4] << 8) | acc[5];
    *xgyro = (gyro[0] << 8) | gyro[1];
    *ygyro = (gyro[2] << 8) | gyro[3];
    *zgyro = (gyro[4] << 8) | gyro[5];
}

void MPU_Send(uint8_t RegisterAddress, uint8_t length, uint8_t *bytes) {
    MPUI2C_Send(MPU6050_DEVICE_ADDRESS, RegisterAddress, length, bytes);
}
void MPU_Receieve(uint8_t RegisterAddress, uint8_t length, uint8_t *bytes) {
    MPUI2C_Receive(MPU6050_DEVICE_ADDRESS, RegisterAddress, length, bytes);
}