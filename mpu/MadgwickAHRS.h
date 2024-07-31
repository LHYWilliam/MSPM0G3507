//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
//#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration

//-------------------------------------------------------------------------------------------
// Function declarations

#include "ti_msp_dl_config.h"

typedef struct SensorMsg {
	float A[3];
	float G[3];
} SensorMsg;

typedef struct MPU6050Params {
	uint8_t MPU6050dt;
	uint64_t preMillis;
	float MPU6050ERROE[6];
	
	
} MPU6050Params;

void begin(float sampleFrequency);
void computeAngles(void);

void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
//float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
//float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
//float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
float getRoll(void);
float getPitch(void);
float getYaw(void);
float getRollRadians(void);
float getPitchRadians(void);
float getYawRadians(void);

void getMPU6050Data();
void dataGetAndFilter();
void MPU_getError();

#endif

