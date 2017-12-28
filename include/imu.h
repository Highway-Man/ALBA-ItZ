#ifndef __IMU_H__
#define __IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

void imuInit();
void imuShutdown();
void imuUpdate();
double imuGet();

#ifdef __cplusplus
}
#endif

#endif
