#ifndef UKF_MRP_H
#define UKF_MRP_H

#define SIZE_STATE 6
#define SIZE_MEASUREMENT 9
#define GYRO_VARIANCE 0.193825 // 11.111... degress in radians
#define POSITION_VARIANCE 0.1

#define ALPHA 0.001
#define BETA 2.0
#define KAPPA 0.0 // (double)(3 - SIZE_STATE)

#define POWER_ITERATIONS 100

void ukf_run(double *result_state, double *measurement, double delta_t);

#endif
