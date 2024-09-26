#pragma once

typedef enum QMI_ODR {
    QMI_ODR_NA_7520_NORMAL = 0,
} QMI_ODR;


#ifdef __cplusplus
exern "C" {
#endif

void qmi8658_test();
void qmi8658c_reset();
int qmi8658c_init();

void qmi8658c_get_acce(float *x, float *y, float *z);
void qmi8658c_get_gyro(float *x, float *y, float *z);


void qmi8658c_get_quaternion(float *dqw, float *dqx, float *dqy, float *dqz);



void qmi8658c_get_temp(float *temp);

#ifdef __cplusplus
}
#endif