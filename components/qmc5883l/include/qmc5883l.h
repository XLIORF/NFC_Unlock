#pragma once
#include <stdint.h>

typedef enum OSR_e {
    OSR_512 = 0,
    OSR_256,
    OSR_128,
    OSR_64,
} OSR_e;

typedef enum RNG_e{
    RNG_2G = 0,
    RNG_8G,
} RNG_e;

typedef enum ODR_e{
    ODR_10Hz = 0,
    ODR_50Hz,
    ODR_100Hz,
    ODR_200Hz,
} ODR_e;

typedef enum MODE_e{
    MODE_Standby = 0,
    MODE_Continuous,
} MODE_e;

#ifdef __cplusplus
exern "C" {
#endif

void qmc5883l_init(MODE_e mode, ODR_e odr, RNG_e rng, OSR_e osr);
void qmc5883l_get_data(float *x, float *y, float *z);
float qmc5883l_get_angle();

void qmc5883l_reset();
void qmc5883l_pointer_rollOver(uint8_t en);
void qmc5883l_intr(uint8_t en);

#ifdef __cplusplus
}
#endif