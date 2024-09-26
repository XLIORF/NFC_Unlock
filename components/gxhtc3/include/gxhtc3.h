#pragma once

#ifdef __cplusplus
exern "C" {
#endif


int gxhtc3_init();
void gxhtc3_reset();
void gxhtc3_sleep();
void gxhtc3_wakeup();
int gxhtc3_measure(float *temperature, float *humanity);

#ifdef __cplusplus
}
#endif