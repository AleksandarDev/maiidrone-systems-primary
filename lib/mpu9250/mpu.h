#ifndef MPU_H
#define MPU_H

#include <Arduino.h>

struct s_mympu {
	float ypr[3];
	float gyro[3];
  float comp[3];
};

extern struct s_mympu mympu;

int8_t mympu_open(unsigned int rate,unsigned short orient);
int8_t mympu_update();
int8_t mympu_update_compass();

#endif
