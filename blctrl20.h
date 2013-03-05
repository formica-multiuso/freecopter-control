#include <stdint.h>

#define MOTOR1_ADDR 0x29       // Motors i2c Addresses 
#define MOTOR2_ADDR 0x2a
#define MOTOR3_ADDR 0x2b
#define MOTOR4_ADDR 0x2c


void blctrl20_init(void);
void blctrl20_set_velocity(void);
