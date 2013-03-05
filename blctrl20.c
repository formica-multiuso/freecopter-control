#include <stdlib.h>
#include <ch.h>
#include <hal.h>

#include "blctrl20.h"

uint8_t m1,m2,m3,m4 = {0,0,0,0};

extern int16_t icu_ch[8]; 
extern int16_t roll_controller_output, pitch_controller_output, yaw_controller_output;

static i2cflags_t a_errors[4] = {0,0,0,0};

static uint8_t blctrl20_lut[800] = {	//LOOKUP Table for blctrl20 ESC
#include "blctrl20_2.lut"
};



void blctrl20_init(void)
{
}

void blctrl20_set_velocity(void)
{
	msg_t status = RDY_OK;
        systime_t tmo = MS2ST(4);
	
	i2cAcquireBus(&I2CD1);
	

	m1 = ((icu_ch[0]-600)/4) + yaw_controller_output - pitch_controller_output;	// FRONT MOTOR
	status = i2cMasterTransmitTimeout(&I2CD1,MOTOR1_ADDR,&m1,1,NULL,0,tmo);
	if (status != RDY_OK){
                 a_errors[0] = i2cGetErrors(&I2CD1);}
	
	
	m2 = ((icu_ch[0]-600)/4) - yaw_controller_output - roll_controller_output;	//RIGHT MOTOR
	status = i2cMasterTransmitTimeout(&I2CD1,MOTOR2_ADDR,&m2,1,NULL,0,tmo);
	if (status != RDY_OK){
                 a_errors[1] = i2cGetErrors(&I2CD1);}


	m3 = ((icu_ch[0]-600)/4) + yaw_controller_output + pitch_controller_output;	//REAR MOTOR
	status = i2cMasterTransmitTimeout(&I2CD1,MOTOR3_ADDR,&m3,1,NULL,0,tmo);
	if (status != RDY_OK){
                 a_errors[2] = i2cGetErrors(&I2CD1);}
	

	m4 = ((icu_ch[0]-600)/4) - yaw_controller_output + roll_controller_output;	//LEFT MOTOR
	status = i2cMasterTransmitTimeout(&I2CD1,MOTOR4_ADDR,&m4,1,NULL,0,tmo);	
	if (status != RDY_OK){
                 a_errors[3] = i2cGetErrors(&I2CD1);}
	

	i2cReleaseBus(&I2CD1);

}



