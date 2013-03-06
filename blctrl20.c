/*
    FreeCopter Control firmware - Copyright (C) 2012, +inf
			      Roberto Marino

    FreeCopter Control is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    FreeCopter IMU is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
	    
    FreeCopter is based on

    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

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



