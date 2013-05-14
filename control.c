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
#include "ch.h"
#include "chvt.h"
#include "control.h"

extern float Roll, Pitch, Yaw;	// Estimation from IMU
extern int16_t icu_ch[8];			// 1. Throttle - 2. Pitch - 3. Roll - 4. Yaw  (RPY are the PID References Signals)
					// Throttle signal is not used in this file but directly summed in blctrl20.c

int8_t roll_controller_output, pitch_controller_output, yaw_controller_output;

float roll_prev_error, pitch_prev_error, yaw_prev_error;
float roll_I, pitch_I, yaw_I;
	
int32_t last_time = 0;




int8_t KP,KI,KD;				// PID Coefficient

void pid_controller(void)
{
	float roll_D, pitch_D, yaw_D, dt;	
	float roll_error, pitch_error, yaw_error;
	int32_t now;
	KP = 10;
	KI = 10;
	KD = 10;
	
//	roll_I_error = 0;
//	pitch_I_error = 0;
//	yaw_I_error = 0;
	
	roll_error = 0 - Roll;
	pitch_error = 0 - Pitch;
	yaw_error = 0 - Yaw;

	roll_I = roll_I + roll_error*0.01;
	pitch_I = pitch_I + pitch_error*0.01;

	roll_D = (roll_error - roll_prev_error)/0.01;
	pitch_D = (pitch_error - pitch_prev_error)/0.01;

	roll_controller_output = (int8_t)(KP*roll_error + KI*roll_I + KD*roll_D);
	pitch_controller_output = (int8_t)(KP*pitch_error + KI*pitch_I + KD*pitch_D);

	roll_controller_output = (int8_t)roll_error;
	pitch_controller_output = (int8_t)pitch_error;	


	roll_prev_error = roll_error;
	pitch_prev_error = pitch_error;
	


	/*
	//error computation
	roll_error = -(icu_ch[1] - 1000)/10 - (int16_t)(Roll/200);
	pitch_error = (icu_ch[2] - 1000)/10 - (int16_t)(Pitch/200);
	yaw_error =  -(icu_ch[3] - 1000)/10 - (int16_t)(Yaw/200);
	
	//Integral of error
	roll_I_error += roll_error * dt;
	pitch_I_error += pitch_error * dt;
	yaw_I_error += yaw_error * dt;
	
	//Derivative of error
	roll_D_error = (roll_error - roll_last_error)/dt;
	pitch_D_error = (pitch_error - pitch_last_error)/dt;
	yaw_D_error = (yaw_error - yaw_last_error)/dt;

	//PID EQUATION
	roll_controller_output = KP*roll_error + KI*roll_I_error + KD*roll_D_error;
	pitch_controller_output = KP*pitch_error + KI*pitch_I_error + KD*pitch_D_error;
	yaw_controller_output = KP*yaw_error + KI*yaw_I_error + KD*yaw_D_error;	
  */
}
	
