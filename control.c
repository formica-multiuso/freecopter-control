#include <stdlib.h>
#include "ch.h"
#include "chvt.h"
#include "control.h"

extern int16_t Roll, Pitch, Yaw;	// Estimation from IMU
extern int16_t icu_ch[8];			// 1. Throttle - 2. Pitch - 3. Roll - 4. Yaw  (RPY are the PID References Signals)
					// Throttle signal is not used in this file but directly summed in blctrl20.c

int8_t roll_controller_output, pitch_controller_output, yaw_controller_output;

int8_t KP,KI,KD;				// PID Coefficient

void pid_controller(void)
{
	static int16_t roll_last_error, pitch_last_error, yaw_last_error;
	int16_t roll_error, pitch_error, yaw_error;
	double roll_I_error, pitch_I_error, yaw_I_error;
	double roll_D_error, pitch_D_error, yaw_D_error, dt;	
	int32_t now;
	static int32_t last_time = 0;

	KP = 0.5;
	KI = 1;
	KD = 0;
	
	roll_I_error = 0;
	pitch_I_error = 0;
	yaw_I_error = 0;
	


	now = chTimeNow();	
	dt = (double)(now - last_time);
	
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
  
}
	
