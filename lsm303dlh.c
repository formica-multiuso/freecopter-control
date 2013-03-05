#include <stdlib.h>
#include <ch.h>
#include <hal.h>

#include "lsm303dlh.h"

int16_t accelX, accelY, accelZ;
int16_t magnX, magnY, magnZ;

static i2cflags_t errors[12] = {0,0,0,0};
static i2cflags_t a_errors[6] = {0,0,0,0,0,0};
static i2cflags_t m_errors[6] = {0,0,0,0,0,0};

int lsm303dlh_init(void){

	msg_t status = RDY_OK;
        uint8_t buffer_tx[2];
        systime_t tmo = MS2ST(4);

		
	i2cAcquireBus(&I2CD1);
	// Configuring Accelerometer
	buffer_tx[0] = CTRL_REG1_A;
	buffer_tx[1] = STATUS_REG_A;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,buffer_tx,2,NULL,0,tmo);
	if (status != RDY_OK){
                 errors[0] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = CTRL_REG4_A;
	buffer_tx[1] = 0x40;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,buffer_tx,2,NULL,0,tmo);
	if (status != RDY_OK){
                 errors[1] = i2cGetErrors(&I2CD1);}
	// Configuring Magnetometer
	buffer_tx[0] = CRA_REG_M;
	buffer_tx[1] = 0x14;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,buffer_tx,2,NULL,0,tmo);
	if (status != RDY_OK){
                 errors[2] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = MR_REG_M;
	buffer_tx[1] = 0x00;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,buffer_tx,2,NULL,0,tmo);
	if (status != RDY_OK){
                 errors[3] = i2cGetErrors(&I2CD1);}
	i2cReleaseBus(&I2CD1);

	return 0;
}

void lsm303dlh_read_acceleration(void)
{
	msg_t status = RDY_OK;
        uint8_t buffer_rx_X[2];
	uint8_t buffer_rx_Y[2];
	uint8_t buffer_rx_Z[2];
	uint8_t buffer_tx;
        systime_t tmo = MS2ST(4);

	i2cAcquireBus(&I2CD1);
	buffer_tx = OUT_X_L_A;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,&buffer_tx,1,buffer_rx_X,2,tmo);
	if (status != RDY_OK){
                 a_errors[0] = i2cGetErrors(&I2CD1);}
	//status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,OUT_X_L_A,1,&buffer_rx[1],1,tmo);
//	if (status != RDY_OK){
//                 a_errors[1] = i2cGetErrors(&I2CD1);}
	buffer_tx = OUT_Y_L_A;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,&buffer_tx,1,buffer_rx_Y,2,tmo);
	if (status != RDY_OK){
                 a_errors[2] = i2cGetErrors(&I2CD1);}
//	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,OUT_Y_L_A,1,&buffer_rx[3],1,tmo);
//	if (status != RDY_OK){
//                 a_errors[3] = i2cGetErrors(&I2CD1);}
	buffer_tx = OUT_Z_L_A;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,&buffer_tx,1,buffer_rx_Z,2,tmo);
	if (status != RDY_OK){
                 a_errors[4] = i2cGetErrors(&I2CD1);}
//	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,OUT_Z_L_A,1,&buffer_rx[5],1,tmo);
//	if (status != RDY_OK){
//                 a_errors[5] = i2cGetErrors(&I2CD1);}
	i2cReleaseBus(&I2CD1);

	accelX = (int16_t) ((buffer_rx_X[1] << 8) | buffer_rx_X[0]);
	accelY = (int16_t) ((buffer_rx_Y[1] << 8) | buffer_rx_Y[0]);
	accelZ = (int16_t) ((buffer_rx_Z[1] << 8) | buffer_rx_Z[0]);
}


void lsm303dlh_read_magfield(void)
{
	msg_t status = RDY_OK;
        uint8_t buffer_rx[6];
        systime_t tmo = MS2ST(4);

	i2cAcquireBus(&I2CD1);
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,OUT_X_L_M,1,&buffer_rx[0],1,tmo);
	if (status != RDY_OK){
                 m_errors[0] = i2cGetErrors(&I2CD1);}
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,OUT_X_H_M,1,&buffer_rx[1],1,tmo);
	if (status != RDY_OK){
                 m_errors[1] = i2cGetErrors(&I2CD1);}
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,OUT_Y_L_M,1,&buffer_rx[2],1,tmo);
	if (status != RDY_OK){
                 m_errors[2] = i2cGetErrors(&I2CD1);}
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,OUT_Y_H_M,1,&buffer_rx[3],1,tmo);
	if (status != RDY_OK){
                 m_errors[3] = i2cGetErrors(&I2CD1);}
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,OUT_Z_L_M,1,&buffer_rx[4],1,tmo);
	if (status != RDY_OK){
                 m_errors[4] = i2cGetErrors(&I2CD1);}
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,OUT_Z_H_M,1,&buffer_rx[5],1,tmo);
	if (status != RDY_OK){
                 m_errors[5] = i2cGetErrors(&I2CD1);}
	i2cReleaseBus(&I2CD1);

	magnX = (buffer_rx[1] << 8) + buffer_rx[0];
	magnY = (buffer_rx[3] << 8) + buffer_rx[2];
	magnZ = (buffer_rx[5] << 8) + buffer_rx[4];
}

