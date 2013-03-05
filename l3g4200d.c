#include <stdlib.h>
#include <ch.h>
#include <hal.h>

#include "l3g4200d.h"

int16_t gyroX, gyroY, gyroZ;

static i2cflags_t errors[5] = {0,0,0,0,0};
static i2cflags_t g_errors[6] = {0,0,0,0,0,0};

int l3g4200d_init(void)
{

	msg_t status = RDY_OK;
        uint8_t buffer_tx[2];
        systime_t tmo = MS2ST(4);

        i2cAcquireBus(&I2CD1);
	buffer_tx[0] = CTRL_REG2;
        buffer_tx[1] = 0x24;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[0] = i2cGetErrors(&I2CD1);}
        buffer_tx[0] = CTRL_REG3;
        buffer_tx[1] = 0x00;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[1] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = CTRL_REG4;
        buffer_tx[1] = 0x30;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[2] = i2cGetErrors(&I2CD1);}
        buffer_tx[0] = CTRL_REG5;
        buffer_tx[1] = 0x00;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[3] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = CTRL_REG1;
        buffer_tx[1] = 0x7F;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[4] = i2cGetErrors(&I2CD1);}
	i2cReleaseBus(&I2CD1);

	return 0;
}

void l3g4200d_read_gyro(void)
{
        msg_t status = RDY_OK;
        uint8_t buffer_tx;
	uint8_t buffer_rx_X[2];
	uint8_t buffer_rx_Y[2];
	uint8_t buffer_rx_Z[2];
        systime_t tmo = MS2ST(4);

        i2cAcquireBus(&I2CD1);
	buffer_tx = OUT_X_L;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,buffer_rx_X,2,tmo);
        if (status != RDY_OK){
                 g_errors[0] = i2cGetErrors(&I2CD1);}
       // status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,OUT_X_L,1,&buffer_rx[1],1,tmo);
       // if (status != RDY_OK){
       //          g_errors[1] = i2cGetErrors(&I2CD1);}
        buffer_tx = OUT_Y_L;
	status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,buffer_rx_Y,2,tmo);
        if (status != RDY_OK){
                 g_errors[2] = i2cGetErrors(&I2CD1);}
        //status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,OUT_Y_L,1,&buffer_rx[3],1,tmo);
        //if (status != RDY_OK){
        //         g_errors[3] = i2cGetErrors(&I2CD1);}
        buffer_tx = OUT_Z_L;
	status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,buffer_rx_Z,2,tmo);
        if (status != RDY_OK){
                 g_errors[4] = i2cGetErrors(&I2CD1);}
       // status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,OUT_Z_L,1,&buffer_rx[5],1,tmo);
       // if (status != RDY_OK){
       //          g_errors[5] = i2cGetErrors(&I2CD1);}
        i2cReleaseBus(&I2CD1);

	gyroX = (buffer_rx_X[0] << 8) + buffer_rx_X[1];
        gyroY = (buffer_rx_Y[0] << 8) + buffer_rx_Y[1];
        gyroZ = (buffer_rx_Z[0] << 8) + buffer_rx_Z[1];
}
