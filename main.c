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

#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "test.h"

#include "usb_cdc.h"
#include "shell.h"
#include "chprintf.h"
#include "blctrl20.h"
#include "i2c_setup.h"
#include "control.h"


#define MOTOR1_ADDR 0x29       // Motors i2c Addresses 
#define MOTOR2_ADDR 0x2a
#define MOTOR3_ADDR 0x2b
#define MOTOR4_ADDR 0x2c

//#define GRAUPNER_MX16_HOTT
#define GRAUPNER_MX16_35MHZ

/*===========================================================================*/
/* GLOBAL VARIABLES                                                          */
/*===========================================================================*/

extern uint8_t m1,m2,m3,m4;

int8_t roll_controller_output, pitch_controller_output, yaw_controller_output;

float roll_D, pitch_D, yaw_D;
float roll_error, pitch_error, yaw_error;
float roll_prev_error, pitch_prev_error, yaw_prev_error;
float roll_I, pitch_I, yaw_I;

int8_t KP,KI,KD;                                // PID Coefficient

float Roll, Pitch, Yaw;

static Mutex mtx_imu, mutex_motors;

int16_t rxbuf_16bit[24];
int16_t txbuf_16bit[24];

icucnt_t last_width = 50;
icucnt_t last_period = 50;
int frame_flag;
int16_t icu_ch[8] = {0,0,0,0,0,0,0,0};



/*===========================================================================*/
/* USB related stuff.                                                        */
/*===========================================================================*/

/*
 * USB Driver structure.
 */
static SerialUSBDriver SDU1;

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0x02,          /* bDeviceClass (CDC).              */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         0x0483,        /* idVendor (ST).                   */
                         0x5740,        /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[67] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
                         0x02,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0xC0,          /* bmAttributes (self powered).     */
                         50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x01,          /* bNumEndpoints.                   */
                         0x02,          /* bInterfaceClass (Communications
                                           Interface Class, CDC section
                                           4.2).                            */
                         0x02,          /* bInterfaceSubClass (Abstract
                                         Control Model, CDC section 4.3).   */
                         0x01,          /* bInterfaceProtocol (AT commands,
                                           CDC section 4.4).                */
                         0),            /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE         (5),            /* bLength.                         */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                           Functional Descriptor.           */
  USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                           Control Management Descriptor).  */
  USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                           Class Interface).                */
  USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                           Interface).                      */
  /* Endpoint 2 Descriptor.*/
  USB_DESC_ENDPOINT     (USB_CDC_INTERRUPT_REQUEST_EP|0x80,
                         0x03,          /* bmAttributes (Interrupt).        */
                         0x0008,        /* wMaxPacketSize.                  */
                         0xFF),         /* bInterval.                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x0A,          /* bInterfaceClass (Data Class
                                           Interface, CDC section 4.5).     */
                         0x00,          /* bInterfaceSubClass (CDC section
                                           4.6).                            */
                         0x00,          /* bInterfaceProtocol (CDC section
                                           4.7).                            */
                         0x00),         /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (USB_CDC_DATA_AVAILABLE_EP,     /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USB_CDC_DATA_REQUEST_EP|0x80,  /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00)          /* bInterval.                       */
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] = {
  USB_DESC_BYTE(38),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
  'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
  'c', 0, 's', 0
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(56),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, '/', 0,
  'R', 0, 'T', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,
  'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0,
  'o', 0, 'r', 0, 't', 0
};

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[] = {
  USB_DESC_BYTE(8),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  '0' + CH_KERNEL_MAJOR, 0,
  '0' + CH_KERNEL_MINOR, 0,
  '0' + CH_KERNEL_PATCH, 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {

  (void)usbp;
  (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &vcom_strings[dindex];
  }
  return NULL;
}

/**
 * @brief   EP1 initialization structure (IN only).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_BULK | USB_EP_MODE_PACKET,
  NULL,
  sduDataTransmitted,
  NULL,
  0x0040,
  0x0000,
  NULL,
  NULL
};

/**
 * @brief   EP2 initialization structure (IN only).
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_INTR | USB_EP_MODE_PACKET,
  NULL,
  sduInterruptTransmitted,
  NULL,
  0x0010,
  0x0000,
  NULL,
  NULL
};

/**
 * @brief   EP3 initialization structure (OUT only).
 */
static const USBEndpointConfig ep3config = {
  USB_EP_MODE_TYPE_BULK | USB_EP_MODE_PACKET,
  NULL,
  NULL,
  sduDataReceived,
  0x0000,
  0x0040,
  NULL,
  NULL
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {

  switch (event) {
  case USB_EVENT_RESET:
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    chSysLockFromIsr();
    usbInitEndpointI(usbp, USB_CDC_DATA_REQUEST_EP, &ep1config);
    usbInitEndpointI(usbp, USB_CDC_INTERRUPT_REQUEST_EP, &ep2config);
    usbInitEndpointI(usbp, USB_CDC_DATA_AVAILABLE_EP, &ep3config);
    chSysUnlockFromIsr();
    return;
  case USB_EVENT_SUSPEND:
    return;
  case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

/*
 * Serial over USB driver configuration.
 */
static const SerialUSBConfig serusbcfg = {
  &USBD1,
  {
    usb_event,
    get_descriptor,
    sduRequestsHook,
    NULL
  }
};

static void end_spi_tx(SPIDriver *spip)
{
	palClearPad(IOPORT3, 11);	
}	


uint16_t spi_frame;

static const SPIConfig hs_spicfg = {
  NULL,
  GPIOB,
  GPIOB_SPI2NSS,
  0
};

static const SPIConfig ls_spicfg = {
  NULL,
  GPIOB,
  GPIOB_SPI2NSS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0
};


/*===========================================================================*/
/* ICU STUFF								     */
/*===========================================================================*/

#ifdef GRAUPNER_MX16_HOTT

static void icuwidthcb(ICUDriver *icup) {
  palSetPad(IOPORT3,10);
  last_width = icuGetWidthI(icup);
  if(last_width > 5000) frame_flag = 1;
  else if((frame_flag >= 1) && (frame_flag <= 9))
  {
	icu_ch[frame_flag - 1] = last_width;
	frame_flag += 1;
  }
 
  if(frame_flag == 9) frame_flag=0;  
}

static void icuperiodcb(ICUDriver *icup) {
  
  palSetPad(IOPORT3,10);
  last_period = icuGetPeriodI(icup);
  if(last_period > 5000) frame_flag = 1;
  if((frame_flag > 1) && (frame_flag <= 9))
  {
	icu_ch[frame_flag++ - 1] = (char) last_period;
		
  }
 
  if(frame_flag == 9) frame_flag=0; 

}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_HIGH,
  1000000,                                    /* 1MHz ICU clock frequency.   */
  icuwidthcb,
  NULL
  //icuperiodcb
};

int icutemp[8];

#endif

#ifdef GRAUPNER_MX16_35MHZ

int radio_count = 0;

static void icuwidthcb(ICUDriver *icup) {
  palSetPad(IOPORT3,10);
  last_width = icuGetWidthI(icup);
  icu_ch[radio_count] = last_width;
  radio_count += 1;
  if(radio_count == 4) radio_count=0;  
}

static void icuperiodcb(ICUDriver *icup) {
  
  palSetPad(IOPORT3,10);
  last_period = icuGetPeriodI(icup);
  if(last_period > 5000) frame_flag = 1;
  if((frame_flag > 1) && (frame_flag <= 9))
  {
	icu_ch[frame_flag++ - 1] = (char) last_period;
		
  }
 
  if(frame_flag == 9) frame_flag=0; 

}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_LOW,
  1000000,                                    /* 1MHz ICU clock frequency.   */
  icuwidthcb,
  NULL
  //icuperiodcb
};

int icutemp[8];

#endif









/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

/*
 * Red LED blinker thread, times are in milliseconds.
 */

static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
    palClearPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(500);
    palSetPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(500);
  }
  return 0;
}

static WORKING_AREA(waMotorsThread, 256);
static msg_t MotorsThread(void *arg) {
  chRegSetThreadName("Motors");
  (void)arg;
  
  blctrl20_init();
 
  while (TRUE) {
	
	chMtxLock(&mutex_motors);
        //pid_controller();
	blctrl20_set_velocity();
	chThdSleepMilliseconds(50);
	chMtxUnlock();
  }
  return 0;
}




static WORKING_AREA(waRCThread, 256);
static msg_t RCThread(void *arg) {
  char frame_flag = 0;
  int i,k;
  chRegSetThreadName("RadioController");
  (void)arg;

  i=0;
  k=0;

  icuStart(&ICUD1, &icucfg);
  icuEnable(&ICUD1);
  while (TRUE) 
  {
	if(frame_flag = 0)
		if(last_width > 30000)
		{
//			if((k++ % 1000) == 0) palTogglePad(IOPORT3,11);
			frame_flag = 1;
			break;
		}
       	if(frame_flag = 1)
		for(i=0;i<=8;i++)
		{
			icutemp[i] = last_width;
		}
		frame_flag = 0;

	chThdSleepMilliseconds(5);
   }
	

  return 0;
}


/*
 * Application entry point.
 */
int main(void) {

   /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */

  uint32_t tempR, tempP, tempY;
  float Roll,Pitch,Yaw;  


  halInit();
  chSysInit();
  i2c_setup();

  palSetPadMode(IOPORT1, 8, PAL_MODE_INPUT);      // PA8 - RADIO INPUT
 
  palSetPadMode(IOPORT2, 13, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);     /* SCK. */
  palSetPadMode(IOPORT2, 14, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* MISO.*/
  palSetPadMode(IOPORT2, 15, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);     /* MOSI.*/
  palSetPadMode(IOPORT2, 12, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);

  palSetPadMode(IOPORT3, 11, PAL_MODE_OUTPUT_PUSHPULL);
  
  chMtxInit(&mtx_imu);
  chMtxInit(&mutex_motors);

  palClearPad(IOPORT3,10);
  //palClearPad(IOPORT3,11);
  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  usbConnectBus(serusbcfg.usbp);
  //palClearPad(GPIOC, GPIOC_USB_DISC);
  
  icuStart(&ICUD1, &icucfg);
  icuEnable(&ICUD1);
  
  chThdSleepSeconds(1);
  
  chThdCreateStatic(waThread1, sizeof(waThread1), HIGHPRIO, Thread1, NULL);

  //chThdCreateStatic(waMotorsThread, sizeof(waMotorsThread), NORMALPRIO, MotorsThread, NULL);

 
  spiAcquireBus(&SPID2);                        /* Acquire ownership of the bus.    */
  spiStart(&SPID2, &ls_spicfg);                 /* Setup transfer parameters.       */
  spiSelect(&SPID2);


  KP = 1;
  KI = 1;
  KD = 0;

  while (TRUE) {

	while(!(SPID2.spi->SR & SPI_SR_RXNE)); 
	 	
  		spiReceive(&SPID2,24,rxbuf_16bit);

//	Floating point calculation of Roll/Pitch/Yaw inside the microcontroller. Decomment next 6 lines if you want to implement
//	the control Law.

	tempR = (uint32_t)((rxbuf_16bit[0] << 31) | (rxbuf_16bit[1] << 23) | (rxbuf_16bit[2] << 11) | rxbuf_16bit[3]);
	tempP = (uint32_t)((rxbuf_16bit[4] << 31) | (rxbuf_16bit[5] << 23) | (rxbuf_16bit[6] << 11) | rxbuf_16bit[7]);
	tempY = (uint32_t)((rxbuf_16bit[8] << 31) | (rxbuf_16bit[9] << 23) | (rxbuf_16bit[10] << 11) | rxbuf_16bit[11]);
		
	Roll = (*(float*)&tempR);
	Pitch = (*(float*)&tempP); 					
	Yaw = (*(float*)&tempY);
	
	// CONTROL LAW HERE
	//++ (ROLL,PITCH,YAW ERRORS) -----> [CONTROLLER] -----> (MOTORS INPUT)
	

	// PID Control Law
	
	roll_error = 0 - Roll;
        pitch_error = 0 - Pitch;
        yaw_error = 0 - Yaw;

        roll_I = roll_I + roll_error*0.01;
        pitch_I = pitch_I + pitch_error*0.01;

        roll_D = (roll_error - roll_prev_error)/0.01;
        pitch_D = (pitch_error - pitch_prev_error)/0.01;

        roll_controller_output = (int8_t)(KP*roll_error + KI*roll_I + KD*roll_D);
        pitch_controller_output = (int8_t)(KP*pitch_error + KI*pitch_I + KD*pitch_D);

        //roll_controller_output = (int8_t)roll_error;
        //pitch_controller_output = (int8_t)pitch_error;


        roll_prev_error = roll_error;
        pitch_prev_error = pitch_error;


	//-------------------------------------------------------------------

	blctrl20_set_velocity();


	palSetPad(IOPORT3,11);


/*	SPI FLOATING POINT TEST PACKET (sending 5.6F)	
	rxbuf_16bit[0] = 0;
	rxbuf_16bit[1] = 129;
	rxbuf_16bit[2] = 1638;
	rxbuf_16bit[3] = 819;
	rxbuf_16bit[4] = 0;
	rxbuf_16bit[5] = 129;
	rxbuf_16bit[6] = 1638;
	rxbuf_16bit[7] = 819;
	rxbuf_16bit[8] = 1;
	rxbuf_16bit[9] = 129;
	rxbuf_16bit[10] = 1638;
	rxbuf_16bit[11] = 819;
*/	



	if(SDU1.config->usbp->state==USB_ACTIVE)
	{
//		chprintf((BaseChannel *)&SDU1,"S:%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d:E\r\n",rxbuf_16bit[0],rxbuf_16bit[1],rxbuf_16bit[2],rxbuf_16bit[3],rxbuf_16bit[4],rxbuf_16bit[5],rxbuf_16bit[6],rxbuf_16bit[7],rxbuf_16bit[8],rxbuf_16bit[9],rxbuf_16bit[10],rxbuf_16bit[11]);
		chprintf((BaseChannel *)&SDU1, "S:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:%6D:E\r\n",rxbuf_16bit[0],rxbuf_16bit[1],rxbuf_16bit[2],rxbuf_16bit[3],rxbuf_16bit[4],rxbuf_16bit[5],rxbuf_16bit[6],rxbuf_16bit[7],rxbuf_16bit[8],rxbuf_16bit[9],rxbuf_16bit[10],rxbuf_16bit[11],(int8_t)Roll,(int8_t)Pitch,(int8_t)Yaw,icu_ch[3],icu_ch[4],roll_controller_output,pitch_controller_output,yaw_controller_output);

	}

    	chThdSleepMilliseconds(10);
  }
  spiUnselect(&SPID2);
  spiReleaseBus(&SPID2);                        /* Ownership release.               */


}
