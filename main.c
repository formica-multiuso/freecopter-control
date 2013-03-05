/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

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


/*===========================================================================*/
/* GLOBAL VARIABLES                                                          */
/*===========================================================================*/

extern uint8_t m1,m2,m3,m4;

int16_t Roll, Pitch, Yaw;

static Mutex mtx_imu, mutex_motors;

static uint8_t txbuf[5] = {0,0,0,0,0}; 		// SPI TX Buffer
static uint8_t rxbuf[6] = {0,0,0,0,0,0};	// SPI RX Buffer
int16_t rxbuf_16bit[5];
int16_t txbuf_16bit[5];

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
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)
#define TEST_WA_SIZE    THD_WA_SIZE(256)

static void cmd_mem(BaseChannel *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseChannel *chp, int argc, char *argv[]) {
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state time\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n",
            (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
            (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
            states[tp->p_state], (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_test(BaseChannel *chp, int argc, char *argv[]) {
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: test\r\n");
    return;
  }
  tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriority(),
                           TestThread, chp);
  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }
  chThdWait(tp);
}

// Dynamic Thread allocation for USB
//------------------------------------
static WORKING_AREA(UsbThreadWA,128);

static msg_t UsbThread(void *arg)
{
	BaseChannel *chp;
	chp = (BaseChannel *)arg;
	while(TRUE)
	{	
		//chMtxLock(&mtx_lsm); chMtxLock(&mtx_l3g); chMtxLock(&mtx_bmp);
//		chprintf(chp,"%Ld:%Ld:%Ld:%d:%d:%d:%d:%d:%d:%d:%d\r\n",accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magnX,magnY,magnZ,temperature,pressure);
		//chMtxUnlock();//chMtxUnlock();chMtxUnlock();
//		chMtxLock(&mutex_motors);
		chprintf(chp,"CHANNELS:%d:%d:%d:%d:%d:%d:%d:%d:IMU:%d:%d:%d:%d:%d:%d:IMUCH:%d:%d:%d:%d:%d:RPY:%d:%d:%d\r\n",icu_ch[0],icu_ch[1],icu_ch[2],icu_ch[3],icu_ch[4],icu_ch[5],icu_ch[6],icu_ch[7],rxbuf[0],rxbuf[1],rxbuf[2],rxbuf[3],rxbuf[4],rxbuf[5],rxbuf_16bit[0],rxbuf_16bit[1],rxbuf_16bit[2],rxbuf_16bit[3],rxbuf_16bit[4],Roll,Pitch,Yaw);
//		chMtxUnlock();
		chThdSleepMilliseconds(25);		
	}
	return 0;
}

static void cmd_read_data(BaseChannel *chp, int argc, char *argv[]) {
  Thread *tp;
  tp = chThdCreateFromHeap(NULL, sizeof(UsbThreadWA), NORMALPRIO, UsbThread, chp);
  chThdWait(tp);
}

//--------------------------------------

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"test", cmd_test},
  {"read-data", cmd_read_data},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseChannel *)&SDU1,
  commands
};

/*===========================================================================*/
/* ICU STUFF								     */
/*===========================================================================*/


int zero = 0;

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
        pid_controller();
	blctrl20_set_velocity();
	chThdSleepMilliseconds(50);
	chMtxUnlock();
  }
  return 0;
}


static WORKING_AREA(waIMUThread, 256);
static msg_t IMUThread(void *arg) {
  chRegSetThreadName("IMU");
  (void)arg;

  while (TRUE) {
	  
	  spiAcquireBus(&SPID2);                        /* Acquire ownership of the bus.    */
	  spiStart(&SPID2, &ls_spicfg);                 /* Setup transfer parameters.       */
	  chMtxLock(&mtx_imu);
	  //spiReceive(&SPID2,5,rxbuf_16bit);       	/* Atomic transfer operations.      */
	  //spi_lld_polled_exchange(&SPID2,spi_frame);
	  //chMtxUnlock();
	  spiExchange(&SPID2,10,txbuf_16bit,rxbuf_16bit);


	  if(rxbuf_16bit[0] != 0) palClearPad(IOPORT3,11);
	  else palSetPad(IOPORT3,11);
	  spiReleaseBus(&SPID2);                        /* Ownership release.               */
	  palSetPad(IOPORT3,11);
	
	 	Roll = (int16_t) rxbuf_16bit[0];
	  	Pitch = (int16_t) rxbuf_16bit[3];
//	  	Yaw = (int16_t) rxbuf_16bit[3];
	  //else chThdSleepMilliseconds(1);

	  chMtxUnlock();
	  chThdSleepMilliseconds(10);
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
  Thread *shelltp = NULL;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
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
  

  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Creates the blinker thread.
   */
  
  chThdSleepSeconds(1);
  
  chThdCreateStatic(waThread1, sizeof(waThread1), HIGHPRIO, Thread1, NULL);

  chThdCreateStatic(waMotorsThread, sizeof(waMotorsThread), NORMALPRIO, MotorsThread, NULL);

//  chThdCreateStatic(waRCThread, sizeof(waRCThread), NORMALPRIO, RCThread, NULL);

  chThdCreateStatic(waIMUThread, sizeof(waIMUThread), NORMALPRIO, IMUThread, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
   // CONTROL LAW HERE!!!
   
  while (TRUE) {
    if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
      shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
    else if (chThdTerminated(shelltp)) {
      chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
      shelltp = NULL;           /* Triggers spawning of a new shell.        */
    }
    chThdSleepMilliseconds(1000);
  }
}
