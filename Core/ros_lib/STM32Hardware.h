#ifndef _STM32_HARDWARE_H_
#define _STM32_HARDWARE_H_

#include "main.h"
#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"


//extern void vcp_write(uint8_t* Buf, uint16_t Len);
//extern int vcp_read(void);

class STM32Hardware
{

	public:

	STM32Hardware(){

	}

	void init(){

	}

	// Read a byte of data from received bytes
	// If no new data is available, returns -1
	int read(){
		return vcp_read();
	}


	// Send a byte of data to ROS connection
	void write(uint8_t* data, int length){
		vcp_write(data,length);
	}

	// Returns milliseconds since start of program
	unsigned long time(void){
		return HAL_GetTick();
	}

};

#endif

