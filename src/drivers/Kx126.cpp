#include "Kx126.h"
#include <hal/nrf_gpio.h>
#include <libraries/delay/nrf_delay.h>
#include <libraries/log/nrf_log.h>
#include "Spi.h"
#include <math.h>
#include <algorithm>
#include "SEGGER_RTT.h" 
using namespace Watch::Drivers;

Kx126::Kx126(Spi& spi) : spi{spi} {
}

 void Kx126::Init() {
  spi.WriteCmdAndBuffer(&KX126_CNTL1, 1,&KX126_CNTL1_CFGDAT, 1);
  //spi.WriteCmdAndBuffer(&KX126_TILT_TIMER, 1,&KX126_TILT_TIMER_CFGDAT, 1);	
  //spi.WriteCmdAndBuffer(&KX126_ODCNTL,1, &KX126_ODCNTL_VAL, 1);
  //spi.WriteCmdAndBuffer(&KX126_CNTL3,1, & KX126_CNTL3_CFGDAT, 1);
  //spi.WriteCmdAndBuffer(&KX126_CNTL4,1, & KX126_CNTL4_CFGDAT, 1);
	//spi.WriteCmdAndBuffer(&KX126_INC2,1,&KX126_INC2_CFGDAT, 1);
	//spi.WriteCmdAndBuffer(&KX126_WUFC,1, &KX126_WUFC_CFGDAT, 1);
  //spi.WriteCmdAndBuffer(&KX126_FFTH,1, &KX126_FFTH_CFGDAT, 1); 
  //spi.WriteCmdAndBuffer(&KX126_BUF_CNTL1,1, &KX126_BUF_CNTL1_BUFE , 1);  
  //spi.WriteCmdAndBuffer(&KX126_BUF_CNTL2,1, &KX126_BUF_CNTL2_BUFE , 1);  
  // Set accelerometer to operating mode (PC1=1)
	RawSensor._uchar = (unsigned char)(KX126_CNTL1_CFGDAT|0x80);
	spi.WriteCmdAndBuffer(&KX126_CNTL1,1, &RawSensor._uchar, 1);	  
}


Kx126::Values Kx126::Process() {
   uint8_t data[6];
    spi.Read(&KX126_XOUT_L, 1, data, 6);
    x=data[1]<<8|data[0];
    y=data[3]<<8|data[2];
    z=data[5]<<8|data[4];
   return {0, x, y, z};                                                                                                                                                                                                                                                                                                                                                                     
}


 void Kx126::SoftReset(){
    spi.WriteCmdAndBuffer(&KX126_CNTL1, 1,&KX126_CNTL1_CFGDAT, 1);
    spi.WriteCmdAndBuffer(&KX126_CNTL2, 1,&KX126_CNTL2_CFGDAT, 1);
    nrf_delay_ms(100);
 }
