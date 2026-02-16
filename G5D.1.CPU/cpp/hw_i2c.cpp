#pragma O3
#pragma Otime

#include "hardware.h"

//#include <types.h>
//#include <core.h>
//#include <time.h>
//#include <CRC\CRC16_8005.h>
//#include <list.h>
//#include <PointerCRC.h>
//#include <SEGGER_RTT\SEGGER_RTT.h>
//#include "hw_rtm.h"


#ifdef CPU_SAME53

static S_I2C i2c(I2C_SERCOM_NUM, PIO_I2C, SCL, I2C_PMUX_SCL, PIO_I2C, SDA, I2C_PMUX_SDA, I2C_GEN_SRC, I2C_GEN_CLK, I2C_DMACH_NUM );

#elif defined(CPU_XMC48)

#endif 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool I2C_Update()
{
	return i2c.Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool I2C_AddRequest(DSCI2C *d)
{
	return i2c.AddRequest(d);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void I2C_Init()
{
	i2c.Connect(I2C_BAUDRATE);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <i2c_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
