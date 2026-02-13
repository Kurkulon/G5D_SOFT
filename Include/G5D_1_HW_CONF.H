#ifndef G26X_3_HW_CONF_H__11_01_2024__11_13
#define G26X_3_HW_CONF_H__11_01_2024__11_13

#pragma once

#include <types.h>
#include <core.h>

#define CLKIN_MHz		16
#define CLKIN_XOSC_NUM	0
#define MCK_MHz			200
#define MCK				(MCK_MHz*1000000)
#define NS2CLK(x)		(((x)*MCK_MHz+500)/1000)
#define US2CLK(x)		((x)*MCK_MHz)
#define MS2CLK(x)		((x)*MCK_MHz*1000)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#define FRAM_SPI_MAINVARS_ADR 0

#define FRAM_I2C_MAINVARS_ADR	0
#define FRAM_I2C_GENTIME_ADR	0x200

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define REAL_TIME_CLOCK_DISABLE
#define SYSTEM_TICK_TIMER_ENABLE

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_SAME53	

	// Test Pins
	// 2	- PA01 	- ManRcvIRQ	
	// 3	- PA02 	- ManRcvIRQ sync true
	// 4	- PA03
	// 5	- PB04
	// 6	- PB05	
	// 10	- PB07	
	// 11	- PB08	
	// 12	- PB09	
	// 13	- PA04
	// 14	- PA05	
	// 15	- PA06
	// 16	- PA07 	- ManTrmIRQ2
	// 19	- PA10	
	// 20	- PA11	
	// 23	- PB10
	// 24	- PB11
	// 25	- PB12
	// 26	- PB13
	// 27	- PB14
	// 28	- PB15
	// 32	- PA15	- main loop	
	// 36	- PA17	- CLKIN/16
	// 37	- PA18	
	// 38	- PA19	
	// 49	- 
	// 50	- PB23	
	// 51	- PA27
	// 59	- PB30
	// 60	- PB31
	// 61	- PB00
	// 62	- PB01
	// 63	- PB02
	// 64	- PB03

	//	
	//
	//	


	// ++++++++++++++	GEN	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define GEN_MCK				0
	#define GEN_32K				1
	#define GEN_16M				2
	#define GEN_1M				3
	#define GEN_DAC				4
	//#define GEN_500K			4
	//#define GEN_EXT32K		5

	#define GEN_DAC_DIV			((u32)((MCK_MHz+6)/12))

	#define GEN_MCK_CLK			MCK
	#define GEN_32K_CLK			32768
	#define GEN_25M_CLK			25000000
	#define GEN_1M_CLK			1000000
	#define GEN_EXT32K_CLK		32768
	#define GEN_DAC_CLK			(MCK/GEN_DAC_DIV)

	// ++++++++++++++	SERCOM	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//#define SERCOM_0			0
	//#define SERCOM_1			1
	#define UART0_SERCOM_NUM	2
	#define I2C_SERCOM_NUM		3
	//#define SERCOM_4			4
	#define SPI_SERCOM_NUM		5
	//#define SERCOM_6			6
	//#define SERCOM_7			7


	// ++++++++++++++	DMA	0...31	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define	B_DMACH_NUM				0
	#define	M_DMACH_NUM				1
	//#define	PWMLB_DMA			2
	//#define	PWMHB_DMA			3
//	#define	DACTC_DMACH_NUM			4
	#define	UART0_DMACH_NUM			5
	//#define	NAND_MEMCOPY_DMA	6
	#define	I2C_DMACH_NUM			7
	#define	SPI_DMA_TX_CH			8
	#define	SPI_DMA_RX_CH			9
	//#define	ADC_DMA					30
	//#define	CRC_DMA					31

	// ++++++++++++++	EVENT 0...31	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define EVENT_DNNKB			0
	#define EVENT_DNNKM			1
	#define EVENT_WIN			2
	#define EVENT_MANR_1		3
	#define EVENT_MANR_2		4
	#define EVENT_BTCC_RETRG	5
	#define EVENT_MTCC_RETRG	6

	// ++++++++++++++	TC	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define	WinTC				TC0
	//#define	DAC_TC				TC1
	//#define MANT_TC			TC2
	#define MANI_TC				TC3
	//#define 					TC4
	//#define 					TC5

	#define GEN_TC0_TC1			GEN_MCK
	#define GEN_TC2_TC3			GEN_MCK
	#define GEN_TC4_TC5			GEN_MCK

	#define CLK_TC0_TC1			GEN_MCK_CLK
	#define CLK_TC2_TC3			GEN_MCK_CLK
	#define CLK_TC4_TC5			GEN_MCK_CLK

	#define GEN_TC0				GEN_TC0_TC1
	#define GEN_TC1				GEN_TC0_TC1
	#define GEN_TC2				GEN_TC2_TC3
	#define GEN_TC3				GEN_TC2_TC3
	#define GEN_TC4				GEN_TC4_TC5
	#define GEN_TC5				GEN_TC4_TC5

	#define GCLK_TC0			GCLK_TC0_TC1
	#define GCLK_TC1			GCLK_TC0_TC1
	#define GCLK_TC2			GCLK_TC2_TC3
	#define GCLK_TC3			GCLK_TC2_TC3
	#define GCLK_TC4			GCLK_TC4_TC5
	#define GCLK_TC5			GCLK_TC4_TC5

	#define CLK_TC0				CLK_TC0_TC1
	#define CLK_TC1				CLK_TC0_TC1
	#define CLK_TC2				CLK_TC2_TC3
	#define CLK_TC3				CLK_TC2_TC3
	#define CLK_TC4				CLK_TC4_TC5
	#define CLK_TC5				CLK_TC4_TC5

	// ++++++++++++++	TCC	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	#define MANT_TCC			TCC0
	#define GEN_TCC				TCC1
	#define B_TCC				TCC2
	#define M_TCC				TCC3
	#define MANR_TCC			TCC4

	#define GEN_TCC0_TCC1		GEN_MCK
	#define GEN_TCC2_TCC3		GEN_MCK
	#define GEN_TCC4			GEN_MCK

	#define CLK_TCC0_TCC1		GEN_MCK_CLK
	#define CLK_TCC2_TCC3		GEN_MCK_CLK
	#define CLK_TCC4			GEN_MCK_CLK

	#define GEN_TCC0			GEN_TCC0_TCC1
	#define GEN_TCC1			GEN_TCC0_TCC1
	#define GEN_TCC2			GEN_TCC2_TCC3
	#define GEN_TCC3			GEN_TCC2_TCC3


	#define GCLK_TCC0			GCLK_TCC0_TCC1
	#define GCLK_TCC1			GCLK_TCC0_TCC1
	#define GCLK_TCC2			GCLK_TCC2_TCC3
	#define GCLK_TCC3			GCLK_TCC2_TCC3


	#define CLK_TCC0			CLK_TCC0_TCC1
	#define CLK_TCC1			CLK_TCC0_TCC1
	#define CLK_TCC2			CLK_TCC2_TCC3
	#define CLK_TCC3			CLK_TCC2_TCC3

	// ++++++++++++++	I2C	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//	#define I2C					CONCAT2(HW::I2C,I2C_SERCOM_NUM)
	#define PIO_I2C				HW::PIOA 
	#define PIN_SDA				22 
	#define PIN_SCL				23 
	#define SDA					(1<<PIN_SDA) 
	#define SCL					(1<<PIN_SCL) 
	#define I2C_PMUX_SDA		PORT_PMUX_C 
	#define I2C_PMUX_SCL		PORT_PMUX_C 
	#define I2C_GEN_SRC			GEN_MCK
	#define I2C_GEN_CLK			GEN_MCK_CLK
	#define I2C_BAUDRATE		400000 

	// ++++++++++++++	MANCH	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define MAN_TRANSMIT_V2

	#define PIO_MANCH		HW::PIOA
	#define PIN_L1			8 
	#define PIN_L2			9 
	#define MANCH_PMUX		PORT_PMUX_F
	#define L1_WO_NUM		0
	#define L2_WO_NUM		1

	#define L1				(1UL<<PIN_L1)
	#define L2				(1UL<<PIN_L2)
	#define H1				0
	#define H2				0

	#define PIO_RXD			HW::PIOA
	#define PIN_RXD			0
	#define RXD				(1UL<<PIN_RXD)

	#define Pin_ManRcvIRQ_Set()		HW::PIOA->BSET(1)
	#define Pin_ManRcvIRQ_Clr()		HW::PIOA->BCLR(1)

	#define Pin_ManTrmIRQ_Set()		HW::PIOA->BSET(7)		
	#define Pin_ManTrmIRQ_Clr()		HW::PIOA->BCLR(7)		

	#define Pin_ManRcvSync_Set()	HW::PIOA->BSET(2)		
	#define Pin_ManRcvSync_Clr()	HW::PIOA->BCLR(2)		
	
	
	// ++++++++++++++	USART	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_UTXD0			HW::PIOA 
	#define PIO_URXD0			HW::PIOA 
	#define PIO_RTS0			HW::PIOA 

	#define PMUX_UTXD0			PORT_PMUX_C
	#define PMUX_URXD0			PORT_PMUX_C 

	#define UART0_TXPO			USART_TXPO_0 
	#define UART0_RXPO			USART_RXPO_1 

	#define PIN_UTXD0			12 
	#define PIN_URXD0			13 
	#define PIN_RTS0			21 

	#define UTXD0				(1UL<<PIN_UTXD0) 
	#define URXD0				(1UL<<PIN_URXD0) 
	#define RTS0				(1UL<<PIN_RTS0) 

	#define UART0_GEN_SRC		GEN_MCK
	#define UART0_GEN_CLK		GEN_MCK_CLK

	// ++++++++++++++	ADC		++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_ADC800			HW::PIOA
	#define PIO_ADCWFB			HW::PIOB

	#define PIN_ADC800			7 
	#define PIN_ADCWFB			4 
	
	#define ADC800				(1UL<<PIN_ADC800	) 
	#define ADCWFB				(1UL<<PIN_ADCWFB	) 

	// ++++++++++++++	FBHV		++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_FBHV			HW::PIOB
	#define PIN_FBHV			16
	#define FBHV				(1UL<<PIN_FBHV	) 

	// ++++++++++++++	PWM		++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_D_FIRE			HW::PIOA
	#define PIO_DNNK			HW::PIOA
	#define PIN_D_FIRE			16
	#define PIN_DNNKB			24
	#define PIN_DNNKM			25
	#define D_FIRE				(1<<PIN_D_FIRE)
	#define DNNKB				(1<<PIN_DNNKB)
	#define DNNKM				(1<<PIN_DNNKM)

	// ++++++++++++++ AD5312 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	#define AD5312_CHANNEL_LEVEL_B		0
	#define AD5312_CHANNEL_LEVEL_M		1

	#define PIO_SCLK			HW::PIOB
	#define PIO_MOSI			HW::PIOB
	#define PIO_SYNC			HW::PIOA

	#define PIN_SCLK			17
	#define PIN_MOSI			16 
	#define PIN_SYNC			20 

	#define SCLK				(1<<PIN_SCLK) 
	#define MOSI				(1<<PIN_MOSI) 
	#define SYNC				(1<<PIN_SYNC) 

	#define SPI_PMUX_SCLK		PORT_PMUX_C 
	#define SPI_PMUX_MOSI		PORT_PMUX_C 
	#define SPI_DIPO_BITS		SPI_DIPO(2)
	#define SPI_DOPO_BITS		SPI_DOPO(0) 

	#define SPI_GEN_SRC			GEN_MCK
	#define SPI_GEN_CLK			GEN_MCK_CLK
	#define SPI_BAUDRATE		8000000

	//#define Pin_SPI_IRQ_Set()	HW::PIOB->BSET(15)		
	//#define Pin_SPI_IRQ_Clr()	HW::PIOB->BCLR(15)		

	// ++++++++++++++	PIO INIT	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIOA_INIT_DIR		(PA01|PA02|PA03|PA04|PA05|PA06|PA07|L1|L2|PA10|PA11|PA15|D_FIRE|PA17|PA18|PA19|RTS0|PA27)
	#define PIOA_INIT_SET		(0)
	#define PIOA_INIT_CLR		(PA01|PA02|PA03|PA04|PA05|PA06|PA07|L1|L2|PA10|PA11|PA15|D_FIRE|PA17|PA18|PA19|RTS0|PA27)
	#define PIOA_TEST_MASK		(PA01|PA02|PA03|PA04|PA05|PA06|PA07|L1|L2|PA10|PA11|UTXD0|PA15|D_FIRE|PA17|PA18|PA19|PA20|RTS0|PA22|PA23|PA27)

	#define PIOB_INIT_DIR		(PB00|PB01|PB02|PB03|PB05|PB07|PB08|PB09|PB10|PB11|PB12|PB13|PB14|PB15|PB16|PB17|PB22|PB23|PB30|PB31)
	#define PIOB_INIT_SET		(0)
	#define PIOB_INIT_CLR		(PB00|PB01|PB02|PB03|PB05|PB07|PB08|PB09|PB10|PB11|PB12|PB13|PB14|PB15|PB16|PB17|PB22|PB23|PB30|PB31)
	#define PIOB_TEST_MASK		(PB00|PB01|PB02|PB03|PB05|PB07|PB08|PB09|PB10|PB11|PB12|PB13|PB14|PB15|PB16|PB17|PB22|PB23|PB30|PB31)

	//#define PIOC_INIT_DIR		(0)
	//#define PIOC_INIT_SET		(0)
	//#define PIOC_INIT_CLR		(0)
	//#define PIOC_TEST_MASK		(0)

	#define Pin_MainLoop_Set()	HW::PIOA->BSET(15)
	#define Pin_MainLoop_Clr()	HW::PIOA->BCLR(15)
	#define Pin_MainLoop_Tgl()	HW::PIOA->BTGL(15)	

#elif defined(WIN32) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define BAUD2CLK(x)				(x)
	#define MT(v)					(v)
	#define Pin_MainLoop_Set()	
	#define Pin_MainLoop_Clr()	

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




#endif // G26X_3_HW_CONF_H__11_01_2024__11_13
