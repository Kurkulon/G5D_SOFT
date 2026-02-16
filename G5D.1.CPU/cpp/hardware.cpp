#include "hardware.h"
#include "types.h"
#include "core.h"
#include "time.h"
#include "CRC\CRC16_8005.h"
#include "list.h"
#include "PointerCRC.h"

#include "SEGGER_RTT\SEGGER_RTT.h"
//#include "hw_rtm.h"
//#include "hw_nand.h"
#include "DMA\DMA.h"
#include "MANCH\manch.h"
//#include <math.h>
#include "spi.h"

//#define WIN_TEST

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 SPI_CS_MASK[] = { SYNC };

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static S_SPIM	spi(SPI_SERCOM_NUM, PIO_SCLK, PIO_MOSI, 0, PIO_SYNC, SCLK, MOSI, 0, SPI_PMUX_SCLK, SPI_PMUX_MOSI, 0, SPI_CS_MASK, ArraySize(SPI_CS_MASK), 
	SPI_DIPO_BITS, SPI_DOPO_BITS, SPI_GEN_SRC, SPI_GEN_CLK, SPI_DMA_TX_CH, SPI_DMA_RX_CH);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <ARM\system_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <time_imp.h>

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <SPIM_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void WDT_Init()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_YELLOW "WDT Init ... ");

	#ifdef CPU_SAME53	

		HW::MCLK->APBAMASK |= APBA_WDT;
		HW::WDT->CONFIG = WDT_WINDOW_CYC512|WDT_PER_CYC1024;
		//while(HW::WDT->SYNCBUSY);

		#ifndef _DEBUG
		HW::WDT->CTRLA = WDT_ENABLE;
		#else
		HW::WDT->CTRLA = 0;
		#endif

		//while(HW::WDT->SYNCBUSY);

	#elif defined(CPU_XMC48)

		#ifndef _DEBUG
	
		//HW::WDT_Enable();

		//HW::WDT->WLB = OFI_FREQUENCY/2;
		//HW::WDT->WUB = (3 * OFI_FREQUENCY)/2;
		//HW::SCU_CLK->WDTCLKCR = 0|SCU_CLK_WDTCLKCR_WDTSEL_OFI;

		//HW::WDT->CTR = WDT_CTR_ENB_Msk|WDT_CTR_DSP_Msk;

		#else

		HW::WDT_Disable();

		#endif

	#endif

	SEGGER_RTT_WriteString(0, "OK\n");
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// GENTCC_OVF ->	WINTC_EVU(RETRIGER)
// 					BTCC_EV1(RETRIGER)
//					MTCC_EV1(RETRIGER)
// 
// DNNKB_EXTINT ->	DNNKB_EVSYS_CHANNEL -> BTCC0_EVSYS_USER(BTCC_EV_0)
// DNNKM_EXTINT ->	DNNKM_EVSYS_CHANNEL -> MTCC0_EVSYS_USER(MTCC_EV_0)
// 
// WINTC_OVF ->		BTCC_MC0 EVGEN_MC_0 -> bDMA(DMCH_TRIGSRC_MC0) -> DMCH_TCMPL B_DMA_IRQ(Win_DMA_IRQ)
//					MTCC_MC0 EVGEN_MC_0 -> mDMA(DMCH_TRIGSRC_MC0) -> DMCH_TCMPL M_DMA_IRQ(Win_DMA_IRQ)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef GEN_TCC

	#define genTCC						HW::GEN_TCC
	#define GENTCC_GEN					CONCAT2(GEN_,GEN_TCC)
	#define GENTCC_GEN_CLK				CONCAT2(CLK_,GEN_TCC) 
	#define GENTCC_IRQ					CONCAT2(GEN_TCC,_0_IRQ)
	#define GCLK_GENTCC					CONCAT2(GCLK_,GEN_TCC)
	#define PID_GENTCC					CONCAT2(PID_,GEN_TCC)

	#if (GENTCC_GEN_CLK > 100000000)
			#define GENTCC_PRESC_NUM	64
	#elif (GENTCC_GEN_CLK > 50000000)
			#define GENTCC_PRESC_NUM	16
	#elif (GENTCC_GEN_CLK > 20000000)
			#define GENTCC_PRESC_NUM	16
	#elif (GENTCC_GEN_CLK > 10000000)
			#define GENTCC_PRESC_NUM	8
	#elif (GENTCC_GEN_CLK > 5000000)
			#define GENTCC_PRESC_NUM	4
	#else
			#define GENTCC_PRESC_NUM	4
	#endif

	#define GENTCC_PRESC_DIV			CONCAT2(TCC_PRESCALER_DIV,GENTCC_PRESC_NUM)
	#define US2GENTCC(v)				(((v)*(GENTCC_GEN_CLK/GENTCC_PRESC_NUM/1000)+500)/1000)

	//#define GENTCC_EVSYS_USER			CONCAT3(EVSYS_USER_, GEN_TCC, _EV_0)
	#define GENTCC_EVENT_GEN			CONCAT3(EVGEN_, GEN_TCC, _OVF)
	#define GENTCC_EVSYS_CHANNEL		(GENTCC_EVENT_GEN|EVSYS_PATH_ASYNCHRONOUS|EVSYS_EDGSEL_RISING_EDGE)

	#define GENTCC_DMCH_TRIGSRC			CONCAT3(DMCH_TRIGSRC_, GEN_TCC, _OVF)

	inline void GENTCC_ClockEnable()	{ HW::GCLK->PCHCTRL[GCLK_GENTCC] = GENTCC_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(PID_GENTCC); }

#else
	#error  Must defined GEN_TCC
#endif


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// GENTCC_OVF ->	WINTC_EVU(RETRIGER)
// 					BTCC_EV1(RETRIGER)
//					MTCC_EV1(RETRIGER)
// 
// DNNKB_EXTINT ->	DNNKB_EVSYS_CHANNEL -> BTCC0_EVSYS_USER(BTCC_EV_0)
// DNNKM_EXTINT ->	DNNKM_EVSYS_CHANNEL -> MTCC0_EVSYS_USER(MTCC_EV_0)
// 
// WINTC_OVF ->		BTCC_MC0 EVGEN_MC_0 -> bDMA(DMCH_TRIGSRC_MC0) -> DMCH_TCMPL B_DMA_IRQ(Win_DMA_IRQ)
//					MTCC_MC0 EVGEN_MC_0 -> mDMA(DMCH_TRIGSRC_MC0) -> DMCH_TCMPL M_DMA_IRQ(Win_DMA_IRQ)

#ifdef B_TCC

	#define BTCC						HW::B_TCC
	#define BTCC_GEN					CONCAT2(GEN_,B_TCC)
	#define BTCC_GEN_CLK				CONCAT2(CLK_,B_TCC) 
	#define BTCC_IRQ					CONCAT2(B_TCC,_0_IRQ)
	#define GCLK_BTCC					CONCAT2(GCLK_,B_TCC)
	#define PID_BTCC					CONCAT2(PID_,B_TCC)

	#if (BTCC_GEN_CLK > 100000000)
			#define BTCC_PRESC_NUM	1
	#elif (BTCC_GEN_CLK > 50000000)
			#define BTCC_PRESC_NUM	1
	#elif (BTCC_GEN_CLK > 20000000)
			#define BTCC_PRESC_NUM	1
	#elif (BTCC_GEN_CLK > 10000000)
			#define BTCC_PRESC_NUM	1
	#elif (BTCC_GEN_CLK > 5000000)
			#define BTCC_PRESC_NUM	1
	#else
			#define BTCC_PRESC_NUM	1
	#endif

	#define BTCC_PRESC_DIV			CONCAT2(TCC_PRESCALER_DIV,BTCC_PRESC_NUM)
	#define US2BTCC(v)				(((v)*(BTCC_GEN_CLK/BTCC_PRESC_NUM)+500000)/1000000)

	//#define BTCC_EVENT_GEN		CONCAT3(EVGEN_, B_TCC, _MC_0)
	#define BTCC_COUNT_EVSYS_USER	CONCAT3(EVSYS_USER_, B_TCC, _EV_0)
	#define BTCC_RETRG_EVSYS_USER	CONCAT3(EVSYS_USER_, B_TCC, _EV_1)
	#define BTCC_CAPTR_EVSYS_USER	CONCAT3(EVSYS_USER_, B_TCC, _MC_0)

	#define BTCC_DMCH_TRIGSRC		CONCAT3(DMCH_TRIGSRC_, B_TCC, _MC0) 

	inline void BTCC_ClockEnable()	{ HW::GCLK->PCHCTRL[GCLK_BTCC] = BTCC_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(PID_BTCC); }

#else
	#error  Must defined B_TCC
#endif


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// GENTCC_OVF ->	WINTC_EVU(RETRIGER)
// 					BTCC_EV1(RETRIGER)
//					MTCC_EV1(RETRIGER)
// 
// DNNKB_EXTINT ->	DNNKB_EVSYS_CHANNEL -> BTCC0_EVSYS_USER(BTCC_EV_0)
// DNNKM_EXTINT ->	DNNKM_EVSYS_CHANNEL -> MTCC0_EVSYS_USER(MTCC_EV_0)
// 
// WINTC_OVF ->		BTCC_MC0 EVGEN_MC_0 -> bDMA(DMCH_TRIGSRC_MC0) -> DMCH_TCMPL B_DMA_IRQ(Win_DMA_IRQ)
//					MTCC_MC0 EVGEN_MC_0 -> mDMA(DMCH_TRIGSRC_MC0) -> DMCH_TCMPL M_DMA_IRQ(Win_DMA_IRQ)

#ifdef M_TCC

	#define MTCC						HW::M_TCC
	#define MTCC_GEN					CONCAT2(GEN_,M_TCC)
	#define MTCC_GEN_CLK				CONCAT2(CLK_,M_TCC) 
	#define MTCC_IRQ					CONCAT2(M_TCC,_0_IRQ)
	#define GCLK_MTCC					CONCAT2(GCLK_,M_TCC)
	#define PID_MTCC					CONCAT2(PID_,M_TCC)

	#if (MTCC_GEN_CLK > 100000000)
			#define MTCC_PRESC_NUM	1
	#elif (MTCC_GEN_CLK > 50000000)
			#define MTCC_PRESC_NUM	1
	#elif (MTCC_GEN_CLK > 20000000)
			#define MTCC_PRESC_NUM	1
	#elif (MTCC_GEN_CLK > 10000000)
			#define MTCC_PRESC_NUM	1
	#elif (MTCC_GEN_CLK > 5000000)
			#define MTCC_PRESC_NUM	1
	#else
			#define MTCC_PRESC_NUM	1
	#endif

	#define MTCC_PRESC_DIV			CONCAT2(TCC_PRESCALER_DIV,MTCC_PRESC_NUM)
	#define US2MTCC(v)				(((v)*(MTCC_GEN_CLK/MTCC_PRESC_NUM)+500000)/1000000)

	//#define MTCC_EVENT_GEN			CONCAT3(EVGEN_, M_TCC, _MC_0)
	#define MTCC_COUNT_EVSYS_USER	CONCAT3(EVSYS_USER_, M_TCC, _EV_0)
	#define MTCC_RETRG_EVSYS_USER	CONCAT3(EVSYS_USER_, M_TCC, _EV_1)
	#define MTCC_CAPTR_EVSYS_USER	CONCAT3(EVSYS_USER_, M_TCC, _MC_0)

	#define MTCC_DMCH_TRIGSRC		CONCAT3(DMCH_TRIGSRC_, M_TCC, _MC0) 

	inline void MTCC_ClockEnable()	{ HW::GCLK->PCHCTRL[GCLK_MTCC] = MTCC_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(PID_MTCC); }

#else
	#error  Must defined M_TCC
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN_TC

	#define winTC						HW::WIN_TC
	#define WINTC_GEN					CONCAT2(GEN_,WIN_TC)
	#define WINTC_GEN_CLK				CONCAT2(CLK_,WIN_TC) 
	#define WINTC_IRQ					CONCAT2(WIN_TC,_IRQ)
	#define WINTC_GCLK					CONCAT2(GCLK_,WIN_TC)
	#define WINTC_PID					CONCAT2(PID_,WIN_TC)

	#if (WINTC_GEN_CLK > 100000000)
			#define WINTC_PRESC_NUM		8
	#elif (WINTC_GEN_CLK > 50000000)
			#define WINTC_PRESC_NUM		4
	#elif (WINTC_GEN_CLK > 20000000)
			#define WINTC_PRESC_NUM		2
	#elif (WINTC_GEN_CLK > 10000000)
			#define WINTC_PRESC_NUM		1
	#elif (WINTC_GEN_CLK > 5000000)
			#define WINTC_PRESC_NUM		1
	#else
			#define WINTC_PRESC_NUM		1
	#endif

	#define WINTC_PRESC_DIV				CONCAT2(TC_PRESCALER_DIV,WINTC_PRESC_NUM)
	#define US2WINTC(v)					(((v)*(WINTC_GEN_CLK/1000/WINTC_PRESC_NUM)+500)/1000)

	#define WINTC_RETRG_EVSYS_USER		CONCAT3(EVSYS_USER_, WIN_TC, _EVU)
	#define WINTC_EVENT_GEN				CONCAT3(EVGEN_, WIN_TC, _OVF)
	#define WINTC_EVSYS_CHANNEL			(WINTC_EVENT_GEN|EVSYS_PATH_ASYNCHRONOUS|EVSYS_EDGSEL_RISING_EDGE)

	inline void WINTC_ClockEnable()		{ HW::GCLK->PCHCTRL[WINTC_GCLK] = WINTC_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(WINTC_PID); }

#else
	#error  Must defined WIN_TC
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// GENTCC_OVF ->	WINTC_EVU(RETRIGER)
// 					BTCC_EV1(RETRIGER)
//					MTCC_EV1(RETRIGER)
// 
// DNNKB_EXTINT ->	DNNKB_EVSYS_CHANNEL -> BTCC0_EVSYS_USER(BTCC_EV_0)
// DNNKM_EXTINT ->	DNNKM_EVSYS_CHANNEL -> MTCC0_EVSYS_USER(MTCC_EV_0)
// 
// WINTC_OVF ->		BTCC_MC0 EVGEN_MC_0 -> bDMA(DMCH_TRIGSRC_MC0) -> DMCH_TCMPL B_DMA_IRQ(Win_DMA_IRQ)
//					MTCC_MC0 EVGEN_MC_0 -> mDMA(DMCH_TRIGSRC_MC0) -> DMCH_TCMPL M_DMA_IRQ(Win_DMA_IRQ)

#define DNNKB_EXTINT				(PIN_DNNKB&15)
#define DNNKM_EXTINT				(PIN_DNNKM&15)

//#define DNNKB_EVSYS_GEN_EIC_EXTINT	EVGEN_EIC_EXTINT_12
//#define DNNKM_EVSYS_GEN_EIC_EXTINT	EVGEN_EIC_EXTINT_13

#define WFG_EVSYS_USER				CONCAT3(EVSYS_USER_, WFG_TC, _EVU)

#define DNNKB_EVSYS_CHANNEL			((EVGEN_EIC_EXTINT_0+DNNKB_EXTINT)|EVSYS_PATH_ASYNCHRONOUS|EVSYS_EDGSEL_RISING_EDGE)
#define DNNKB_EVSYS_USER			(BTCC_COUNT_EVSYS_USER)

#define DNNKM_EVSYS_CHANNEL			((EVGEN_EIC_EXTINT_0+DNNKM_EXTINT)|EVSYS_PATH_ASYNCHRONOUS|EVSYS_EDGSEL_RISING_EDGE)
#define DNNKM_EVSYS_USER			(MTCC_COUNT_EVSYS_USER)

//#define GEN_DIV						256
//#define GEN_PRESC					TCC_PRESCALER_DIV256
//#define WIN_DIV						16
//#define WIN_PRESC					TC_PRESCALER_DIV16

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//u32 m_ts[256];
//u32 b_ts[256];

u16 cur_ts_index = 0;
u16 max_ts_index = 64;
u16 windowCount = 64;
u16 windowTime = 64;
u16 genWorkTimeMinutes = 0;
u16 genWorkTimeMilliseconds = 0;
u16 genTimeOut = 0;
u16 fireCount = 0;

u32 gen_period = US2GENTCC(100000);// 4687;
u16 gen_freq = 10;

byte saveGenCount = 0;

WINDSC windsc_arr[4];

List<WINDSC>	readyWinList;
List<WINDSC>	freeWinList;

WINDSC *curWinDsc = 0;

static void SaveGenTime();
static void PrepareWin();

//u16 temp = 0;

static DMA_CH bDMA(B_DMACH_NUM);
static DMA_CH mDMA(M_DMACH_NUM);

#define B_DMA_IRQ CONCAT3(DMAC_,B_DMACH_NUM,_IRQ)
#define M_DMA_IRQ CONCAT3(DMAC_,M_DMACH_NUM,_IRQ)

//static void* eepromData = 0;
//static u16 eepromWriteLen = 0;
//static u16 eepromReadLen = 0;
//static u16 eepromStartAdr = 0;

//inline u16 ReverseWord(u16 v) { __asm	{ rev16 v, v };	return v; }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline void SaveGenTime() { saveGenCount = 1; }

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitWinLsit()
{
	for (u32 i = 0; i < ArraySize(windsc_arr); i++) freeWinList.Add(&windsc_arr[i]);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

WINDSC* AllocWinDsc()
{
	return freeWinList.Get();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FreeWinDsc(WINDSC* d)
{
	freeWinList.Add(d);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

WINDSC* GetReadyWinDsc()
{
	return readyWinList.Get();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetWindowCount(u16 wc)
{
	if (wc < 2)
	{
		wc = 2;
	}
	else if (wc > WINDOW_SIZE)
	{
		wc = WINDOW_SIZE;
	};

	windowCount = wc;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetWindowTime(u16 wt)
{
	if (wt < 2)
	{
		wt = 2;
	}
	else if (wt > 512)
	{
		wt = 512;
	};

	windowTime = wt;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetGenFreq(u16 freq)
{
	if (freq < 4)
	{
		freq = 4;
	}
	else if (freq > 50)
	{
		freq = 50;
	};

	gen_freq = freq;

	gen_period = ((GENTCC_GEN_CLK/GENTCC_PRESC_NUM) + (gen_freq>>1)) / gen_freq - 1;

	genTCC->PERBUF		= gen_period;
	genTCC->CTRLBSET	= TCC_CMD_UPDATE;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ResetGenWorkTime()
{
	genWorkTimeMinutes = 0;
	genWorkTimeMilliseconds = 0;

	SaveGenTime();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void EnableGen()
{
	PIO_D_FIRE->PINCFG[PIN_D_FIRE] = PINGFG_PMUXEN;

	genTimeOut = 60000;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void DisableGen()
{
	PIO_D_FIRE->PINCFG[PIN_D_FIRE] = 0;
	genTimeOut = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline bool CheckGenEnabled()
{
	return PIO_D_FIRE->PINCFG[PIN_D_FIRE] & PINGFG_PMUXEN;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadGenTime()
{
	static DSCI2C dsc;
	static u16 romAdr = 0;

	byte buf[sizeof(genWorkTimeMinutes)*2+8];

	bool loadVarsOk = false;

	romAdr = ReverseWord(FRAM_I2C_GENTIME_ADR);

	dsc.wdata = &romAdr;
	dsc.wlen = sizeof(romAdr);
	dsc.wdata2 = 0;
	dsc.wlen2 = 0;
	dsc.rdata = buf;
	dsc.rlen = sizeof(buf);
	dsc.adr = 0x50;

	if (I2C_AddRequest(&dsc))
	{
		while (!dsc.ready) { I2C_Update(); };
	};

	PointerCRC p(buf);

	for (byte i = 0; i < 2; i++)
	{
		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&genWorkTimeMinutes, sizeof(genWorkTimeMinutes));
		p.ReadW();

		if (p.CRC.w == 0) { loadVarsOk = true; break; };
	};

	if (!loadVarsOk)
	{
		genWorkTimeMinutes = 0;

		saveGenCount = 2;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateSaveGenTime()
{
	static DSCI2C dsc;

	static u16 romAdr = 0;
	static byte buf[sizeof(genWorkTimeMinutes) * 2 + 8];

	static byte i = 0;
	static TM32 tm;

	PointerCRC p(buf);

	switch (i)
	{
	case 0:

		if (saveGenCount > 0)
		{
			saveGenCount--;
			i++;
		};

		break;

	case 1:

		for (byte j = 0; j < 2; j++)
		{
			p.CRC.w = 0xFFFF;
			p.WriteArrayB(&genWorkTimeMinutes, sizeof(genWorkTimeMinutes));
			p.WriteW(p.CRC.w);
		};

		romAdr = ReverseWord(FRAM_I2C_GENTIME_ADR);

		dsc.wdata = &romAdr;
		dsc.wlen = sizeof(romAdr);
		dsc.wdata2 = buf;
		dsc.wlen2 = p.b-buf;
		dsc.rdata = 0;
		dsc.rlen = 0;
		dsc.adr = 0x50;

		tm.Reset();

		I2C_AddRequest(&dsc);

		i++;

		break;

	case 2:

		if (dsc.ready || tm.Check(100))
		{
			i = 0;
		};

		break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateGenTime()
{
	static u32 pt = 0;

	u32 t = GetMilliseconds();
	u32 dt = t - pt;

	pt = t;

	if (dt != 0)
	{
		if (CheckGenEnabled())
		{
			genWorkTimeMilliseconds += dt;

			if (genWorkTimeMilliseconds >= 60000)
			{
				genWorkTimeMinutes += 1;

				genWorkTimeMilliseconds -= 60000;

				SaveGenTime();
			};

			if (genTimeOut < dt) 
			{
				DisableGen();
			}
			else
			{
				genTimeOut -= dt;
			};
		};

		UpdateSaveGenTime();
	};

	if (curWinDsc == 0)	PrepareWin();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void PrepareWin()
{
	using namespace HW;

	curWinDsc = AllocWinDsc();

	if (curWinDsc == 0) return;

	winTC->CTRLA = TC_SWRST;
	while(winTC->SYNCBUSY);

	//WinTC->READREQ = TC_RCONT|0x18;

	winTC->CC16[0] = US2WINTC(windowTime) - 1; // временнќе окно
	winTC->CC16[1] = 1; // временнќе окно
	winTC->EVCTRL = TC_TCEI|TC_EVACT_RETRIGGER|TC_OVFEO;
	winTC->WAVE = TC_WAVEGEN_MFRQ;
	winTC->CTRLA = WINTC_PRESC_DIV;

	curWinDsc->fireCount = 0;
	curWinDsc->genWorkTime = genWorkTimeMinutes;
	curWinDsc->temp = 0;
	curWinDsc->winCount = windowCount;

	bDMA.ReadPeripheral(&BTCC->CC[0], curWinDsc->b_data, windowCount, DMCH_TRIGACT_BURST|BTCC_DMCH_TRIGSRC, DMDSC_BEATSIZE_HWORD|DMDSC_BLOCKACT_INT);
	mDMA.ReadPeripheral(&MTCC->CC[0], curWinDsc->m_data, windowCount, DMCH_TRIGACT_BURST|MTCC_DMCH_TRIGSRC, DMDSC_BEATSIZE_HWORD|DMDSC_BLOCKACT_INT);
	
	//HW::DMAC->CH[B_DMACH_NUM].INTENSET = DMCH_TCMPL;
	//HW::DMAC->CH[M_DMACH_NUM].INTENSET = DMCH_TCMPL;

	winTC->CTRLA |= TC_ENABLE;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void GenIRQ()
{
	using namespace HW;

	PIOA->BSET(17);

	genTCC->INTFLAG = ~0;

	if (curWinDsc != 0)
	{
		//winTC->CTRLA |= TC_ENABLE;

		//BTCC->CTRLBSET = TC_CMD_RETRIGGER;
		//MTCC->CTRLBSET = TC_CMD_RETRIGGER;

		if (CheckGenEnabled()) { fireCount += 1; };
	};

	PIOA->BCLR(17);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void Win_DMA_IRQ()
{
	using namespace HW;

	PIOA->BSET(19);

	u32 stat = DMAC->INTSTATUS;

	if (stat & (1<<B_DMACH_NUM))
	{
		bDMA.Disable();
	};

	if (stat & (1<<M_DMACH_NUM))
	{
		mDMA.Disable();
	};

	if ((DMAC->BUSYCH & ((1<<B_DMACH_NUM)|(1<<M_DMACH_NUM))) == 0)
	{
		winTC->CTRLBSET = TC_CMD_STOP;

		readyWinList.Add(curWinDsc); curWinDsc = 0;

		PrepareWin();
	};

	PIOA->BCLR(19);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// EVENT_GEN(GENTCC_OVF) ->	WINTC_RETRG_EVSYS_USER(WINTC_EVU-RETRIGER)
// 							BTCC_RETRG_EVSYS_USER(BTCC_EV1-RETRIGER)
//							MTCC_RETRG_EVSYS_USER(MTCC_EV1-RETRIGER)
// 
// EVENT_DNNKB(DNNKB_EXTINT) ->	DNNKB_EVSYS_CHANNEL -> BTCC_CAPTR_EVSYS_USER(BTCC_EV_0)
// EVENT_DNNKM(DNNKM_EXTINT) ->	DNNKM_EVSYS_CHANNEL -> MTCC_CAPTR_EVSYS_USER(MTCC_EV_0)
// 
// EVENT_WIN(WINTC_OVF) ->		BTCC_MC0 EVGEN_MC_0 -> bDMA(DMCH_TRIGSRC_MC0) -> DMCH_TCMPL B_DMA_IRQ(Win_DMA_IRQ)
//								MTCC_MC0 EVGEN_MC_0 -> mDMA(DMCH_TRIGSRC_MC0) -> DMCH_TCMPL M_DMA_IRQ(Win_DMA_IRQ)

void InitGen()
{
	using namespace HW;

	GENTCC_ClockEnable();
	BTCC_ClockEnable();
	MTCC_ClockEnable();
	WINTC_ClockEnable();

	HW::GCLK->PCHCTRL[EVENT_DNNKB+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;
	HW::GCLK->PCHCTRL[EVENT_DNNKM+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;
	HW::GCLK->PCHCTRL[EVENT_GEN+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;
	HW::GCLK->PCHCTRL[EVENT_WIN+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;


	PIO_D_FIRE->DIRSET = D_FIRE;
	PIO_D_FIRE->CLR(D_FIRE);

	PIO_D_FIRE->SetWRCONFIG(D_FIRE,		PORT_PMUX_F | PORT_WRPINCFG | PORT_WRPMUX);
	PIO_DNNK->SetWRCONFIG(DNNKB|DNNKM,	PORT_PMUX_A | PORT_WRPINCFG | PORT_PMUXEN | PORT_WRPMUX);

	EIC->CTRLA &= ~EIC_ENABLE;

	EIC->EVCTRL |= (EIC_EXTINT0 << DNNKB_EXTINT) | (EIC_EXTINT0 << DNNKM_EXTINT);
	EIC->SetConfig(DNNKB_EXTINT, 0, EIC_SENSE_BOTH);
	EIC->SetConfig(DNNKM_EXTINT, 0, EIC_SENSE_BOTH);
	EIC->INTENCLR = (EIC_EXTINT0 << DNNKB_EXTINT) | (EIC_EXTINT0 << DNNKM_EXTINT);
	
	EIC->CTRLA |= EIC_ENABLE;

	EVSYS->CH[EVENT_GEN].CHANNEL = GENTCC_EVSYS_CHANNEL;
	EVSYS->USER[WINTC_RETRG_EVSYS_USER] = EVENT_GEN+1;
	EVSYS->USER[BTCC_RETRG_EVSYS_USER] = EVENT_GEN+1;
	EVSYS->USER[MTCC_RETRG_EVSYS_USER] = EVENT_GEN+1;

	EVSYS->CH[EVENT_DNNKB].CHANNEL = DNNKB_EVSYS_CHANNEL;
	EVSYS->USER[DNNKB_EVSYS_USER] = EVENT_DNNKB+1;

	EVSYS->CH[EVENT_DNNKM].CHANNEL = DNNKM_EVSYS_CHANNEL;
	EVSYS->USER[DNNKM_EVSYS_USER] = EVENT_DNNKM+1;

	//EVSYS->CH[EVENT_BTCC_RETRG].CHANNEL = BTCC_EVENT_GEN|EVSYS_PATH_ASYNCHRONOUS|EVSYS_EDGSEL_RISING_EDGE;
	//EVSYS->USER[BTCC1_EVSYS_USER] = EVENT_BTCC_RETRG+1;

	//EVSYS->CH[EVENT_MTCC_RETRG].CHANNEL = MTCC_EVENT_GEN|EVSYS_PATH_ASYNCHRONOUS|EVSYS_EDGSEL_RISING_EDGE;
	//EVSYS->USER[MTCC1_EVSYS_USER] = EVENT_MTCC_RETRG+1;

	EVSYS->CH[EVENT_WIN].CHANNEL = WINTC_EVSYS_CHANNEL;
	EVSYS->USER[BTCC_CAPTR_EVSYS_USER] = EVENT_WIN+1;
	EVSYS->USER[MTCC_CAPTR_EVSYS_USER] = EVENT_WIN+1;


	BTCC->CTRLA = TCC_SWRST;
	MTCC->CTRLA = TCC_SWRST;

	while(BTCC->SYNCBUSY|MTCC->SYNCBUSY);


	BTCC->WAVE = TCC_WAVEGEN_NFRQ;
	MTCC->WAVE = TCC_WAVEGEN_NFRQ;

#ifdef WIN_TEST

	BTCC->EVCTRL = TCC_TCEI1|TCC_EVACT1_RETRIGGER|TCC_MCEI0|TCC_MCEO0;
	MTCC->EVCTRL = TCC_TCEI1|TCC_EVACT1_RETRIGGER|TCC_MCEI0|TCC_MCEO0;

	BTCC->CTRLA = TC_PRESCALER_DIV256|TCC_CPTEN0|TCC_ENABLE;
	MTCC->CTRLA = TC_PRESCALER_DIV256|TCC_CPTEN0|TCC_ENABLE;

#else

	BTCC->EVCTRL = TCC_TCEI0|TCC_EVACT0_COUNTEV|TCC_TCEI1|TCC_EVACT1_RETRIGGER|TCC_MCEI0|TCC_MCEO0;
	MTCC->EVCTRL = TCC_TCEI0|TCC_EVACT0_COUNTEV|TCC_TCEI1|TCC_EVACT1_RETRIGGER|TCC_MCEI0|TCC_MCEO0;

	BTCC->CTRLA = TCC_CPTEN0|TCC_ENABLE;
	MTCC->CTRLA = TCC_CPTEN0|TCC_ENABLE;

#endif


	VectorTableExt[B_DMA_IRQ] = Win_DMA_IRQ;
	CM4::NVIC->CLR_PR(B_DMA_IRQ);
	CM4::NVIC->SET_ER(B_DMA_IRQ);	

	VectorTableExt[M_DMA_IRQ] = Win_DMA_IRQ;
	CM4::NVIC->CLR_PR(M_DMA_IRQ);
	CM4::NVIC->SET_ER(M_DMA_IRQ);	

	//PIOA->SetWRCONFIG(PA18,	PORT_PMUX_E | PORT_WRPINCFG | PORT_PMUXEN | PORT_WRPMUX);

	InitWinLsit();

	PIOA->SetWRCONFIG(PA04,	PORT_PMUX_E | PORT_WRPINCFG | PORT_PMUXEN | PORT_WRPMUX);

	//PrepareWin();

	// Gen timer

	genTCC->CTRLA = TCC_SWRST;

	while(genTCC->SYNCBUSY);

	VectorTableExt[GENTCC_IRQ] = GenIRQ;
	CM4::NVIC->CLR_PR(GENTCC_IRQ);
	CM4::NVIC->SET_ER(GENTCC_IRQ);	

	genTCC->WAVE = TCC_WAVEGEN_NPWM;
	genTCC->PER = gen_period;
	genTCC->CC[0] = US2GENTCC(10);

	genTCC->INTENCLR = ~0;
	genTCC->INTENSET = TCC_OVF;

	genTCC->INTFLAG = ~0;

	genTCC->EVCTRL = TCC_OVFEO;

	genTCC->CTRLA = GENTCC_PRESC_DIV;
	genTCC->CTRLA = GENTCC_PRESC_DIV|TCC_ENABLE;

	PIOB->SetWRCONFIG(PB10,		PORT_PMUX_G | PORT_WRPINCFG | PORT_WRPMUX);

	SetGenFreq(gen_freq);

	genTimeOut = 60000;

	LoadGenTime();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 ad5312_data[2] = { 2048, 2048 };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void AD5312_Set(byte channel, u16 dac)
{
	ad5312_data[channel&1] = dac & 0x3FF;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Init_AD5312()
{
	spi.Connect(SPI_BAUDRATE);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update_AD5312()
{
	static byte state = 0;
	static byte chn = 0;
	static U16u data(0);

	static TM32 tm;

	static DSCSPI dsc;

	switch (state)
	{
		case 0:

			if (tm.Check(101))
			{
				dsc.adr = ReverseWord((chn<<15) | (ad5312_data[chn] << 2)); chn = (chn + 1) & 1;
				dsc.alen = 2;
				dsc.csnum = 0;
				dsc.wdata = 0;
				dsc.rdata = 0;
				dsc.wlen = 0;
				dsc.rlen = 0;

				spi.AddRequest(&dsc);

				state++;
			};

			break;

		case 1:

			spi.Update();

			if (dsc.ready)
			{
				state = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitHardware()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_YELLOW "Hardware Init ... ");

#ifdef CPU_SAME53	

	using namespace HW;

	//	HW::PIOA->BSET(13);

	HW::GCLK->GENCTRL[GEN_32K]		= GCLK_DIV(1)	|GCLK_SRC_OSCULP32K	|GCLK_GENEN;

	HW::GCLK->GENCTRL[GEN_1M]		= GCLK_DIV(CLKIN_MHz)	|GCLK_SRC_XOSC0		|GCLK_GENEN		|GCLK_OE;

	HW::GCLK->GENCTRL[GEN_16M]		= GCLK_DIV(1)	|GCLK_SRC_XOSC0		|GCLK_GENEN;

	//	HW::GCLK->GENCTRL[GEN_500K] 	= GCLK_DIV(50)	|GCLK_SRC_XOSC1		|GCLK_GENEN;

	//HW::PIOA->SetWRCONFIG(PA17, PORT_PMUX_M|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_PULLEN);

	//HW::GCLK->GENCTRL[GEN_EXT32K]	= GCLK_DIV(1)	|GCLK_SRC_GCLKIN	|GCLK_GENEN		;


	HW::MCLK->APBAMASK |= APBA_EIC;
	HW::GCLK->PCHCTRL[GCLK_EIC] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;

	HW::MCLK->APBBMASK |= APBB_EVSYS;

	HW::GCLK->PCHCTRL[GCLK_SERCOM_SLOW]		= GCLK_GEN(GEN_32K)|GCLK_CHEN;	// 32 kHz

#endif

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_GREEN "OK\n");

	Init_time();
	I2C_Init();
	InitManRecieve();
	InitManTransmit();
	InitGen();
	Init_AD5312();

	WDT_Init();

	HW::PIOA->BCLR(13);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{
	static byte i = 0;

	//	static Deb db(false, 20);

#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateGenTime()	);
		CALL( Update_AD5312();	);
		CALL( I2C_Update();		);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
