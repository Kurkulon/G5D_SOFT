#ifndef HARDWARE_H__23_12_2013__11_37
#define HARDWARE_H__23_12_2013__11_37

#include "G5D_1_HW_CONF.H"
#include "types.h"
//#include "core.h"
#include "time.h"
#include "i2c.h"
//#include "hw_nand.h"
//#include "hw_rtm.h"
#include "MANCH\manch.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define WINDOW_SIZE	256

struct WINDSC
{
	WINDSC*		next;
	u16			rw;
	u16			fireCount;
	u16			genWorkTime;
	u16			temp;
	u16			winCount;
	u16			m_data[WINDOW_SIZE];
	u16			b_data[WINDOW_SIZE];
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void EnableGen();
extern void DisableGen();

inline u16 GetFireCount() {extern u16 fireCount; __disable_irq(); u16 v = fireCount; fireCount = 0; __enable_irq(); return v; }
inline u16 GetGenWorkTime() {extern u16 genWorkTimeMinutes; return genWorkTimeMinutes; }

extern void SetGenFreq(u16 freq);
extern void SetWindowCount(u16 wc);
extern void SetWindowTime(u16 wt);
extern void ResetGenWorkTime();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void AD5312_Set(byte channel, u16 dac);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern WINDSC*	AllocWinDsc();
extern void		FreeWinDsc(WINDSC* d);
extern WINDSC*	GetReadyWinDsc();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void InitHardware();
extern void UpdateHardware();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern u16 Get_FBPOW2();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#endif // HARDWARE_H__23_12_2013__11_37
