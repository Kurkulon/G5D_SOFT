#include "hardware.h"
//#include "options.h"
//#include "hw_emac.h"
#include <CRC\CRC16.h>
#include <ComPort\ComPort.h>
#include <CRC\CRC16_CCIT.h>
#include <list.h>
#include <PointerCRC.h>
#include <SEGGER_RTT\SEGGER_RTT.h>
#include "hw_com.h"
//#include "G_TRM.h"
#include "TaskList.h"
#include "FLASH\nand_ecc.h"
#include "MQCODER\mqcoder.h"
#include <string.h>

//#include "G_TRM.H"


enum { VERSION = 0x101 };

//#pragma O3
//#pragma Otime

#ifndef _DEBUG
	static const bool __debug = false;
#else
	static const bool __debug = true;
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct MainVars // NonVolatileVars  
{
	u32 timeStamp;

	u16 numDevice;
	u16 genFreq;		//	Частота генератора(Гц), 
	u16 winCount;		//	Количество временных окон(шт), 
	u16 winTime;		//	Длительность временного окна(мкс),
	u16 bLevel;			//	Уровень дискриминации МЗ(у.е), 
	u16 mLevel;			//	Уровень дискриминации БЗ(у.е),
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static MainVars mv;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 fps;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 manRcvData[10];
static u16 manTrmData[4096];
static u16 manPckData[128 + WINDOW_SIZE*4 + 16];
static u16 manUnpData[128 + WINDOW_SIZE*4 + 16];
static u16 manTrmBaud = 0;
static u16 tlsTrmBaud = 0;

static const u16 manReqWord = 0x7000;
static const u16 manReqMask = 0xFF00;

static const u16 tlsReqWord = 0x0000;
static const u16 tlsReqMask = 0xFF00;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//u16 txbuf[128 + 512 + 16];

//static u16 reqFireVoltage = 0;
//static u16 curFireVoltage = 300;


static u16 verDevice = VERSION;
static bool numDevValid = false;

i16 temp = 0;

u32 m_ts[WINDOW_SIZE];
u32 b_ts[WINDOW_SIZE];

static byte svCount = 0;

//static Rsp72 *curRsp72 = 0;

u16 fireAmp = 0;
u16 fireFreq = 3000;

i16 loc = 0;
i16 loc_min = 0x7FFF;
i16 loc_max = 0x8000;
u32 loc_gk = 0;
i16 loc_tension = 0;
u32 loc_period = 0;
u16 loc_req_count = 0;
u16 framErrorMask = 0;

enum { FRAM_ERROR_LOADVARS = 0, FRAM_ERROR_ECC, FRAM_ERROR_PARECC, FRAM_CORR_ECC, FRAM_NOACKR, FRAM_NOACKW, FRAM_LOAD_OK };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SaveMainParams()
{
	svCount = 1;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_00(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	__packed u16 *start = manTrmData;

	data = manTrmData;

	*(data++)	= (manReqWord & manReqMask) | 0;
	*(data++)	= mv.numDevice;
	*(data++)	= verDevice;

	mtb->data1 = manTrmData;
	mtb->len1 = data - start;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_10(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	__packed u16 *start = manTrmData;

	data = manTrmData;

	*(data++)	= (manReqWord & manReqMask) | 0x10;		//	1. Ответное слово (принятая команда)
	*(data++)	= mv.genFreq;							//	2. Частота генератора(Гц), 				
	*(data++)	= mv.winCount;							//	3. Количество временных окон(шт), 					
	*(data++)	= mv.winTime;							//	4. Длительность временного окна(мкс), 						
	*(data++)	= mv.mLevel;							//	5. Уровень дискриминации МЗ(у.е), 					
	*(data++)	= mv.bLevel;							//	6. Уровень дискриминации БЗ(у.е),					

	mtb->data1 = manTrmData;
	mtb->len1 = data - start;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 1 || mtb == 0) return false;

	if (data[0] & 1)
	{
		EnableGen();
	}
	else
	{
		DisableGen();
	};

	__packed u16 *start = manTrmData;

	manTrmData[0] = data[0];					//	1. Ответное слово (принятая команда)
	
	data = manTrmData;
	data++;

	u16 wc;
												
	*(data++)	= GetFireCount();				//	2. Количество вспышек(ushort)		
	*(data++)	= GetGenWorkTime();				//	3. Наработка генератора(мин)(ushort)						
	*(data++)	= temp;							//	4. Температура в приборе(0.1 гр)(short)									
	*(data++)	= wc = mv.winCount;				//	5. Количество временных окон(шт)					
	*(data++)	= mv.winTime;					//	6. Длительность временного окна(мкс)						
	*(data++)	= loc;							//	7. Локатор
	*(data++)	= loc_min;						//	8. Локатор минимум
	*(data++)	= loc_max;						//	9. Локатор максимум
	*(data++)	= MIN(loc_gk, 65535);			//	10. ГК(имп/период)
	*(data++)	= loc_period;					//	11,12. Период(мс)(uint32)
	*(data++)	= loc_period>>16;				
	*(data++)	= loc_req_count;				//	13. Счётчик запросов ЛК-ГК			
	*(data++)	= framErrorMask;				//	14. Статус ошибог FRAM
	*(data++)	= Get_FBPOW2();					//	15. Напряжение жилы (0.1В)
	
	framErrorMask = 0;

	loc_gk = 0;
	loc_period = 0;
	loc_min = 0x7FFF;
	loc_max = 0x8000;
	//u16 n = 6;

	for (u16 i = 0; i < wc; i++)
	{
		u32 tm = m_ts[i]; m_ts[i] = 0;
		u32 tb = b_ts[i]; b_ts[i] = 0;

		data[0]		= MIN(tm, 0xFFFF);			//	15..x. Спектр МЗ(ushort)
		data[wc]	= MIN(tb, 0xFFFF);			//	x..y. Спектр БЗ(ushort)

		data++;
	};

	data += wc;

	#ifdef COMPRESS_REQ20

		manPckData[0] = manTrmData[0];
		manPckData[1] = data-start-1;

		byte *src = (byte*)(manTrmData+1);
		byte *dst = (byte*)(manPckData+2);

		u32 plen = MQcompressFast(src, manPckData[1]*2, dst);

		dst[plen] = ~0;

		//MQdecompress(dst, plen, (byte*)manUnpData, manPckData[1]*2);

		mtb->data1 = manPckData;
		mtb->len1 = 2 + (plen+1)/2;

	#else

		mtb->data1 = manTrmData;
		mtb->len1 = data-start;

	#endif

	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static bool RequestMan_30(u16 *data, u16 len, MTB* mtb)
//{
//	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;
//
//	__packed u16 *start = manTrmData;
//
//	data = manTrmData;
//
//	*(data++)	= (manReqWord & manReqMask) | 0x30;		//	1. ответное слово
//	*(data++)	= loc;									//	2. Локатор
//	*(data++)	= loc_min;								//	3. Локатор минимум			
//	*(data++)	= loc_max;								//	4. Локатор максимум					
//	*(data++)	= loc_gk;								//	5. ГК(имп/период)			
//	*(data++)	= loc_period;							//	6,7. Период(мс)(uint32)			
//	*(data++)	= loc_period>>16;						//	6,7. Период(мс)(uint32)			
//
//	mtb->data1 = manTrmData;
//	mtb->len1 = data - start;
//	mtb->data2 = 0;
//	mtb->len2 = 0;
//
//	return true;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_80(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch (data[1])
	{
		case 1:

			mv.numDevice = data[2];

			break;

		case 2:

			manTrmBaud = data[2] - 1;	//SetTrmBoudRate(data[2]-1);

			break;
	};

	manTrmData[0] = (manReqWord & manReqMask) | 0x80;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_90(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch(data[1])
	{
		case 0x01:	SetGenFreq(							mv.genFreq	= LIM(data[2], 4, 50)			);	break;	//	0x1 - Частота генератора(4..50 Гц), 
		case 0x02:	SetWindowCount(						mv.winCount	= LIM(data[2], 2, WINDOW_SIZE)	);	break;	//	0x2 - Количество временных окон(2..1024 шт), 
		case 0x03:	SetWindowTime(						mv.winTime	= LIM(data[2], 2, 512)			);	break;	//	0x3 - Длительность временного окна(2..2048 мкс), 
		case 0x04:	AD5312_Set(AD5312_CHANNEL_LEVEL_M,	mv.mLevel	= MIN(data[2], 0x3FF)			);	break;	//	0x4 - Уровень дискриминации МЗ(у.е), 
		case 0x05:	AD5312_Set(AD5312_CHANNEL_LEVEL_B,	mv.bLevel	= MIN(data[2], 0x3FF)			);	break;	//	0x5 - Уровень дискриминации БЗ(у.е),

		default:

			return false;
	};

	manTrmData[0] = (manReqWord & manReqMask) | 0x90;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_A0(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch(data[1])
	{
		case 0x01:	ResetGenWorkTime();	break;	

		default:

			return false;
	};

	manTrmData[0] = (manReqWord & manReqMask) | 0xA0;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_F0(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	SaveMainParams();

	manTrmData[0] = (manReqWord & manReqMask) | 0xF0;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan(u16 *buf, u16 len, MTB* mtb)
{
	if (buf == 0 || len == 0 || mtb == 0) return false;

	if ((buf[0] & manReqMask) != manReqWord) return false;

	bool r = false;

	byte i = (buf[0]>>4)&0xF;

	switch (i)
	{
		case 0x0: 	r = RequestMan_00(buf, len, mtb); break;
		case 0x1: 	r = RequestMan_10(buf, len, mtb); break;
		case 0x2: 	r = RequestMan_20(buf, len, mtb); break;
	//	case 0x3:	r = RequestMan_30(buf, len, mtb); break;
		case 0x8: 	r = RequestMan_80(buf, len, mtb); break;
		case 0x9:	r = RequestMan_90(buf, len, mtb); break;
		case 0xA:	r = RequestMan_A0(buf, len, mtb); break;
		case 0xF:	r = RequestMan_F0(buf, len, mtb); break;
	};

	if (r) { mtb->baud = manTrmBaud; };

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestTLS_00(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	__packed u16 *start = manTrmData;

	data = manTrmData;

	*(data++)	= (tlsReqWord & tlsReqMask) | 0;
	*(data++)	= mv.numDevice;
	*(data++)	= 0x108;

	mtb->data1 = manTrmData;
	mtb->len1 = data - start;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestTLS_10(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	__packed u16 *start = manTrmData;

	data = manTrmData;

	*(data++)	= (tlsReqWord & tlsReqMask) | 0x10;		//	1. Ответное слово (принятая команда)

	mtb->data1 = manTrmData;
	mtb->len1 = data - start;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestTLS_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 1 || mtb == 0) return false;

	__packed u16 *start = manTrmData;

	manTrmData[0] = data[0];					//	1. ответное слово			

	data = manTrmData;
	data++;

	*(data++)	= Get_FBPOW2();				//	2. Напряжение питания (0.1 В short)			
	*(data++)	= temp;						//	3. Температура в приборе (0.1 гр short)					
	*(data++)	= 10;						//	4. Амплитуда запроса (у.е short)								

	mtb->data1 = manTrmData;
	mtb->len1 = data-start;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestTLS_80(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch (data[1])
	{
		case 1:

			//mv.numDevice = data[2];

			break;

		case 2:

			tlsTrmBaud = data[2] - 1;	//SetTrmBoudRate(data[2]-1);

			break;
	};

	manTrmData[0] = (tlsReqWord & tlsReqMask) | 0x80;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestTLS_90(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	//switch(data[1])
	//{
	//case 0x01:	SetGenFreq(							mv.genFreq	= LIM(data[2], 4, 50)			);	break;	//	0x1 - Частота генератора(4..50 Гц), 
	//case 0x02:	SetWindowCount(						mv.winCount	= LIM(data[2], 2, WINDOW_SIZE)	);	break;	//	0x2 - Количество временных окон(2..1024 шт), 
	//case 0x03:	SetWindowTime(						mv.winTime	= LIM(data[2], 2, 512)			);	break;	//	0x3 - Длительность временного окна(2..2048 мкс), 
	//case 0x04:	AD5312_Set(AD5312_CHANNEL_LEVEL_M,	mv.mLevel	= MIN(data[2], 0x3FF)			);	break;	//	0x4 - Уровень дискриминации МЗ(у.е), 
	//case 0x05:	AD5312_Set(AD5312_CHANNEL_LEVEL_B,	mv.bLevel	= MIN(data[2], 0x3FF)			);	break;	//	0x5 - Уровень дискриминации БЗ(у.е),

	//default:

	//	return false;
	//};

	manTrmData[0] = (tlsReqWord & tlsReqMask) | 0x90;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestTLS_F0(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	SaveMainParams();

	manTrmData[0] = (tlsReqWord & tlsReqMask) | 0xF0;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestTLS(u16 *buf, u16 len, MTB* mtb)
{
	if (buf == 0 || len == 0 || mtb == 0) return false;

	if ((buf[0] & tlsReqMask) != tlsReqWord) return false;

	bool r = false;

	byte i = (buf[0]>>4)&0xF;

	switch (i)
	{
	case 0x0: 	r = RequestTLS_00(buf, len, mtb); break;
	case 0x1: 	r = RequestTLS_10(buf, len, mtb); break;
	case 0x2: 	r = RequestTLS_20(buf, len, mtb); break;
	case 0x8: 	r = RequestTLS_80(buf, len, mtb); break;
	case 0x9:	r = RequestTLS_90(buf, len, mtb); break;
	case 0xF:	r = RequestTLS_F0(buf, len, mtb); break;
	};

	if (r) { mtb->baud = tlsTrmBaud; };

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMan()
{
	static MTB mtb;
	static MRB mrb;

	static byte i = 0;

	static u16 rcvLen = 0;

	static CTM32 tm;

	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;

//	u16 c;

	switch (i)
	{
		case 0:

//			HW::P5->BSET(7);

			mrb.data = manRcvData;
			mrb.maxLen = ArraySize(manRcvData);
			RcvManData(&mrb);

			i++;

			break;

		case 1:

			ManRcvUpdate();

			if (mrb.ready)
			{
				tm.Reset();

				if (!mrb.OK || mrb.len == 0)
				{
					i = 0;	
				}
				else
				{
					if ((manRcvData[0] & manReqMask) == manReqWord && RequestMan(manRcvData, mrb.len, &mtb))
					{
						i++;
					}
					else if ((manRcvData[0] & tlsReqMask) == tlsReqWord && RequestTLS(manRcvData, mrb.len, &mtb))
					{
						i++;
					}
					else
					{
						i += 3;
					};
				};
			}
			else if (mrb.len > 0)
			{

			};

			break;

		case 2:

			if (tm.Check(US2CTM(200)))
			{
				SendManData(&mtb);

				i++;
			};

			break;

		case 3:

			if (mtb.ready)
			{
				i = 0;
			};

			break;

		case 4:

			//for (u32 n = 0; n < mrb.len; n++) manRcvData[n] = ReverseWord(manRcvData[n]);

			wb.data = manRcvData;
			wb.len = mrb.len*2;

			comdsp.Write(&wb);

			i++;

			break;

		case 5:

			if (!comdsp.Update())
			{
				rb.data = manTrmData;
				rb.maxLen = sizeof(manTrmData);
				comdsp.Read(&rb, MS2COM(10), US2COM(100));

				HW::PIOB->BSET(15);

				i++;
			};

			break;

		case 6:
		{
			bool c = !comdsp.Update();

			if (tm.Timeout(US2CTM(200)))
			{
				if (rb.recieved || rb.len > 2)
				{
					HW::PIOB->BCLR(15);

					mtb.baud	= tlsTrmBaud;
					mtb.data1	= manTrmData;
					mtb.len1	= 0;
					mtb.data2	= 0;
					mtb.len2	= 0;
					mtb.lenptr = &rcvLen;

					rcvLen = ArraySize(manTrmData); //rb.len/2;

					SendManData(&mtb);

					HW::PIOB->BSET(15);

					i++;
				}
				else if (c)
				{
					i = 0;
				};
			};

			break;
		};

		case 7:
		{
			bool c = !comdsp.Update();

			if (rb.recieved || c)
			{
				rcvLen = rb.len/2;

				i -= 4;

				HW::PIOB->BCLR(15);
			};

			break;
		};
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateCom()
{
	static byte i = 0;
	static CTM32 ctm;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
//	static MTB mtb;
	static u16 buf[16];
	
	switch(i)
	{
		case 0:

			buf[0] = 0x220;

			wb.data = buf;
			wb.len = 2;

			comdsp.Write(&wb);

			i++;

			break;

		case 1:

			if (!comdsp.Update())
			{
				rb.data = buf;
				rb.maxLen = sizeof(buf);
				comdsp.Read(&rb, MS2COM(10), US2COM(500));

				i++;
			};

			break;

		case 2:

			if (!comdsp.Update())
			{
				if (rb.recieved && rb.len >= 16 && buf[0] == 0x220)
				{
					loc				= buf[1];
					loc_min			= MIN((i16)buf[2], loc_min);
					loc_max			= MAX((i16)buf[2], loc_max);
					loc_gk			+= buf[4];
					loc_tension		= buf[5];
					loc_period		+= buf[6]|(buf[7]<<16);
					loc_req_count	+= 1;
				};

				i++;
			};

			break;

		case 3:

			if (ctm.Check(MS2CTM(100)))
			{
				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateTemp()
{
	static byte i = 0;

	static DSCI2C dsctemp;//, dsc2;

//	static byte reg = 0;
	static u16 rbuf = 0;
	static byte buf[10];

	static TM32 tm;

	switch (i)
	{
		case 0:

			if (tm.Check(100))
			{
				if (!__debug) { HW::ResetWDT(); };

				buf[0] = 0;

				dsctemp.adr = 0x49;
				dsctemp.wdata = buf;
				dsctemp.wlen = 1;
				dsctemp.rdata = &rbuf;
				dsctemp.rlen = 2;
				dsctemp.wdata2 = 0;
				dsctemp.wlen2 = 0;

				if (I2C_AddRequest(&dsctemp))
				{
					i++;
				};
			};

			break;

		case 1:

			if (dsctemp.ready)
			{
				if (dsctemp.ack && dsctemp.readedLen == dsctemp.rlen)
				{
					i32 t = (i16)ReverseWord(rbuf);

					temp = (t * 10 + 64) / 128;
				};

				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static u16 prevDstFV = 0;
//static u16 prevCurFV = 0;
//static i16 dCurFV = 0;
//static i16 dDstFV = 0;
/*
static void UpdateHV()
{
	static byte i = 0;
	static DSCI2C dsc;
	static byte wbuf[4];
	static byte rbuf[4];
	static TM32 tm;
	//static CTM32 ctm;
//	static i32 filtFV = 0;
	//static i32 filtMV = 0;
	static u16 correction = 0x200;
	static u16 dstFV = 0;

	//static u16 count = 0;

	//if (!ctm.Check(US2CCLK(50))) return;

	switch (i)
	{
		case 0:

			if (tm.Check(10))
			{
				wbuf[0] = 2;
				wbuf[1] = 0;
				wbuf[2] = 0;

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 1:

			if (dsc.ready)
			{
				wbuf[0] = 3;	
				wbuf[1] = 1;	
				wbuf[2] = 0;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 2:

			if (dsc.ready)
			{
				wbuf[0] = 4;	
				wbuf[1] = 1;	
				wbuf[2] = 1;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 3:

			if (dsc.ready)
			{
				wbuf[0] = 5;	
				wbuf[1] = 0;	
				wbuf[2] = 0;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 4:

			if (dsc.ready)
			{
				//curFireVoltage = GetCurFireVoltage();

				u16 t = reqFireVoltage;

				if (t > curFireVoltage)
				{
					if (correction < 0x3FF)
					{
						correction += 1;
					};
				}
				else if (t < curFireVoltage)
				{
					if (correction > 0)
					{
						correction -= 1;
					};
				};

				if (reqFireVoltage > dstFV)
				{
					dstFV += 2;
				}
				else if (reqFireVoltage < dstFV)
				{
					dstFV = reqFireVoltage;
				};

				//dstFV += (i16)reqFireVoltage - (dstFV+16)/32;

				t = dstFV;//(dstFV+16)/32;

				u32 k = (0x1E00 + correction) >> 3;

				t = (k*t+128) >> 10;

				if (t > 955) t = 955;

				t = ~(((u32)t * (65535*16384/955)) / 16384); 

				//if (DacHvInverted()) t = ~t;

				wbuf[0] = 8;	
				wbuf[1] = t>>8;
				wbuf[2] = t;

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = rbuf;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				//count++;

				//if (count >= 100)
				//{
				//	count = 0;

				//	dCurFV = (i16)curFireVoltage - (i16)prevCurFV;
				//	dDstFV = (i16)dstFV - (i16)prevDstFV;

				//	prevDstFV = dstFV;
				//	prevCurFV = curFireVoltage;
				//};

				i++;
			};

			break;

		case 5:

			if (dsc.ready)
			{
				i = 0;
			};

			break;


	};
}
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitMainVars()
{
	mv.numDevice	= 11111;
	mv.genFreq		= 10;	
	mv.winCount		= 64;	
	mv.winTime		= 64;	
	mv.bLevel		= 0;		
	mv.mLevel		= 0;		
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadVars()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN "Load Vars ... ");

	static DSCI2C dsc;
	static u16 romAdr = 0;
	
	byte buf[sizeof(mv)*2+16];

	bool c2 = false;

	bool loadVarsOk = false;
	
	framErrorMask = 0;

	romAdr = ReverseWord(FRAM_I2C_MAINVARS_ADR);

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

	if (!dsc.ack) framErrorMask |= (1<<FRAM_NOACKR);

	PointerCRC p(buf);

	u32 err = 0, corrErr = 0, parErr = 0;

	for (byte i = 0; i < 2; i++)
	{
		Nand_ECC_Corr(p.b, sizeof(mv)+2, 256, p.b+sizeof(mv)+2, &err, &corrErr, &parErr);

		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&mv, sizeof(mv));
		p.ReadW();
		p.b += 3;

		if (p.CRC.w == 0) { c2 = true; break; };
	};

	if (!c2)		framErrorMask |= 1<<FRAM_ERROR_LOADVARS;
	if (err)		framErrorMask |= 1<<FRAM_ERROR_ECC;
	if (corrErr)	framErrorMask |= 1<<FRAM_CORR_ECC;
	if (parErr)		framErrorMask |= 1<<FRAM_ERROR_PARECC;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "FRAM I2C - "); SEGGER_RTT_WriteString(0, (c2) ? (RTT_CTRL_TEXT_BRIGHT_GREEN "OK\n") : (RTT_CTRL_TEXT_BRIGHT_RED "ERROR\n"));

	loadVarsOk = c2;

	if (!loadVarsOk)
	{
		InitMainVars();

		svCount = 2;
	};

	numDevValid = true;

	SetGenFreq(							mv.genFreq	);
	SetWindowCount(						mv.winCount	);
	SetWindowTime(						mv.winTime	);
	AD5312_Set(AD5312_CHANNEL_LEVEL_B,	mv.bLevel	);
	AD5312_Set(AD5312_CHANNEL_LEVEL_M,	mv.mLevel	);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SaveVars()
{
	static DSCI2C dsc;
	static u16 romAdr = 0;
	static byte buf[sizeof(mv) * 2 + 16];

	static byte i = 0;
	static TM32 tm;

	PointerCRC p(buf);

	switch (i)
	{
		case 0:

			if (svCount > 0)
			{
				svCount--;
				i++;
			};

			break;

		case 1:

			mv.timeStamp = GetMilliseconds();

			for (byte j = 0; j < 2; j++)
			{
				byte* start = p.b;
				p.CRC.w = 0xFFFF;
				p.WriteArrayB(&mv, sizeof(mv));
				p.WriteW(p.CRC.w);
				
				Nand_ECC_Calc(start, p.b-start, 256, p.b); p.b += 3; 
			};

			romAdr = ReverseWord(FRAM_I2C_MAINVARS_ADR);

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

static void TestFRAM()
{
	static DSCI2C dscfram;
	static u16 romAdr = 0;

	static byte state = 0;
	static TM32 tm;

	static MainVars vv;

	static byte buf[sizeof(vv)*2+16];

	static byte count = 0;

	static u32 timeStamp = 0;

	bool loadVarsOk = false;

	switch(state)
	{
		case 0:

			if (tm.Check(100))
			{
				memset(buf, 0, sizeof(buf));

				u32 tt = timeStamp = GetMilliseconds();

				vv.timeStamp = tt;

				vv.numDevice	= tt += 101;
				vv.genFreq		= tt += 101;	
				vv.winCount		= tt += 101;	
				vv.winTime		= tt += 101;	
				vv.bLevel		= tt += 101;		
				vv.mLevel		= tt += 101;		

				PointerCRC p(buf);

				for (byte j = 0; j < 2; j++)
				{
					byte* start = p.b;
					p.CRC.w = 0xFFFF;
					p.WriteArrayB(&vv, sizeof(vv));
					p.WriteW(p.CRC.w);

					Nand_ECC_Calc(start, p.b-start, 256, p.b); p.b += 3; 
				};

				romAdr = ReverseWord(0x1000);

				dscfram.wdata = &romAdr;
				dscfram.wlen = sizeof(romAdr);
				dscfram.wdata2 = buf;
				dscfram.wlen2 = p.b-buf;
				dscfram.rdata = 0;
				dscfram.rlen = 0;
				dscfram.adr = 0x50;

				tm.Reset();

				I2C_AddRequest(&dscfram);

				state++;
			};

			break;

		case 1:

			if (dscfram.ready)
			{
				if (!dscfram.ack) framErrorMask |= 1<<FRAM_NOACKW;

				tm.Reset();

				state++;
			};

			break;

		case 2:

			if (tm.Check(10))
			{
				romAdr = ReverseWord(0x1000);

				dscfram.wdata = &romAdr;
				dscfram.wlen = sizeof(romAdr);
				dscfram.wdata2 = 0;
				dscfram.wlen2 = 0;
				dscfram.rdata = buf;
				dscfram.rlen = sizeof(buf);
				dscfram.adr = 0x50;

				memset(buf, 0, sizeof(buf));
				memset(&vv, 0, sizeof(vv));

				if (I2C_AddRequest(&dscfram))
				{
					state++;
				};
			};

			break;

		case 3:

			if (dscfram.ready)
			{
				if (!dscfram.ack || dscfram.readedLen != dscfram.rlen) framErrorMask |= (1<<FRAM_NOACKR);

				PointerCRC p(buf);

				u32 err = 0, corrErr = 0, parErr = 0;

				bool c2 = false;

				for (byte i = 0; i < 2; i++)
				{
					Nand_ECC_Corr(p.b, sizeof(vv)+2, 256, p.b+sizeof(vv)+2, &err, &corrErr, &parErr);

					p.CRC.w = 0xFFFF;
					p.ReadArrayB(&vv, sizeof(vv));
					p.ReadW();
					p.b += 3;

					if (p.CRC.w == 0) { c2 = true; break; };
				};

				if (vv.timeStamp != timeStamp) c2 = false;

				if (c2)			framErrorMask |= 1<<FRAM_LOAD_OK;//, count += 1, framErrorMask = (framErrorMask & 0xFF)|(count<<8);
				if (!c2)		framErrorMask |= 1<<FRAM_ERROR_LOADVARS;
				if (err)		framErrorMask |= 1<<FRAM_ERROR_ECC;
				if (corrErr)	framErrorMask |= 1<<FRAM_CORR_ECC;
				if (parErr)		framErrorMask |= 1<<FRAM_ERROR_PARECC;

				state = 0;
			};
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateWindow()
{
	static byte i = 0;

	static WINDSC *d = 0;

	if (d == 0)
	{
		d = GetReadyWinDsc();

		if (d == 0) return;
	};

	u16 bsum = d->b_data[0];
	u16 msum = d->m_data[0];

	b_ts[0] += bsum;
	m_ts[0] += msum;

	u16 t; u32 s;

	for (u32 n = 1; n < d->winCount; n++)
	{
		t = d->b_data[n]; b_ts[n] += (u16)(t - bsum); bsum = t;
		t = d->m_data[n]; m_ts[n] += (u16)(t - msum); msum = t;
	};

	FreeWinDsc(d); d = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void UpdateParams()
//{
//	static byte i = 0;
//
//	#define CALL(p) case (__LINE__-S): p; break;
//
//	enum C { S = (__LINE__+3) };
//	switch(i++)
//	{
//		CALL( UpdateTemp()				);
//		CALL( SaveVars();				);
//		CALL( UpdateHV();				);
//	};
//
//	i = (i > (__LINE__-S-3)) ? 0 : i;
//
//	#undef CALL
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void UpdateMisc()
//{
//	static byte i = 0;
//
//	#define CALL(p) case (__LINE__-S): p; break;
//
//	enum C { S = (__LINE__+3) };
//	switch(i++)
//	{
//		CALL( UpdateCom(); 		);
//		CALL( UpdateHardware();	);
//		CALL( UpdateParams();	);
//	};
//
//	i = (i > (__LINE__-S-3)) ? 0 : i;
//
//	#undef CALL
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static TaskList taskList;

static void InitTaskList()
{
	static Task tsk[] =
	{
		Task(UpdateTemp,		US2CTM(100)	),
		Task(SaveVars,			US2CTM(100)	),
//		Task(UpdateCom,			US2CTM(100)	),
		Task(UpdateHardware,	US2CTM(1)	),
		Task(UpdateMan,			US2CTM(10)	),
		Task(UpdateWindow,		US2CTM(1000)),
		Task(TestFRAM,			US2CTM(100))
	};

	for (u16 i = 0; i < ArraySize(tsk); i++) taskList.Add(tsk+i);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_WHITE "main() start ...\n");

	TM32 tm;

	//__breakpoint(0);

	InitHardware();

	LoadVars();

	InitTaskList();

	comdsp.Connect(ComPort::ASYNC, 250000, 2, 1);

	u32 fc = 0;
	u16 n = 0;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "Main Loop start ...\n");

	while (1)
	{
		Pin_MainLoop_Set();

		taskList.Update(); //UpdateMisc();

		Pin_MainLoop_Clr();

		fc++;

		if (tm.Check(1000))
		{
			fps = fc; fc = 0; 

			//PrepareFire((n++)&3, 3000, 1000, 2, 5000); CM4::NVIC->STIR = EIC_0_IRQ+PWM_EXTINT;
			//PrepareFire((n++)&3, 3000, 1000, 2, 5000); HW::EVSYS->SWEVT = 1;
		};
	}; // while (1)
}
