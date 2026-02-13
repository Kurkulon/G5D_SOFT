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
static u16 manTrmData[50];
static u16 manTrmBaud = 0;

static const u16 manReqWord = 0x7000;
static const u16 manReqMask = 0xFF00;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 txbuf[128 + 512 + 16];

static u16 reqFireVoltage = 0;
static u16 curFireVoltage = 300;


static u16 verDevice = VERSION;
static bool numDevValid = false;

i16 temp = 0;

u32 m_ts[WINDOW_SIZE];
u32 b_ts[WINDOW_SIZE];

static byte svCount = 0;

//static Rsp72 *curRsp72 = 0;

u16 fireAmp = 0;
u16 fireFreq = 3000;

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
	*(data++)	= mv.bLevel;							//	5. Уровень дискриминации МЗ(у.е), 					
	*(data++)	= mv.mLevel;							//	6. Уровень дискриминации БЗ(у.е),					

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

	manTrmData[0] = data[0];//(manReqWord & manReqMask) | 0x20;
	
	data = manTrmData;
	data++;

	u16 wc;

	*(data++)	= GetFireCount();				//2. Количество вспышек(ushort)						
	*(data++)	= GetGenWorkTime();				//3. Наработка генератора(мин)(ushort)								
	*(data++)	= temp;							//4. Температура в приборе(0.1 гр)(short)										
	*(data++)	= wc = mv.winCount;				//5. Количество временных окон(шт)								
	*(data++)	= mv.winTime;					//6. Длительность временного окна(мкс)								

	u16 n = 6;

	for (u16 i = 0; i < wc; i++)
	{
		u32 tm = m_ts[i]; m_ts[i] = 0;
		u32 tb = b_ts[i]; b_ts[i] = 0;

		data[0]		= MIN(tm, 0xFFFF);			//7..x. Спектр МЗ(ushort)
		data[wc]	= MIN(tb, 0xFFFF);			//x..y. Спектр БЗ(ushort)

		data++;
	};

	data += wc;

	mtb->data1 = manTrmData;
	mtb->len1 = data - start;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

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
		case 0x01:	SetGenFreq(							mv.genFreq	= LIM(data[2], 4, 50)	);	break;	//	0x1 - Частота генератора(4..50 Гц), 
		case 0x02:	SetWindowCount(						mv.winCount	= LIM(data[2], 2, 1024)	);	break;	//	0x2 - Количество временных окон(2..1024 шт), 
		case 0x03:	SetWindowTime(						mv.winTime	= LIM(data[2], 2, 2048)	);	break;	//	0x3 - Длительность временного окна(2..2048 мкс), 
		case 0x04:	AD5312_Set(AD5312_CHANNEL_LEVEL_M,	mv.mLevel	= MIN(data[2], 0x3FF)	);	break;	//	0x4 - Уровень дискриминации МЗ(у.е), 
		case 0x05:	AD5312_Set(AD5312_CHANNEL_LEVEL_B,	mv.bLevel	= MIN(data[2], 0x3FF)	);	break;	//	0x5 - Уровень дискриминации БЗ(у.е),

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
		case 0x8: 	r = RequestMan_80(buf, len, mtb); break;
		case 0x9:	r = RequestMan_90(buf, len, mtb); break;
		case 0xA:	r = RequestMan_A0(buf, len, mtb); break;
		case 0xF:	r = RequestMan_F0(buf, len, mtb); break;
	};

	if (r) { mtb->baud = manTrmBaud; };

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMan()
{
	static MTB mtb;
	static MRB mrb;

	static byte i = 0;

	static CTM32 tm;

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

				if (mrb.OK && mrb.len > 0 && ((manRcvData[0] & manReqMask) == manReqWord && RequestMan(manRcvData, mrb.len, &mtb)))
				{
					i++;
				}
				else
				{
					i = 0;
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

	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef bool (*REQF)(byte *req, u16 len, ComPort::WriteBuffer *wb);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef kdjfhgkdfhgjdfg

static bool Request01(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	// Запуск импульса излучателя

	ReqTrm01::Req &req = *((ReqTrm01::Req*)data);
	
	PrepareFire(req.n, req.fireFreq, req.fireAmp, req.fireCount, req.fireDuty);

	wb->data = 0;
	wb->len = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request02(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	static RspTrm02 rsp;

	ReqTrm02::Req &req = *((ReqTrm02::Req*)data);

	reqFireVoltage = req.reqHV;

	if (req.numDevValid) mv.numDevice = req.numDev, numDevValid = true;
	
	if (req.saveParams) SaveMainParams();

	rsp.func		= req.func;
	rsp.numDevValid = numDevValid;
	rsp.numdev		= mv.numDevice;
	rsp.verdev		= VERSION;
	rsp.hv			= curFireVoltage;
	rsp.temp		= temp;
	rsp.crc			= GetCRC16(&rsp, sizeof(rsp)-sizeof(rsp.crc));

	wb->data	= &rsp;
	wb->len		= sizeof(rsp);

	return false;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request03(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	static u16 buf[2];

//	ReqTrm03::Req &req = *((ReqTrm03::Req*)data);

	curRsp72 = GetReadyRsp72();

	if (curRsp72 == 0)
	{
		buf[0] = TRM_RSP03_RW;
		buf[1] = GetCRC16(buf, 2);

		wb->data = buf;
		wb->len = sizeof(buf);
	}
	else
	{
		RspTrm03 &rsp = curRsp72->h;

		rsp.rw = TRM_RSP03_RW;

		i16 max = 0;

		for (u16 i = 0; i < rsp.sl; i++)
		{
			i16 t = rsp.data[i]-0x800;
			t *= 2;
			rsp.data[i] = t;

			if (t < 0) t = -t;
			if (t > max) max = t;
		};

		wb->data = &rsp;
		wb->len = sizeof(rsp) - sizeof(rsp.data) - sizeof(rsp.crc) + rsp.sl*2;

		rsp.amp = max;
		rsp.data[rsp.sl] = GetCRC16(wb->data, wb->len);

		wb->len += 2;
	};

	return false;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static REQF listReq[3] = { Request01, Request02, Request03 };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool UpdateRequest(ComPort::WriteBuffer *wb, ComPort::ReadBuffer *rb)
{
	//static Req nulReq;

	static const byte fl[3] = { sizeof(ReqTrm01::r[0])-1, sizeof(ReqTrm02::r[0])-1, sizeof(ReqTrm03::r[0])-1 };

	if (rb == 0 || rb->len < 4) return false;

	bool result = false;

	u16 rlen = rb->len;

	byte *p = (byte*)rb->data;

	while(rlen > 3)
	{
		byte len = p[0];
		byte func = p[1]-1;

		if (func < 4 && len == fl[func] && len < rlen && GetCRC16(p+1, len) == 0)
		{
			//Req *req = (Req*)p;

			result = listReq[func](p, len+1, wb);

			break;
		}
		else
		{
			p += 1;
			rlen -= 1;
		};
	};

	return result;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

			rb.data = buf;
			rb.maxLen = sizeof(buf);
			comdsp.Read(&rb, MS2COM(5000), US2COM(100));
			i++;

			break;

		case 1:

			if (!comdsp.Update())
			{
				if (rb.recieved && rb.len >= 6)
				{
					ctm.Reset();

					if (UpdateRequest(&wb, &rb))
					{
						i += 2;
					}
					else if (wb.data != 0 && wb.len != 0)
					{
						comdsp.Write(&wb);

						i++;
					}
					else
					{
						i = 0;
					};
				}
				else
				{
					i = 0;
				};
			};

			break;

		case 2:

			if (!comdsp.Update())
			{
				if (curRsp72 != 0) FreeRsp72(curRsp72), curRsp72 = 0;

				i = 0;
			};

			break;

		case 3:

			if (IsFireOK() || ctm.Check(MS2CTM(50)))
			{
				DisableFire();

				i = 0;
			};

			break;
	};
}
#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateTemp()
{
	static byte i = 0;

	static DSCI2C dsc, dsc2;

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

				dsc.adr = 0x49;
				dsc.wdata = buf;
				dsc.wlen = 1;
				dsc.rdata = &rbuf;
				dsc.rlen = 2;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				if (I2C_AddRequest(&dsc))
				{
					i++;
				};
			};

			break;

		case 1:

			if (dsc.ready)
			{
				if (dsc.ack && dsc.readedLen == dsc.rlen)
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
	
	byte buf[sizeof(mv)*2+4];

	bool c2 = false;

	bool loadVarsOk = false;

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

	PointerCRC p(buf);

	for (byte i = 0; i < 2; i++)
	{
		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&mv, sizeof(mv));
		p.ReadW();

		if (p.CRC.w == 0) { c2 = true; break; };
	};

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
	static byte buf[sizeof(mv) * 2 + 8];

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
				p.CRC.w = 0xFFFF;
				p.WriteArrayB(&mv, sizeof(mv));
				p.WriteW(p.CRC.w);
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
		Task(UpdateHV,			US2CTM(100)	),
//		Task(UpdateCom,			US2CTM(1)	),
		Task(UpdateHardware,	US2CTM(1)	),
		Task(UpdateMan,			US2CTM(100)	)
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

	//comdsp.Connect(ComPort::ASYNC, TRM_COM_BAUDRATE, TRM_COM_PARITY, TRM_COM_STOPBITS);

	u32 fc = 0;
	u16 n = 0;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "Main Loop start ...\n");

	while (1)
	{
		Pin_MainLoop_Set();

		taskList.Update(); //UpdateMisc();

		Pin_MainLoop_Clr();

		fc++;

		if (tm.Check(250))
		{
			fps = fc; fc = 0; 

			//PrepareFire((n++)&3, 3000, 1000, 2, 5000); CM4::NVIC->STIR = EIC_0_IRQ+PWM_EXTINT;
			//PrepareFire((n++)&3, 3000, 1000, 2, 5000); HW::EVSYS->SWEVT = 1;
		};
	}; // while (1)
}
