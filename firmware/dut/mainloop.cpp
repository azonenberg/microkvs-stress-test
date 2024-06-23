/***********************************************************************************************************************
*                                                                                                                      *
* microkvs-stress-test                                                                                                 *
*                                                                                                                      *
* Copyright (c) 2024 Andrew D. Zonenberg and contributors                                                              *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#include "stressdut.h"

#include <peripheral/QuadSPI.h>

#include <microkvs/driver/STM32StorageBank.h>

GPIOPin g_led0(&GPIOB, 5, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);
GPIOPin g_led1(&GPIOB, 6, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);
GPIOPin g_led2(&GPIOB, 7, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);
GPIOPin g_led3(&GPIOA, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);

//fail is an open drain output, will be pulled low on the controller side
GPIOPin g_fail(&GPIOA, 0, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW, 0);

void InitQuadSPI();

bool KVSTestIteration(KVS* kvs);
void DumpKVS(KVS* kvs);

void App_Init()
{
	g_led0 = 1;
	g_led1 = 1;
	g_led2 = 1;
	g_led3 = 1;

	//Sectors 126 and 127 are pretty beat up on our test board, back up a bit
	//Use sectors 124 and 125 of flash for a for a 2 kB microkvs
	g_log("Initializing KVS..\n");
	static STM32StorageBank left(reinterpret_cast<uint8_t*>(0x0803e000), 0x800);
	static STM32StorageBank right(reinterpret_cast<uint8_t*>(0x0803e800), 0x800);
	InitKVS(&left, &right, 32);

	//Set up external flash quad SPI interface
	InitQuadSPI();
}

void InitQuadSPI()
{
	g_log("Initializing quad SPI...\n");
	LogIndenter li(g_log);

	//Configure the I/O pins
	static GPIOPin qspi_cs_n(&GPIOA, 2, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 10);
	static GPIOPin qspi_sck(&GPIOA, 3, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 10);
	static GPIOPin qspi_dq0(&GPIOB, 1, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 10);
	static GPIOPin qspi_dq1(&GPIOB, 0, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 10);
	static GPIOPin qspi_dq2(&GPIOA, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 10);
	static GPIOPin qspi_dq3(&GPIOA, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 10);

	//128 MB flash running at 10 MHz (AHB/8)
	static QuadSPI qspi(&QUADSPI, 128 * 1024 * 1024, 80);
	qspi.SetInstructionMode(QuadSPI::MODE_SINGLE);
	qspi.SetDoubleRateMode(false);
	//qspi.SetAddressMode(QuadSPI::MODE_QUAD, 3);
	qspi.SetAltBytesMode(QuadSPI::MODE_NONE);
	//qspi.SetDataMode(QuadSPI::MODE_QUAD);
	qspi.SetDummyCycleCount(0);
	qspi.SetDeselectTime(1);
	//qspi.SetMemoryMapMode(APBFPGAInterface::OP_APB_READ, APBFPGAInterface::OP_APB_WRITE);

	//Read the ID code
	qspi.SetAddressMode(QuadSPI::MODE_SINGLE, 0);
	qspi.SetDataMode(QuadSPI::MODE_SINGLE);
	uint8_t rdbuf[16];
	qspi.BlockingRead(0x9f, 0, rdbuf, 3);
	g_log("IDCODE = %02x %02x %02x\n",
		rdbuf[0], rdbuf[1], rdbuf[2]);

	//Try reading some data
	qspi.BlockingRead(0x03, 0, rdbuf, 16);
	g_log("Data = %02x %02x %02x %02x %02x %02x %02x %02x\n",
		rdbuf[0], rdbuf[1], rdbuf[2], rdbuf[3], rdbuf[4], rdbuf[5], rdbuf[6], rdbuf[7]);
}

bool KVSTestIteration(KVS* kvs)
{
	//Verify headers are sane
	auto freelog = kvs->GetFreeLogEntries();
	auto logcap = kvs->GetLogCapacity();
	auto freedata = kvs->GetFreeDataSpace();
	auto datacap = kvs->GetDataCapacity();
	if( (freelog > logcap) || (logcap > 32) )
	{
		g_log(Logger::ERROR, "Invalid state (KVS reports %u of %u free log entries)\n", freelog, logcap);
		return false;
	}
	if( (freedata > datacap) || (datacap > 2048) )
	{
		g_log(Logger::ERROR, "Invalid state (KVS reports %u of %u free data bytes)\n", freedata, datacap);
		return false;
	}

	const uint32_t magicA = 0xdeadf00d;
	const uint32_t magicB = 0xbaadc0de;

	//Verify we have exactly two objects
	KVSListEntry list[4];
	auto nfound = kvs->EnumObjects(list, 4);
	if(nfound == 0)
	{
		g_log(Logger::WARNING, "No objects found, writing initial test state\n");

		static const uint32_t initA[5] = { 1, 1, 1, 1, magicA };
		static const uint32_t initB[5] = { 2, 2, 2, 2, magicB };

		if(!kvs->StoreObject("A", (const uint8_t*)initA, sizeof(initA)))
		{
			g_log(Logger::ERROR, "Initial write of A failed\n");
			return false;
		}
		if(!kvs->StoreObject("B", (const uint8_t*)initB, sizeof(initB)))
		{
			g_log(Logger::ERROR, "Initial write of B failed\n");
			return false;
		}
		return true;
	}
	else if(nfound != 2)
	{
		g_log(Logger::ERROR, "Found wrong number of objects in KVS\n");
		return false;
	}

	//If we get here, we have two objects (expecting A and B)
	//Read them both
	auto ha = kvs->FindObject("A");
	auto hb = kvs->FindObject("B");
	if(!ha)
	{
		g_log(Logger::ERROR, "A not found\n");
		return false;
	}
	if(!hb)
	{
		g_log(Logger::ERROR, "B not found\n");
		return false;
	}

	//Both should be 20 bytes in length
	if(ha->m_len != 20)
	{
		g_log(Logger::ERROR, "A has bad length %d (expected 16)\n", ha->m_len);
		return false;
	}
	if(hb->m_len != 20)
	{
		g_log(Logger::ERROR, "B has bad length %d (expected 16)\n", hb->m_len);
		return false;
	}

	//Each object should consist of four copies of the same 32-bit value, then the magic number
	auto pa = (uint32_t*)kvs->MapObject(ha);
	auto pb = (uint32_t*)kvs->MapObject(hb);
	if( (pa[0] != pa[1]) || (pa[0] != pa[2]) || (pa[0] != pa[3]) )
	{
		g_log(Logger::ERROR, "A has bad content %08x %08x %08x %08x %08x\n", pa[0], pa[1], pa[2], pa[3], pa[4]);
		return false;
	}
	if( (pb[0] != pb[1]) || (pb[0] != pb[2]) || (pb[0] != pb[3]) )
	{
		g_log(Logger::ERROR, "B has bad content %08x %08x %08x %08x %08x\n", pb[0], pb[1], pb[2], pb[3], pb[4]);
		return false;
	}
	if(pa[4] != magicA)
	{
		g_log(Logger::ERROR, "A has bad magic %08x %08x %08x %08x %08x\n", pa[0], pa[1], pa[2], pa[3], pa[4]);
		return false;
	}
	if(pb[4] != magicB)
	{
		g_log(Logger::ERROR, "B has bad magic %08x %08x %08x %08x %08x\n", pb[0], pb[1], pb[2], pb[3], pb[4]);
		return false;
	}

	//A and B should differ by exactly one
	if( (pa[0] != (pb[0] + 1)) && (pa[0] != (pb[0] - 1)) )
	{
		g_log(Logger::ERROR, "A and B inconsistent (%08x %08x)\n", pa[0], pb[0]);
		return false;
	}

	//Write new version of one object
	if(pa < pb)
	{
		uint32_t nval = pb[0] + 1;
		uint32_t nextA[5] = { nval, nval, nval, nval, magicA };
		if(!kvs->StoreObject("A", (const uint8_t*)nextA, sizeof(nextA)))
		{
			g_log(Logger::ERROR, "Write of A failed\n");
			return false;
		}
	}
	else
	{
		uint32_t nval = pa[0] + 1;
		uint32_t nextB[5] = { nval, nval, nval, nval, magicB };
		if(!kvs->StoreObject("B", (const uint8_t*)nextB, sizeof(nextB)))
		{
			g_log(Logger::ERROR, "Write of B failed\n");
			return false;
		}
	}

	return true;
}

/**
	@brief Prints the contents of the KVS
 */
void DumpKVS(KVS* kvs)
{
	KVSListEntry list[4];
	auto nfound = kvs->EnumObjects(list, 4);
	for(size_t i=0; i<nfound; i++)
	{
		g_uart.Printf("    %s (%d bytes, %02d revs): ",
			list[i].key, list[i].size, list[i].revs);

		auto hobject = kvs->FindObject(list[i].key);
		if(!hobject)
		{
			g_uart.Printf("[fail to find object]\n");
			continue;
		}
		auto ptr = kvs->MapObject(hobject);
		if(!ptr)
		{
			g_uart.Printf("[fail to map object]\n");
			continue;
		}
		for(size_t j=0; j<list[i].size; j++)
			g_uart.Printf("%02x ", ptr[j]);

		g_uart.Printf("\n");
	}
}

/**
	@brief Set up the microkvs key-value store for persisting our configuration
 */
/*
void InitExternalKVS(StorageBank* left, StorageBank* right, uint32_t logsize)
{
	g_log("Initializing microkvs key-value store\n");
	static KVS kvs(left, right, logsize);
	g_kvs = &kvs;

	LogIndenter li(g_log);
	g_log("Block size:  %d bytes\n", kvs.GetBlockSize());
	g_log("Log:         %d / %d slots free\n", (int)kvs.GetFreeLogEntries(), (int)kvs.GetLogCapacity());
	g_log("Data:        %d / %d bytes free\n", (int)kvs.GetFreeDataSpace(), (int)kvs.GetDataCapacity());
	g_log("Active bank: %s (rev %zu)\n",
		kvs.IsLeftBankActive() ? "left" : "right",
		kvs.GetBankHeaderVersion() );
}*/

void BSP_MainLoopIteration()
{
	static unsigned int i=0;

	bool ok = KVSTestIteration(g_kvs);

	i++;
	if( ((i % 50) == 0) || !ok)
	{
		g_uart.Printf("%u\n", i);
		DumpKVS(g_kvs);
	}

	if(!ok)
	{
		//assert fail signal
		g_fail.SetMode(GPIOPin::MODE_OUTPUT, 0, false);
		g_fail = 1;

		while(1)
		{}
	}
}
