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

#include "stressctrl.h"
#include <stdlib.h>

GPIOPin g_led0(&GPIOB, 5, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);
GPIOPin g_led1(&GPIOB, 6, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);
GPIOPin g_led2(&GPIOB, 7, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);

GPIOPin g_dutPwr(&GPIOA, 0, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);
GPIOPin g_dutRst(&GPIOA, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);

GPIOPin g_failDetect(&GPIOA, 8, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW, 0);

void RNG_Init();
uint32_t GenerateRandom();

uint32_t g_rngseed = 1;

void App_Init()
{
	RNG_Init();

	g_led0 = 1;
	g_led1 = 1;
	g_led2 = 1;

	g_dutPwr = 1;
	g_dutRst = 1;

	g_failDetect.SetPullMode(GPIOPin::PULL_DOWN);
}

void RNG_Init()
{
	/*//Turn it on
	RCCHelper::Enable(&RNG);

	//Turn on the RNG and disable clock error detection for some reason
	//RNGCLK needs to be at least 5 MHz (80 MHz APB/16)
	RNG.CR = 0;
	RNG.CR |= RNG_CED;
	RNG.CR |= RNG_EN;
	*/
	g_rngseed = 1;
}

/**
	@brief Return a 32-bit random number
 */
uint32_t GenerateRandom()
{
	/*
	//Block until data is ready to read
	while( (RNG.SR & RNG_DRDY) == 0)
	{}

	//Get the current data word
	//return RNG.DR;

	uint32_t tmp = RNG.DR;
	g_log("rng %08x\n", tmp);
	return tmp;
	*/

	g_rngseed = (g_rngseed * 214013 + 2531011) & 0x7fffffff;

	return (g_rngseed >> 16) & 0x3fff;
}

void BSP_MainLoopIteration()
{
	const int logTimerMax = 60000;
	static uint32_t next1HzTick = 0;
	static uint32_t nextPowerFailOrReset = 5000;

	//Check for overflows on our log message timer
	if(g_log.UpdateOffset(logTimerMax) )
	{
		if(next1HzTick >= logTimerMax)
			next1HzTick -= logTimerMax;

		if(nextPowerFailOrReset >= logTimerMax)
			nextPowerFailOrReset -= logTimerMax;
	}

	//1 Hz timer event
	if(g_logTimer.GetCount() >= next1HzTick)
		next1HzTick = g_logTimer.GetCount() + 10000;

	//If the DUT has failed, stop
	if(g_failDetect)
	{
		g_log(Logger::ERROR, "DUT reported a failure, freezing\n");
		while(1)
		{}
	}

	//Reboot the DUT
	if(g_logTimer.GetCount() >= nextPowerFailOrReset)
	{
		//randomly reset or power down
		uint32_t delta = 5000 + (GenerateRandom() % 5000);
		if(GenerateRandom() & 1)
		{
			g_log("Resetting DUT, next in %d.%d ms\n", delta/10, delta%10);
			g_dutRst = 0;
		}
		else
		{
			g_log("Power cycling DUT, next in %d.%d ms\n", delta/10, delta%10);
			g_dutPwr = 0;
		}

		//wait 50ms to ensure it's reset
		g_logTimer.Sleep(500);

		//turn it back on
		g_dutPwr = 1;
		g_dutRst = 1;

		//reboot randomly after 500 - 1000 ms
		nextPowerFailOrReset = g_logTimer.GetCount() + delta;
	}
}
