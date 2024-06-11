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

GPIOPin g_led0(&GPIOB, 5, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);
GPIOPin g_led1(&GPIOB, 6, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);
GPIOPin g_led2(&GPIOB, 7, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);
GPIOPin g_led3(&GPIOA, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0);

void App_Init()
{
	g_led0 = 1;
	g_led1 = 1;
	g_led2 = 1;
	g_led3 = 1;
}

void BSP_MainLoopIteration()
{
	const int logTimerMax = 60000;
	static uint32_t next1HzTick = 0;

	//Check for overflows on our log message timer
	if(g_log.UpdateOffset(logTimerMax) && (next1HzTick >= logTimerMax) )
		next1HzTick -= logTimerMax;

	//1 Hz timer event
	if(g_logTimer.GetCount() >= next1HzTick)
	{
		next1HzTick = g_logTimer.GetCount() + 10000;

	}
}