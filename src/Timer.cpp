/**
 * @file Timer.cpp
 * @author Yuki Suga (ysuga.net), Yuki Nakagawa (RT Corp.), Motomasa Tanaka (Mayekawa MFC Co., Ltd.)
 * @copyright RT Corp. 2012 All rights reserved.
 */

#ifdef WIN32
#include <windows.h>
#include <mmsystem.h>
#endif

#include <string.h>

#include "Timer.h"

using namespace pcwrapper;


Timer::Timer(void)
{
#ifdef WIN32
	memset(&m_Frequency, 0, sizeof(LARGE_INTEGER));
	memset(&m_Before, 0, sizeof(LARGE_INTEGER));
	memset(&m_After, 0, sizeof(LARGE_INTEGER));
	QueryPerformanceFrequency(&m_Frequency);
#endif
}

Timer::~Timer(void)
{
}


void Timer::tick(void)
{
#ifdef WIN32
	::QueryPerformanceCounter(&m_Before);	
#else

#endif
}

void Timer::tack(TimeSpec* pCurrentTime)
{
#ifdef WIN32
	QueryPerformanceCounter(&m_After);
	DWORD dwTime = (DWORD)((m_After.QuadPart - m_Before.QuadPart) * 1000000 / m_Frequency.QuadPart);
	pCurrentTime->sec =  dwTime / 1000000;
	pCurrentTime->usec = dwTime % 1000000;
#else

#endif
}