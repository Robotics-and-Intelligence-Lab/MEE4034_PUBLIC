#include <iostream>
#include <math.h>
#include <time.h>

#define CONTROL_FREQ 1000 // [Hz]
// 1ms = 1e-3[s] = X * 1/(10 * 1e6)[s]
// X = 1e-3 * (10 * 1e6)
#define TOTAL_SIM_TIME 30 // [s]
#define COUNT2SEC(X) X / 1e7 // 1Cnt = 1/(10*1e6) [s]

#ifdef __linux__
#include "EposManager.h"
timespec InitTime;
timespec TickTime;
timespec TockTime;
#elif _WIN32
#include <Windows.h>
LARGE_INTEGER InitTime;
LARGE_INTEGER TickTime;
LARGE_INTEGER TockTime;
#endif

int64_t TotalCnt = 0; // 1/(10*1e6) [s]
int64_t DeltaCnt = 0; // 1/(10*1e6) [s]

inline void StartTimer();
inline void UpdateTime();
inline void Tock();

/** 제어 Loop에 해당합니다. (TotalTime, DeltaTime: [s])*/
inline void Callback(double TotalTime, double DeltaTime)
{
	// 총 실행 시간 및 제어 loop 경과 시간을 표기합니다.
	printf("[%f] dt:%.6f\n", TotalTime, DeltaTime);
}

/** 변경하지 마세요! */
int main(int argc, char **argv)
{	
	int64_t Interval = static_cast<int64_t>(1e7 / CONTROL_FREQ);
	StartTimer();

	while (COUNT2SEC(TotalCnt) < TOTAL_SIM_TIME)
	{
		UpdateTime();
		if (DeltaCnt >= Interval)
		{
			Tock();
			Callback(COUNT2SEC(TotalCnt), COUNT2SEC(DeltaCnt));
		}
	}
}

inline void StartTimer()
{
#ifdef __linux__
	clock_gettime(CLOCK_MONOTONIC, &InitTime);
#elif _WIN32
	QueryPerformanceCounter(&InitTime);
#endif
	TickTime = InitTime;
	TockTime = InitTime;
}

inline void UpdateTime()
{
#ifdef __linux__
	clock_gettime(CLOCK_MONOTONIC, &TickTime);

	// Win10 카운터 단위로 일치
	TotalCnt = (TickTime.tv_sec - InitTime.tv_sec) * 1e7; // change unit: [s] to 1/10M [s] = 100ns
	TotalCnt += (TickTime.tv_nsec - InitTime.tv_nsec) * 0.01;
	DeltaCnt = (TickTime.tv_sec - TockTime.tv_sec) * 1e7; // change unit: [s] to 1/10M [s] = 100ns
	DeltaCnt += (TickTime.tv_nsec - TockTime.tv_nsec) * 0.01;
#elif _WIN32
	// Win10 이상의 시스템의 경우 CounterFrequency가 10MHz로 고정
	// static LARGE_INTEGER CounterFrequency;
	// QueryPerformanceFrequency(&CounterFrequency);

	QueryPerformanceCounter(&TickTime);
	TotalCnt = TickTime.QuadPart - InitTime.QuadPart;
	DeltaCnt = TickTime.QuadPart - TockTime.QuadPart;
#endif
}

inline void Tock()
{
	TockTime = TickTime;
}