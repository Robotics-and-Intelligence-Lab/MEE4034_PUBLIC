#include <iostream>
#include "EnvironmentManager.h"

#define CONTROL_FREQ 1000 // [Hz]
#define TOTAL_SIM_TIME 3 // [s]

UEnvironmentManager Env; // env instance

/** 제어 Loop에 해당합니다. (DeltaTime: [s])*/
void Callback(double DeltaTime)
{
	// Simulation 시간 및 제어 loop 경과 시간을 표기합니다.
	printf("[%f] dt:%.6f\n", Env.GetSimulationTime(), DeltaTime);
}

/** 변경하지 마세요! */
int main(int argc, char** argv)
{
	// 1ms마다 Callback 함수를 수행합니다.
	Env.SetControlLoopFrequency(CONTROL_FREQ);
	Env.SetCallback(Callback);

	// Simulation time이 30s이 될 때까지 수행합니다.
	while (Env.GetSimulationTime() < TOTAL_SIM_TIME)
	{
		Env.StepOnce();
	}

	return 0;
}