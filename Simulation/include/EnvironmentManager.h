#ifndef ENVIRONMENT_MANAGER_H
#define ENVIRONMENT_MANAGER_H

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <string>
#include <functional>

#define DESIRED_FPS 60.0

class UEnvironmentManager
{
public:
  UEnvironmentManager();
  ~UEnvironmentManager();

  void SetCallback(std::function<void(double)> Func); 
  
  void GetJointPosition(double Q[2]);
  void GetJointVelocity(double V[2]);
  void GetJointAcceleration(double Acc[2]);

  void SetJointTorque(double T[2]);

  void StepOnce();
  void UpdateScreen();
  double GetSimulationTime();
  
  /** Set frequency of control loop func */
  void SetControlLoopFrequency(double Freq);

  void GetFullJointAcc(double Acc[5])
  {
    for(int i = 0; i < 5; i++)
    {
      Acc[i] = SimData->qacc[i];
    }
  }

  void GetMassMatrix(double M[5][5])
  {
    const int nv = 5;
    double DenseMat[nv*nv] = {0};
    mj_fullM(SimModel, DenseMat, SimData->qM);

    for(int i = 0; i < nv; i++)
    {
      for(int j = 0; j < nv; j++)
      {
        M[i][j] = DenseMat[i * nv + j];
      }
    }
  }
private:
  void Initialize(std::string ModelPath);
  void Callback(double DeltaTime) { ControlCallback(DeltaTime); }

private:
  mjModel* SimModel = nullptr;
  mjData* SimData = nullptr;
  mjvCamera SimCam;
  mjvOption SimOption;
  mjvScene SimScene;
  mjrContext SimContext;

  char ErrBuffer[1000];

  GLFWwindow* Window;

  double CallbackInterval; // [s]

  double RequestedTorque[2];

  std::function<void(double)> ControlCallback;
  bool bCallbackAttached = false;
};

#endif