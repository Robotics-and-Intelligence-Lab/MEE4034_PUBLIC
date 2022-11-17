#ifndef EPOS_MANAGER_H
#define EPOS_MANAGER_H

#include <iostream>

class UEposManager
{
  typedef void *Handle;

public:
  UEposManager();
  ~UEposManager();

  /** Open devices & Change OP mode to Current Mode (CM) */
  void Init(bool bReset=false);
  /** Switch controller's state to Enabled */
  void EnableController();
  /** Disable controller */
  void DisableController();

  /** Clear controller's error */
  void ClearError();
  /** Reset the all devices (to change motor 0 position) */
  void ResetAll();

  /** Set desired torque in Nm */
  void SetTorque(double T1, double T2);
  /** Get current position in rads */
  void GetPosition(double &Q1, double &Q2);
  /** Get current velocity in rads/s */
  double GetVelocity(double &V1, double &V2);

private:
  template <typename T>
  inline void ApplyClipping(T &Value, T Min, T Max);

  /** Open the 1st controller device */
  void OpenMainDev();
  /** Open the 2nd controller device */
  void OpenSubDev();
  /** Close the 1st controller device */
  void CloseMainDev();
  /** Close the 2nd controller device */
  void CloseSubDev();

  /** Set specific node's desired current [mA] */
  void SetDesiredCurrent(int Node, int DesiredValue);

private:
  double TorqueConst[2]; // [Nm]/A
  double MaxCurrent[2]; // nominal current [A]
  double HomePosition[2]; // Home position in rads
  uint16_t GearRatio[2];

  const int PulsePerRev; // 2048 * 4

  Handle MainDev;
  Handle SubDev;

  uint32_t ErrorCode;

  bool bOpened[2];
};

#endif