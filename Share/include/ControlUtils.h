#pragma once

#include <math.h>

class UControlUtils
{
public:
  UControlUtils(double Mass, double Zeta[2], double Omega[2])
  {
    double M[2] = {Mass, Mass};
    double ComputedK[2];
    double ComputedB[2];
    
    for(int i = 0; i < 2; i++)
    {
      ComputedK[i] = M[i] * Omega[i] * Omega[i];
      ComputedB[i] = 2.0 * Zeta[i] * sqrt(ComputedK[i] * M[i]);
    }

    SetM(M);
    SetK(ComputedK);
    SetB(ComputedB);
  }

  UControlUtils(double M[2], double K[2], double B[2])
  {
    SetM(M);
    SetK(K);
    SetB(B);
  }

  UControlUtils(double Mass, double Spring, double Damper)
  {
    double M[2] = {Mass, Mass};
    double K[2] = {Spring, Spring};
    double B[2] = {Damper, Damper};
    
    SetM(M);
    SetK(K);
    SetB(B);
  }

  void ComputeTorque(double Q_E[2], double V_E[2], double A_E[2], double OutT[2])
  {
    for(int i = 0; i < 2; i++)
    {
      OutT[i] = M_D[i] * A_E[i] + B_D[i] * V_E[i] + K_D[i] * Q_E[i];
    }
  }

private:
  /** Set standard params (Zeta, Omega) + additional param(Mass) */
  void SetStandardParams(double Mass, double Zeta, double Omega);
  /** Set mass const manually */
  void SetM(double M[2]) { for(int i = 0; i < 2; i++) { M_D[i] = M[i]; } }
  /** Set spring const manually */
  void SetK(double K[2]) { for(int i = 0; i < 2; i++) { K_D[i] = K[i]; } }
  /** Set damper const manually */
  void SetB(double B[2]) { for(int i = 0; i < 2; i++) { B_D[i] = B[i]; } }

private:
  double M_D[2] = {0, 0};
  double K_D[2] = {0, 0};
  double B_D[2] = {0};
};