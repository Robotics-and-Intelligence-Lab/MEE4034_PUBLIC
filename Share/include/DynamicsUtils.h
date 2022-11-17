#pragma once

#include <math.h>

#define G_CONST 9.81
#define R_LOAD 0.022
#define M_LOAD 0.1
#define M1 0.4009
#define M2_LINK 0.0844
#define I1 739.265e-6
#define I2_AXIS 108.41e-6
#define I2_LINK 218.052e-6

#define L0 0.15
#define L1 0.0185
#define L2_LINK 0.032

class UDynamicsUtils
{
public:
    UDynamicsUtils();

    void UpdateDynamics(double Q[2], double V[2]);

    void UpdateM(double Q[2]);
    void UpdateC(double Q[2], double V[2]);
    void UpdateG(double Q[2]);

    void GetM(double Out[2][2]);
    void GetC(double Out[2]);
    void GetG(double Out[2]);

    void EsimateAcc(double T[2], double Acc[2]);
    void ComputeTorque(double Acc[2], double T[2]);

    const double M2;
    const double I_LOAD;
    const double L2;
    const double I2_LINK_LOAD;

    double M[2][2];
    double C[2];
    double G[2];
};

// FFrictionParams Params;
// Params.Kc[0] = 0.032103;
// Params.Kv[0] = 19.2303e-5;
// Params.Kc[1] = 0.0459;
// Params.Kv[1] = 9.2871e-4;



// const double GainP[2] = {1.0, 1.0};
// const double GainD[2] = {0.08, 0.024};
// double DesiredQ[2] = {DEG2RAD(0), DEG2RAD(0)};
// double DesiredV[2] = {0.0, 0.0};