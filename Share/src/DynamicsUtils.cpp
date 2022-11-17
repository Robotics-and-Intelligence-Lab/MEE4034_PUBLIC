#include "DynamicsUtils.h"
#include "MathUtils.h"

UDynamicsUtils::UDynamicsUtils()
    : M2(M2_LINK + M_LOAD),
      I_LOAD(1 / 2.0 * M_LOAD * R_LOAD * R_LOAD),
      L2((L0 * M_LOAD + L2_LINK * M2_LINK) / (M2_LINK + M_LOAD)),
      I2_LINK_LOAD(I2_LINK + I_LOAD + (M_LOAD + M2_LINK) * (L0 - L2) * (L0 - L2))
{
    M[0][0] = I1 + (M1 * L1 * L1 + M2 * L0 * L0);
    M[1][1] = I2_LINK_LOAD + I2_AXIS + (M2 * L2 * L2);
}

void UDynamicsUtils::UpdateDynamics(double Q[2], double V[2])
{
    UpdateM(Q);
    UpdateC(Q, V);
    UpdateG(Q);
}

void UDynamicsUtils::UpdateM(double Q[2])
{
    const static double ConstTerm = M2 * L0 * L2; // compute once
    M[0][1] = ConstTerm * cos(Q[1] - Q[0]);
    M[1][0] = M[0][1];
}

void UDynamicsUtils::UpdateC(double Q[2], double V[2])
{
    const static double ConstTerm = M2 * L0 * L2;
    double Common = ConstTerm * sin(Q[1] - Q[0]);
    C[0] = Common * (-V[1] * V[1]);
    C[1] = Common * (V[0] * V[0]);
}

void UDynamicsUtils::UpdateG(double Q[2])
{
    // G[0] = G_CONST * (M1 * L1) * cos(Q[0]);
    G[0] = G_CONST * (M1 * L1 + M2 * L0) * cos(Q[0]);
    G[1] = G_CONST * M2 * L2 * cos(Q[1]);
}

void UDynamicsUtils::GetM(double Out[2][2])
{
    for(int i = 0; i < 2*2; i++) { Out[0][i] = M[0][i]; }
}

void UDynamicsUtils::GetC(double Out[2])
{
    for(int i = 0; i < 2; i++) { Out[i] = C[i]; }
}

void UDynamicsUtils::GetG(double Out[2])
{
    for(int i = 0; i < 2; i++) { Out[i] = G[i]; }
}

void UDynamicsUtils::EsimateAcc(double T[2], double Acc[2])
{
    double InvM[2][2];
    double SumT[2];
    
    UMathUtils::InvMat2x2(M, InvM);

    UMathUtils::VecSub(T, C, SumT, 2); // T - C
    UMathUtils::VecSub(SumT, G, SumT, 2); // T - C - G

    UMathUtils::MultMatVec2D(InvM, SumT, Acc);
}

void UDynamicsUtils::ComputeTorque(double Acc[2], double T[2])
{
    UMathUtils::MultMatVec2D(M, Acc, T); // M*qddot
    UMathUtils::VecAdd(T, C, T, 2); // M*qddot + c
    UMathUtils::VecAdd(T, G, T, 2); // M*qddot + c + g
}
