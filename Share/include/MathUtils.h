#pragma once

#ifdef _WIN32
    #define _USE_MATH_DEFINES
#endif

#include <iostream>
#include <math.h>

#define RAD2DEG(X) X * 180.0 / M_PI
#define DEG2RAD(X) X * M_PI / 180.0

class UMathUtils
{
public:
    static bool InvMat2x2(double In[2][2], double Out[2][2]);
    static void MultMatVec2D(double Mat[2][2], double Vec[2], double Out[2]);

    static void MultMatVec(double Mat[], double Vec[], double Out[], const int Rows, const int Cols);

    static void Vec2Add(double In1[2], double In2[2], double Out[2]);
    static void Vec2Sub(double In1[2], double In2[2], double Out[2]);
    static void Vec2Mult(double In[2], double K, double Out[2]);

    static void VecAdd(double In1[], double In2[], double Out[], const int Size);
    static void VecSub(double In1[], double In2[], double Out[], const int Size);
    static void VecMult(double In[], double K, double Out[], const int Size);

    static void PrintMatrix2D(double M[2][2], std::string Name);
    static void PrintVector2D(double Vec[2], std::string Name);

    static void PrintMatrix(double M[], const int Rows, const int Cols, std::string Name);
    static void PrintVector(double Vec[], const int Size, std::string Name);

    template <typename T> T Constain(T Value, T Min, T Max) { return std::max(Min, std::min(Value, Max)); }
};