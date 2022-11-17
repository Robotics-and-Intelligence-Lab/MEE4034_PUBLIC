#include "MathUtils.h"

bool UMathUtils::InvMat2x2(double In[2][2], double Out[2][2])
{
    double Det = In[0][0] * In[1][1] - In[1][0] * In[0][1]; // det(M) = (a*d - b*c)
    Out[0][0] = In[1][1] / Det;
    Out[1][1] = In[0][0] / Det;

    Out[0][1] = -In[0][1] / Det;
    Out[1][0] = -In[1][0] / Det;

    if (abs(Det) < 1e-10)
    {
        std::cerr << "det(M) = " << Det << std::endl;
        return false;
    }

    return true;
}

void UMathUtils::MultMatVec2D(double Mat[2][2], double Vec[2], double Out[2])
{
    UMathUtils::MultMatVec(&Mat[0][0], Vec, Out, 2, 2);   
}

void UMathUtils::MultMatVec(double Mat[], double Vec[], double Out[], const int Rows, const int Cols)
{
    // double Temp[Cols];
    double* Temp = new double[Cols];

    VecMult(Vec, 1.0, Temp, Cols);

    for (int i = 0; i < Rows; i++)
    {
        Out[i] = 0;
        for (int j = 0; j < Cols; j++)
        {
            // Out[i] += Mat[i * Cols + j] * Vec[j];
            Out[i] += Mat[i * Cols + j] * Temp[j];
        }
    }

    delete[] Temp;
}

void UMathUtils::Vec2Add(double In1[2], double In2[2], double Out[2])
{
    UMathUtils::VecAdd(In1, In2, Out, 2);
}

void UMathUtils::Vec2Sub(double In1[2], double In2[2], double Out[2])
{
    UMathUtils::VecSub(In1, In2, Out, 2);
}

void UMathUtils::Vec2Mult(double In[2], double K, double Out[2])
{
    UMathUtils::VecMult(In, K, Out, 2);
}

void UMathUtils::VecAdd(double In1[], double In2[], double Out[], const int Size)
{
    for(int i = 0; i < Size; i++)
    {
        Out[i] = In1[i] + In2[i];
    }
}

void UMathUtils::VecSub(double In1[], double In2[], double Out[], const int Size)
{
    // double Temp[Size];
    double* Temp = new double[Size];

    VecMult(In2, -1.0, Temp, Size);
    VecAdd(In1, Temp, Out, Size);

    delete[] Temp;
}

void UMathUtils::VecMult(double In[], double K, double Out[], const int Size)
{
    for(int i = 0; i < Size; i++)
    {
        Out[i] = K * In[i];
    }
}

void UMathUtils::PrintMatrix2D(double M[2][2], std::string Name)
{
    UMathUtils::PrintMatrix(&M[0][0], 2, 2, Name);
}

void UMathUtils::PrintVector2D(double Vec[2], std::string Name)
{
    UMathUtils::PrintVector(Vec, 2, Name);
}

void UMathUtils::PrintMatrix(double M[], const int Rows, const int Cols, std::string Name)
{
  std::cout << Name << "(" << Rows << "x" << Cols << ") = "; 
  if(Rows != 1) { std::cout << std::endl; }
  std::cout << "[ ";
  for(int i = 0; i < Rows; i++)
  {
    for(int j = 0; j < Cols; j++)
    {
      std::cout << M[i * Cols + j] << " ";
    }
    if(i == (Rows-1)) { std::cout << "]"; }
    else { std::cout << ";"; }
    std::cout << std::endl;
  }
}

void UMathUtils::PrintVector(double Vec[], const int Size, std::string Name)
{
    UMathUtils::PrintMatrix(Vec, 1, Size, Name);
}