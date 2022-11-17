#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#define INITIAL_HIST_SIZE 5000

class USimpleLogger
{
public:
  /** 데이터를 저장할 파일 이름(확장자 제외)을 지정하여 Logger instance를 생성합니다. 예) pd_control */
  USimpleLogger(std::string FileName) : LogFile(nullptr)
  {
    FilePath = FileName + ".txt";
    Hist.reserve(INITIAL_HIST_SIZE);
  }

  ~USimpleLogger()
  {
    End();
  }

  /** 지정된 파일 경로를 기반으로 File handle을 등록합니다. 성공 유무를 반환합니다. */
  bool Init()
  {
    LogFile = std::fopen(FilePath.c_str(), "w");
    if(!LogFile) 
    { 
      std::cerr << "Can't open file. path: " << FilePath << std::endl;
      return false;
    }

    std::cout << "File path: " << FilePath << std::endl;
    
    return true;    
  }

  /** 수집한 Data를 File에 기록하고 File handle을 반납합니다. */
  void End()
  {
    if(LogFile == nullptr) { return; }

    for(size_t i = 0; i < Hist.size(); i++)
    {
      WriteFile(Hist[i]);
      printf("[Logger]progress: %ld/%ld\n", (i+1), Hist.size());
    }    

    printf("[Logger]data successfully saved. file path: %s\n", FilePath.c_str());
    std::fclose(LogFile);
    LogFile = nullptr;
  }

  /** Data를 추가합니다. */
  void Append(double Time, double Q[2])
  {
    std::vector<double> Item = {Time, Q[0], Q[1]};
    Hist.push_back(Item);
  }

  /** Data를 추가합니다. */
  void Append(double Time, double Q[2], double Error[2])
  {
    std::vector<double> Item = {Time, Q[0], Q[1], Error[0], Error[1]};
    Hist.push_back(Item);
  }

  /** Data를 추가합니다. */
  void Append(double Time, double Q[2], double Error[2], double Torque[2])
  {
    std::vector<double> Item = {Time, Q[0], Q[1], Error[0], Error[1], Torque[0], Torque[1]};
    Hist.push_back(Item);
  }

  /** Data를 추가합니다. */
  void Append(std::vector<double> Item)
  {
    Hist.push_back(Item);
  }

private:
  void WriteFile(std::vector<double>& Data, char Delimiter = ' ')
  {
    for(auto Iter = Data.begin(); Iter != Data.end(); Iter++)
    {
      fprintf(LogFile, "%f", *Iter);
      // printf("%f", *Iter);
      
      if(std::distance(Iter, Data.end()) != 1) { fprintf(LogFile, "%c", Delimiter); }
    }
    fprintf(LogFile, "\n");
    // printf("\n");
  }

private:
  std::vector<std::vector<double>> Hist;
  FILE* LogFile;

  std::string FilePath;
};