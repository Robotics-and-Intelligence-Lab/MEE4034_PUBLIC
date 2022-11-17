#ifdef _WIN32
    #define _USE_MATH_DEFINES
#endif

#include <math.h>

#include "EposManager.h"
#include "Definitions.h"

UEposManager::UEposManager() : PulsePerRev(2048 * 4), bOpened{false, false}, MainDev(nullptr), SubDev(nullptr), GearRatio{3, 3}, HomePosition{-M_PI/2.0, -M_PI/2.0}
{
    TorqueConst[0] = 40.4e-3; // [Nm/A]
    TorqueConst[1] = 36.9e-3; // [Nm/A]
    MaxCurrent[0] = 4.0; // A
    MaxCurrent[1] = 3.21; // A
}

UEposManager::~UEposManager()
{
    DisableController();

    CloseMainDev();
    CloseSubDev();
}

/** Open devices & Change OP mode to Current Mode (CM) */
void UEposManager::Init(bool bReset)
{
    // Open devices
    OpenMainDev();
    OpenSubDev();

    if(bOpened[0] && bOpened[1]) 
    {
        ClearError();
        if(bReset) { ResetAll(); }
        
        // Change OP mode to Current Mode (CM) 
        // if(!VCS_SetOperationMode(MainDev, 1, OMD_CURRENT_MODE, &ErrorCode)) 
        // {
        //     std::cerr << "Can't change 1st dev's op mode" << std::endl;
        // }
        if(!VCS_SetOperationMode(SubDev, 1, OMD_CURRENT_MODE, &ErrorCode)) 
        {
            std::cerr << "Can't change 1st dev's op mode" << std::endl;
        }
        if(!VCS_SetOperationMode(SubDev, 2, OMD_CURRENT_MODE, &ErrorCode))
        {
            std::cerr << "Can't change 2nd dev's op mode" << std::endl;
        }
    }
}

/** Switch controller's state to Enabled */
void UEposManager::EnableController()
{
    if(!VCS_SetEnableState(SubDev, 1, &ErrorCode))
    {
        std::cerr << "Can't change 1st dev's enable state" << std::endl;
    }
    if(!VCS_SetEnableState(SubDev, 2, &ErrorCode))
    {
        std::cerr << "Can't change 2nd dev's enable state" << std::endl;
    }    
}

/** Disable controller */
void UEposManager::DisableController()
{
    // if(bOpened[0] && !VCS_SetDisableState(MainDev, 3, &ErrorCode))
    // {
    //     std::cerr << "Can't change main dev's enable state" << std::endl;
    // }
    if(bOpened[1] && !VCS_SetDisableState(SubDev, 1, &ErrorCode))
    {
        std::cerr << "Can't change 1st dev's enable state" << std::endl;
    }    
    if(bOpened[1] && !VCS_SetDisableState(SubDev, 2, &ErrorCode))
    {
        std::cerr << "Can't change 2nd dev's enable state" << std::endl;
    }    
}

/** Clear controller's error */
void UEposManager::ClearError()
{
    VCS_ClearFault(MainDev, 3, &ErrorCode);
    VCS_ClearFault(SubDev, 1, &ErrorCode);
    VCS_ClearFault(SubDev, 2, &ErrorCode);    
}

/** Reset the all devices (to change motor 0 position) */
void UEposManager::ResetAll()
{   
    std::cout << "Reset the 1st device" << std::endl;
    // if(!VCS_ResetDevice(MainDev, 1, &ErrorCode))
    if(!VCS_ResetDevice(SubDev, 1, &ErrorCode))
    {
        std::cerr << "Can't reset 1st device" << std::endl;        
    }

    std::cout << "Reset the 2nd device" << std::endl;
    if(!VCS_ResetDevice(SubDev, 2, &ErrorCode))
    {
        std::cerr << "Can't reset 2nd device" << std::endl;        
    }
}

/** Set desired torque in Nm */
void UEposManager::SetTorque(double T1, double T2)
{
    // ScaleFactor: Convert Torque[Nm] to Current[A]
    static double ScaleFactor[2] = { 1000.0 / (TorqueConst[0] * GearRatio[0]), 1000.0 / (TorqueConst[1] * GearRatio[1]) };
    static int MaxCurrentMA[2] = {static_cast<int>(1000.0 * MaxCurrent[0]), static_cast<int16_t>(1000.0 * MaxCurrent[1])};

    // short CommandedCurrent;
    // VCS_GetCurrentMust(SubDev, 1, &CommandedCurrent, &ErrorCode);
    // std::cout << "C1[A]: " << CommandedCurrent << std::endl;

    int Current = static_cast<int>(T1 * ScaleFactor[0]);
    ApplyClipping(Current, -MaxCurrentMA[0], MaxCurrentMA[0]);
    std::cout << "C1: " << Current << std::endl;
    // VCS_SetCurrentMustEx(MainDev, 1, Current, &ErrorCode);
    VCS_SetCurrentMustEx(SubDev, 1, Current, &ErrorCode);



    Current = static_cast<int>(T2 * ScaleFactor[1]);
    ApplyClipping(Current, -MaxCurrentMA[1], MaxCurrentMA[1]);
    std::cout << "C2: " << Current << std::endl;
    VCS_SetCurrentMustEx(SubDev, 2, Current, &ErrorCode);
}

/** Get current position in rads */
void UEposManager::GetPosition(double &Q1, double &Q2)
{
    // ScaleFactor: Convert motor's position(pulse) to Actual joint's position(rads)
    static double ScaleFactor[2] = {2.0 * M_PI / (PulsePerRev * GearRatio[0]), 2.0 * M_PI / (PulsePerRev * GearRatio[1]) };

    static int PositionPulse;    
    // VCS_GetPositionIs(MainDev, 1, &PositionPulse, &ErrorCode);
    VCS_GetPositionIs(SubDev, 1, &PositionPulse, &ErrorCode);
    Q1 = PositionPulse * ScaleFactor[0] + HomePosition[0];
    VCS_GetPositionIs(SubDev, 2, &PositionPulse, &ErrorCode);
    Q2 = PositionPulse * ScaleFactor[1] + HomePosition[1];

    // std::cout << HomePosition[0] << std::endl;
}

/** Get current velocity in rads/s */
double UEposManager::GetVelocity(double &V1, double &V2)
{
    // ScaleFactor: Convert motor's velocity(RPM) to Actual joint's velocity(rads/s)
    static double ScaleFactor[2] = {2.0 * M_PI / (60.0 * GearRatio[0]), 2.0 * M_PI / (60.0 * GearRatio[1])}; // 1[rev]/1[min] = 2*pi[rads]/60[s]

    static int Velocity;
    // VCS_GetVelocityIs(MainDev, 1, &Velocity, &ErrorCode);
    VCS_GetVelocityIs(SubDev, 1, &Velocity, &ErrorCode);
    V1 = Velocity * ScaleFactor[0];
    VCS_GetVelocityIs(SubDev, 2, &Velocity, &ErrorCode);
    V2 = Velocity * ScaleFactor[1];
}

template <typename T>
inline void UEposManager::ApplyClipping(T& Value, T Min, T Max)
{
    Value = std::max(Min, std::min(Max, Value));
}
  
/** Open the 1st controller device */
void UEposManager::OpenMainDev()
{
    char* DevName = "EPOS2";
    char* ProtocolStackName = "MAXON SERIAL V2";
    char* InterfaceName = "USB";
    char* PortName = "USB0";
    int Baudrate = 1000000;

    std::cout << "Open main device" << std::endl;
    
    MainDev = VCS_OpenDevice(DevName, ProtocolStackName, InterfaceName, PortName, &ErrorCode);
    if(MainDev) 
    {
        std::cout << "Main device opened" << std::endl;
        bOpened[0] = true;
    }
}

/** Open the 2nd controller device */
void UEposManager::OpenSubDev()
{
    if(!bOpened[0]) 
    {
        std::cerr << "Main device should be opened" << std::endl;
    }    

    char* DevName = "EPOS2";
    char* ProtocolStackName = "CANopen";

    std::cout << "Open sub device" << std::endl;
    SubDev = VCS_OpenSubDevice(MainDev, DevName, ProtocolStackName, &ErrorCode);

    if(SubDev)
    {
        std::cout << "Sub device opened" << std::endl;
        bOpened[1] = true;
    }
}  

/** Close the 1st controller device */
void UEposManager::CloseMainDev()
{
    if(!bOpened[0]) { return; }

    if(VCS_CloseDevice(MainDev, &ErrorCode) != 0)
    {
        std::cout << "Main device closed" << std::endl;
    }
    else
    {
        std::cerr << "Can't close main device" << std::endl;
    }    
}

/** Close the 2nd controller device */
void UEposManager::CloseSubDev()
{
    if(!bOpened[1]) { return; }
    
    if(VCS_CloseSubDevice(SubDev, &ErrorCode))
    {
        printf("Sub device closed\n");
    }
    else
    {
        printf("Can't close sub device");
    }
}