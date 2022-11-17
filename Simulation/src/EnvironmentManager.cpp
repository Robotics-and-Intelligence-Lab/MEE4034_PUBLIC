#include "EnvironmentManager.h"

#include <iostream>

UEnvironmentManager::UEnvironmentManager()
{
    Initialize("model/two_link_arm.xml");
}

UEnvironmentManager::~UEnvironmentManager()
{
    // free visualization storage
    mjv_freeScene(&SimScene);
    mjr_freeContext(&SimContext);

    // free MuJoCo model and data, deactivate
    mj_deleteData(SimData);
    mj_deleteModel(SimModel);
    mj_deactivate();

#if defined(_WIN32)
    glfwTerminate();
#endif
}

void UEnvironmentManager::SetCallback(std::function<void(double)> Func)
{
    bCallbackAttached = true;
    ControlCallback = Func;
}

void UEnvironmentManager::GetJointPosition(double Q[2])
{
    Q[0] = SimData->qpos[0];
    Q[1] = SimData->qpos[1];
}

void UEnvironmentManager::GetJointVelocity(double V[2])
{
    V[0] = SimData->qvel[0];
    V[1] = SimData->qvel[1];
}

void UEnvironmentManager::GetJointAcceleration(double Acc[2])
{
    Acc[0] = SimData->qacc[0];
    Acc[1] = SimData->qacc[1];
}

void UEnvironmentManager::SetJointTorque(double T[2])
{
    SimData->ctrl[0] = T[0];
    SimData->ctrl[1] = T[1];
}

void UEnvironmentManager::StepOnce()
{
    static mjtNum LatestScreenUpdateTime = -1.0 / DESIRED_FPS;
    static mjtNum LatestCallTime = -CallbackInterval;

    if (SimData->time - LatestScreenUpdateTime >= 1.0 / DESIRED_FPS)
    {
        LatestScreenUpdateTime = SimData->time;
        UpdateScreen();
    }

    if (SimData->time - LatestCallTime >= CallbackInterval)
    {
        if (bCallbackAttached)
        {
            Callback(SimData->time - LatestCallTime);
        }
        LatestCallTime = SimData->time;
    }

    mj_step(SimModel, SimData);
}

void UEnvironmentManager::UpdateScreen()
{
    if (glfwWindowShouldClose(Window))
        return;

    mjrRect Viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(Window, &Viewport.width, &Viewport.height);
    mjv_updateScene(SimModel, SimData, &SimOption, NULL, &SimCam, mjCAT_ALL, &SimScene);
    mjr_render(Viewport, &SimScene, &SimContext);

    glfwSwapBuffers(Window);
    glfwPollEvents();
}

double UEnvironmentManager::GetSimulationTime()
{
    return SimData->time;
}

void UEnvironmentManager::SetControlLoopFrequency(double Freq)
{
    CallbackInterval = 1 / Freq;
}

void UEnvironmentManager::Initialize(std::string ModelPath)
{
    // load and compile model
    SimModel = mj_loadXML(ModelPath.c_str(), 0, ErrBuffer, 1000);
    if (!SimModel)
    {
        printf("Load model error: %s\n", ErrBuffer);
    }

    // make data
    SimData = mj_makeData(SimModel);

    // init GLFW
    if (!glfwInit())
    {
        printf("Could not initialize GLFW\n");
    }

    Window = glfwCreateWindow(1200, 700, "Simulator", NULL, NULL);
    glfwMakeContextCurrent(Window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&SimCam);
    mjv_defaultOption(&SimOption);
    mjv_defaultScene(&SimScene);
    mjr_defaultContext(&SimContext);
    mjv_makeScene(SimModel, &SimScene, 2000);                // space for 2000 objs
    mjr_makeContext(SimModel, &SimContext, mjFONTSCALE_150); // model-specific context

    SimCam.azimuth = 90.0;
    SimCam.elevation = -90.0;
    SimCam.distance = 1.0;
    SimCam.lookat[0] = 0;
    SimCam.lookat[1] = 0;
    SimCam.lookat[2] = 0;
}