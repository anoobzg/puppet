#pragma once
#include "../interface/slam_interface.h"

extern "C" _declspec(dllexport) esslam::IESSlam* CreateSlam();
extern "C" _declspec(dllexport) void DestroySlam(esslam::IESSlam* slam);