#pragma once

#ifdef SIMULATION_TOOLS_EXPORTS
#define SIMTOOL_API __declspec(dllexport)
#else
#define SIMTOOL_API __declspec(dllimport)
#endif