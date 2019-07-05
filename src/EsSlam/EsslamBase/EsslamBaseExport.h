#pragma once

#ifdef ESSLAM_EXPORTS
#define ESSLAM_API __declspec(dllexport)
#else
#define ESSLAM_API __declspec(dllimport)
#endif