#pragma once
#ifndef _POLYHEDRON_PROFILER
#define _POLYHEDRON_PROFILER
#include <CGAL/Timer.h>

#define PROFILE_TIMER 	CGAL::Timer timer; timer.start();
#define PROFILE_TIME(x) std::cout << x << timer.time() << std::endl;  timer.reset();
#endif // _POLYHEDRON_PROFILER