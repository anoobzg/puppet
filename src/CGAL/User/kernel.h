#pragma once
#ifndef KERNEL_H
#define KERNEL_H
#include <CGAL\Simple_cartesian.h>

typedef double                DP;
typedef float				  FP;
typedef CGAL::Simple_cartesian<DP>  DKernel;
typedef CGAL::Simple_cartesian<FP>  FKernel;

#endif // KERNEL_H