#pragma once
#include "interface.h"

class Octree;
class Solver
{
public:
	Solver(unsigned depth);
	~Solver();

	void BuildTable(unsigned depth);
	void Solve(Octree& octree);
private:
	double* m_lookup_table_0;
	double* m_lookup_table_1;
	double* m_lookup_table_2;
};