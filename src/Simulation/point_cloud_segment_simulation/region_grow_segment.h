#pragma once
#include <vector>

class PointSource;
class RegionGrowSegment
{
public:
	static void Do(PointSource& point_source, std::vector<float>& flags);
};