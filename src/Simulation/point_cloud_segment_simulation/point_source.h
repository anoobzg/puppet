#pragma once
#include "pcl_def.h"

class PointSource
{
	friend class PointSourceTraits;
	friend class RegionGrowSegment;
public:
	PointSource();
	~PointSource();

	bool Load(const char* file_name);
private:
	PointCloud::Ptr m_point_cloud;
	NormalCloud::Ptr m_normal_cloud;
	KdTree::Ptr m_kdtree;
};