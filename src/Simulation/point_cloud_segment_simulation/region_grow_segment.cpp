#include "region_grow_segment.h"
#include "point_source.h"
#include <pcl\segmentation\region_growing.h>

void RegionGrowSegment::Do(PointSource& point_source, std::vector<float>& flags)
{
	// Region growing
	pcl::RegionGrowing<Point, Normal> rg;
	rg.setSmoothModeFlag(false); // Depends on the cloud being processed
	rg.setInputCloud(point_source.m_point_cloud);
	rg.setInputNormals(point_source.m_normal_cloud);
	rg.setMinClusterSize(3000);
	rg.setSmoothnessThreshold(50.0 / 180.0 * M_PI);
	std::vector <pcl::PointIndices> clusters;
	rg.extract(clusters);

	float value = 1.0f;
	flags.resize(point_source.m_point_cloud->size(), 1.0f);
	for (size_t i = 0; i < clusters.size(); ++i)
	{
		std::vector<int>& indices = clusters[i].indices;
		for (size_t j = 0; j < indices.size(); ++j)
		{
			int index = indices[j];
			flags[index] = value;
		}

		value += 1.0f;
		if (value > 10.0f) value = 1.0f;
	}
}