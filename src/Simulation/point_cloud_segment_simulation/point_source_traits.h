#pragma once
#include <osg\vec3>
#include <vector>

class PointSource;
class PointSourceTraits
{
public:
	static void Trait(PointSource& point_source, std::vector<osg::Vec3f>& position, std::vector<osg::Vec3f>& normals, std::vector<float>& flags);
};