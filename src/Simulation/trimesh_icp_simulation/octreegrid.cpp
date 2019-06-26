#include "octreegrid.h"
#include <osgWrapper/UtilCreator.h>

OctreeGrid::OctreeGrid(const osg::Vec3f& center)
{
	AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)));

	float res = 0.1f * (1 << 11);
	int n = 1 << 1;
	osg::Vec3i size = osg::Vec3i(n, n, n);
	float len = (float)(n/2) * res;
	osg::Vec3f bmin = center + osg::Vec3f(-len, -len, -len);
	osg::Vec3f bmax = center + osg::Vec3f(len, len, len);;

	AddChild(OSGWrapper::UtilCreator::CreateGrid(bmin, bmax, size));
}

OctreeGrid::~OctreeGrid()
{

}