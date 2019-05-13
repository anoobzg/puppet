#pragma once
#ifndef ARRAY_CREATOR_H
#define ARRAY_CREATOR_H
#include <osg\Array>
#include <osg\PrimitiveSet>

namespace OSGWrapper
{

class OSG_EXPORT ArrayCreator
{
public:
	static osg::Array* CreateFloatArray(unsigned size, float* raw_data);
	static osg::Array* CreateVec2Array(unsigned size, float* raw_data);
	static osg::Array* CreateVec3Array(unsigned size, float* raw_data);
	static osg::Array* CreateVec4Array(unsigned size, float* raw_data);
	static osg::PrimitiveSet* CreatePrimitiveSet(osg::PrimitiveSet::Mode mode, unsigned size, unsigned* raw_data);
};

}
#endif // ARRAY_CREATOR