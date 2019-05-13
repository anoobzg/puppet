#include <osgWrapper/ArrayCreator.h>
namespace OSGWrapper
{

osg::Array* ArrayCreator::CreateFloatArray(unsigned size, float* raw_data)
{
	osg::FloatArray* array = new osg::FloatArray(size, raw_data);
	return array;
}

osg::Array* ArrayCreator::CreateVec2Array(unsigned size, float* raw_data)
{
	osg::Vec2Array* array = new osg::Vec2Array(size, (const osg::Vec2*)raw_data);
	return array;
}

osg::Array* ArrayCreator::CreateVec3Array(unsigned size, float* raw_data)
{
	osg::Vec3Array* array = new osg::Vec3Array(size, (const osg::Vec3*)raw_data);
	return array;
}

osg::Array* ArrayCreator::CreateVec4Array(unsigned size, float* raw_data)
{
	osg::Vec4Array* array = new osg::Vec4Array(size, (const osg::Vec4*)raw_data);
	return array;
}

osg::PrimitiveSet* ArrayCreator::CreatePrimitiveSet(osg::PrimitiveSet::Mode mode, unsigned size, unsigned* raw_data)
{
	osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(mode, size, (const GLuint*)raw_data);
	return primitive;
}

}