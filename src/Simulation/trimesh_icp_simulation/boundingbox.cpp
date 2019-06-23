#include "boundingbox.h"
#include <osg\PrimitiveSet>
#include <osgWrapper\GeometryCreator.h>

BoundingBox::BoundingBox()
{
	SetRenderProgram("purecolor430");
	//m_matrix = new osg::Uniform("model_matrix", osg::Matrixf::identity());
	//AddUniform(m_matrix);
}

BoundingBox::~BoundingBox()
{

}

void BoundingBox::UpdateBoundingBox(const osg::BoundingBoxf& box, const osg::Matrixf& matrix)
{
	const osg::Vec3f& min = box._min;
	const osg::Vec3f& max = box._max;
	osg::BoundingBoxf obox;
	osg::Vec3f v0 = osg::Vec3f(min.x(), min.y(), min.z());
	osg::Vec3f v1 = osg::Vec3f(max.x(), min.y(), min.z());
	osg::Vec3f v2 = osg::Vec3f(max.x(), min.y(), max.z());
	osg::Vec3f v3 = osg::Vec3f(min.x(), min.y(), max.z());
	osg::Vec3f v4 = osg::Vec3f(min.x(), max.y(), min.z());
	osg::Vec3f v5 = osg::Vec3f(max.x(), max.y(), min.z());
	osg::Vec3f v6 = osg::Vec3f(max.x(), max.y(), max.z());
	osg::Vec3f v7 = osg::Vec3f(min.x(), max.y(), max.z());
	obox.expandBy(v0 * matrix);
	obox.expandBy(v1 * matrix);
	obox.expandBy(v2 * matrix);
	obox.expandBy(v3 * matrix);
	obox.expandBy(v4 * matrix);
	obox.expandBy(v5 * matrix);
	obox.expandBy(v6 * matrix);
	obox.expandBy(v7 * matrix);

	UpdateBoundingBox(obox._min, obox._max);
}

void BoundingBox::UpdateBoundingBox(const osg::Vec3f& min, const osg::Vec3f& max)
{
	osg::Vec3Array* coord_array = new osg::Vec3Array();
	osg::Vec3f v0 = osg::Vec3f(min.x(), min.y(), min.z());
	osg::Vec3f v1 = osg::Vec3f(max.x(), min.y(), min.z());
	osg::Vec3f v2 = osg::Vec3f(max.x(), min.y(), max.z());
	osg::Vec3f v3 = osg::Vec3f(min.x(), min.y(), max.z());
	osg::Vec3f v4 = osg::Vec3f(min.x(), max.y(), min.z());
	osg::Vec3f v5 = osg::Vec3f(max.x(), max.y(), min.z());
	osg::Vec3f v6 = osg::Vec3f(max.x(), max.y(), max.z());
	osg::Vec3f v7 = osg::Vec3f(min.x(), max.y(), max.z());
	coord_array->push_back(v0);
	coord_array->push_back(v1);
	coord_array->push_back(v2);
	coord_array->push_back(v3);
	coord_array->push_back(v4);
	coord_array->push_back(v5);
	coord_array->push_back(v6);
	coord_array->push_back(v7);
	osg::DrawElementsUInt* draw_elements = new osg::DrawElementsUInt(GL_LINE_STRIP);
	draw_elements->push_back(0);
	draw_elements->push_back(1);
	draw_elements->push_back(2);
	draw_elements->push_back(3);
	draw_elements->push_back(0);
	draw_elements->push_back(4);
	draw_elements->push_back(5);
	draw_elements->push_back(1);
	draw_elements->push_back(2);
	draw_elements->push_back(6);
	draw_elements->push_back(7);
	draw_elements->push_back(3);
	draw_elements->push_back(7);
	draw_elements->push_back(4);
	draw_elements->push_back(7);
	draw_elements->push_back(6);
	draw_elements->push_back(5);
	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_elements, coord_array);
	RemoveAll();
	AddChild(geometry);
}