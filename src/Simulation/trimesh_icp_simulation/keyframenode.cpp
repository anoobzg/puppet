#include "keyframenode.h"
#include <random>
#include <osg\Point>
#include <osg\PrimitiveSet>
#include <osgWrapper\GeometryCreator.h>
#include <iostream>

KeyFrameNode::KeyFrameNode()
{
	srand(time(NULL));
	int id = rand();
	m_matrix_uniform = new osg::Uniform("key_matrix", osg::Matrixf::identity());
	m_color_uniform = new osg::Uniform("colorid", (float)id);

	AddUniform(m_matrix_uniform);
	AddUniform(m_color_uniform);
	//SetAttribute(new osg::Point(8.0f));
}

KeyFrameNode::~KeyFrameNode()
{

}

void KeyFrameNode::SetKeyMatrix(const osg::Matrixf& matrix)
{
	m_matrix_uniform->set(matrix);
}

void KeyFrameNode::SetDefaultID()
{
	m_color_uniform->set(0.0f);
}

void KeyFrameNode::GenerateBoundingBox()
{
	const osg::BoundingSphere&  box = getBound();
	osg::Vec3f center = box.center();
	float radius = box.radius();

	//std::cout << "radius #######################" << radius << std::endl;
	osg::Vec3f dmin = center - osg::Vec3f(-radius, -radius, -radius);
	osg::Vec3f dmax = center + osg::Vec3f(radius, radius, radius);

	osg::Vec3Array* coord_array = new osg::Vec3Array();
	osg::Vec3Array* normal_array = new osg::Vec3Array();
	osg::Vec3f v0 = osg::Vec3f(dmin.x(), dmin.y(), dmin.z());
	osg::Vec3f v1 = osg::Vec3f(dmax.x(), dmin.y(), dmin.z());
	osg::Vec3f v2 = osg::Vec3f(dmax.x(), dmin.y(), dmax.z());
	osg::Vec3f v3 = osg::Vec3f(dmin.x(), dmin.y(), dmax.z());
	osg::Vec3f v4 = osg::Vec3f(dmin.x(), dmax.y(), dmin.z());
	osg::Vec3f v5 = osg::Vec3f(dmax.x(), dmax.y(), dmin.z());
	osg::Vec3f v6 = osg::Vec3f(dmax.x(), dmax.y(), dmax.z());
	osg::Vec3f v7 = osg::Vec3f(dmin.x(), dmax.y(), dmax.z());
	osg::Vec3f n0 = v0 - center;  n0.normalize();
	osg::Vec3f n1 = v1 - center;  n1.normalize();
	osg::Vec3f n2 = v2 - center;  n2.normalize();
	osg::Vec3f n3 = v3 - center;  n3.normalize();
	osg::Vec3f n4 = v4 - center;  n4.normalize();
	osg::Vec3f n5 = v5 - center;  n5.normalize();
	osg::Vec3f n6 = v6 - center;  n6.normalize();
	osg::Vec3f n7 = v7 - center;  n7.normalize();
	coord_array->push_back(v0);
	normal_array->push_back(n0);
	coord_array->push_back(v1);
	normal_array->push_back(n1);
	coord_array->push_back(v2);
	normal_array->push_back(n2);
	coord_array->push_back(v3);
	normal_array->push_back(n3);
	coord_array->push_back(v4);
	normal_array->push_back(n4);
	coord_array->push_back(v5);
	normal_array->push_back(n5);
	coord_array->push_back(v6);
	normal_array->push_back(n6);
	coord_array->push_back(v7);
	normal_array->push_back(n7);
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
	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_elements, coord_array, normal_array);
	AddChild(geometry);
}