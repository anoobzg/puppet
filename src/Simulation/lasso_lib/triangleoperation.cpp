#include "triangleoperation.h"
#include <osgWrapper\RenderScene.h>
#include <iostream>
#include <osg\Depth>

TriangleOperation::TriangleOperation(int frame, FeedGeometry* geometry)
	:m_geometry(geometry), m_frame(frame)
{
	m_feed_node = new AttributeUtilNode();
	m_triangle_node = new AttributeUtilNode();
	m_point_node = new AttributeUtilNode();
	//osg::Camera* camera = new osg::Camera();
	//camera->setReferenceFrame(osg::Transform::RELATIVE_RF);
	//camera->setRenderOrder(osg::Camera::POST_RENDER, 0);
	//camera->addChild(m_point_node);
	AddChild(m_point_node);
	AddChild(m_triangle_node);
	AddChild(m_feed_node);

	m_feed_node->SetRenderProgram("feed_triangle");
	m_feed_node->AddUniform(new osg::Uniform("mask", (int)0));

	m_feed_node->AddChild(m_geometry);
	m_point_node->SetRenderProgram("viewport");
	m_point_node->SetAttribute(new osg::Depth(osg::Depth::ALWAYS));
	//m_point_node->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	m_triangle_node->SetRenderProgram("viewporttexture");
	m_triangle_node->SetAttribute(new osg::Depth(osg::Depth::ALWAYS));
	m_triangle_node->AddUniform(new osg::Uniform("mask", (int)0));
	m_vertex_array = new osg::Vec2Array();
	m_vertex_array->reserve(10000);
	m_points = new osg::Geometry();
	m_points->setCullingActive(false);
	m_points->setVertexAttribArray(0, m_vertex_array, osg::Array::BIND_PER_VERTEX);
	m_point_node->AddChild(m_points);
	m_triangles = new osg::Geometry();
	m_triangles->setCullingActive(false);
	m_triangles->setVertexAttribArray(0, m_vertex_array, osg::Array::BIND_PER_VERTEX);
	m_triangle_node->AddChild(m_triangles);
}

TriangleOperation::~TriangleOperation()
{

}

void TriangleOperation::SetTexture(osg::Texture2D* texture)
{
	m_texture = texture;
	m_feed_node->getOrCreateStateSet()->setTextureAttributeAndModes(0, m_texture.get());
	m_triangle_node->getOrCreateStateSet()->setTextureAttributeAndModes(0, m_texture.get());
}

const char* TriangleOperation::OperationName()
{
	return "TriangleOperation";
}

bool TriangleOperation::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	osg::Vec2f xy = osg::Vec2f(ea.getX(), ea.getY());
	//std::cout << "x :" << xy.x() << " y :" << xy.y() << std::endl;
	if (LEFT_MOUSE_PUSH(ea))
	{
		m_vertex_array->push_back(xy);
		UpdateGeometry();
	}
	else if (LEFT_MOUSE_DRAG(ea))
	{
		m_vertex_array->push_back(xy);
		UpdateGeometry();
	}
	else if (LEFT_MOUSE_RELEASE(ea))
	{
		m_vertex_array->clear();
		UpdateGeometry();
	}
	return true;
}

void TriangleOperation::UpdateGeometry()
{
	size_t size = m_vertex_array->size();
	m_points->removePrimitiveSet(0, m_points->getNumPrimitiveSets());
	m_triangles->removePrimitiveSet(0, m_triangles->getNumPrimitiveSets());
	if (size == 0)
	{
		return;
	}

	m_vertex_array->dirty();
	m_points->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP, 0, size));
	m_triangles->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLE_FAN, 0, size));
}
