#include "scene.h"
#include <osg\Geometry>
#include <osg\PolygonMode>
#include <osg\BlendFunc>
#include <osg\BlendEquation>
#include <osgWrapper/GeometryCreator.h>
#include <osg/Depth>

Scene::Scene(osg::Geometry* geometry, osg::Texture* texture)
{
	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);
	
	m_render_node = new OSGWrapper::AttributeUtilNode();
	m_render_node->SetRenderProgram("pointphong430");
	m_render_node->SetTextureAttribute(0, texture);
	m_render_node->AddUniform(new osg::Uniform("tex", 0));
	m_manipulable_node->AddChild(m_render_node);
	m_render_node->AddChild(geometry);

	addChild(m_manipulable_node);
	UpdateCamera();
}

Scene::~Scene()
{

}

void Scene::UpdateCamera()
{
	const osg::BoundingSphere& sphere = m_manipulable_node->getBound();

	osg::Vec3f center = sphere.center();
	float radius = 0.8f * sphere.radius();

	float fovy = GetFovy();
	float len = radius / sin(fovy * 3.1415926f / 180.0f / 2.0f);
	osg::Vec3f eye = center + osg::Vec3f(0.0f, 0.0f, -1.0f) * len;
	osg::Vec3f up = osg::Vec3f(0.0f, -1.0f, 0.0f);
	osg::Matrixf view_matrix = osg::Matrixf::lookAt(eye, center, up);

	SetViewMatrix(view_matrix);
	SetNearFar(0.1f, len + 10.0f * radius);
}

bool Scene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool Scene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	return true;
}