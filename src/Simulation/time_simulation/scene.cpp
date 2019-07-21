#include "scene.h"
#include <osg\Geometry>
#include <osg\PolygonMode>
#include <osg\BlendFunc>
#include <osg\BlendEquation>
#include <osgWrapper/GeometryCreator.h>
#include <osg/Depth>
#include <osgWrapper/UIPanel.h>

Scene::Scene()
{
	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);

	m_time_panel = new TimePanel();
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
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_R))
	{
	}
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_B))
	{
	}

	return true;
}

void Scene::OnEnterUI(OSGWrapper::UIPanel* panel)
{
	panel->SetGlobalUI(new OSGWrapper::UIItem());
	panel->AddUI(m_time_panel);
}

void Scene::OnEnterOutUI(OSGWrapper::UIPanel* panel)
{

}

void Scene::LoadFrom(int argc, char* argv[])
{
	const char* usb = 0;
	const char* build = 0;
	const char* locate = 0;
	const char* fusion = 0;

	if (argc >= 2) usb = argv[1];
	if (argc >= 3) build = argv[2];
	if (argc >= 4) locate = argv[3];
	if (argc >= 5) fusion = argv[4];
	m_time_panel->Load(usb, build, locate, fusion);
}