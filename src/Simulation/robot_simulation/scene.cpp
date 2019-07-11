#include "scene.h"
#include <osg\Geometry>
#include <osg\PolygonMode>
#include <osg\BlendFunc>
#include <osg\BlendEquation>

Scene::Scene(osg::Geometry* geometry)
{
	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new OSGWrapper::AttributeUtilNode();
	m_render_node->SetRenderProgram("colorrobot");
	//m_render_node->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)));
	m_render_node->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));
	m_render_node->SetMode(GL_BLEND, state_on);
	m_render_node->SetAttribute(new osg::BlendEquation(osg::BlendEquation::FUNC_ADD, osg::BlendEquation::ALPHA_MAX));

	osg::BlendFunc* func = new osg::BlendFunc();
	func->setFunction(GL_SRC_ALPHA, GL_DST_ALPHA, GL_ZERO, GL_ONE);
	m_render_node->SetAttribute(func);
	m_manipulable_node->AddChild(m_render_node);

	m_render_node->AddChild(geometry);

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