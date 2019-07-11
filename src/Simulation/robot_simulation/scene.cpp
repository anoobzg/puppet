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

	m_layer_node = new OSGWrapper::AttributeUtilNode();
	m_layer_node->SetRenderProgram("layer");
	m_layer_node->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));

	m_render_node = new OSGWrapper::AttributeUtilNode();
	m_render_node->SetMode(GL_BLEND, state_on);
	m_render_node->SetMode(GL_DEPTH_TEST, state_off);
	m_render_node->SetRenderProgram("antique");
	m_render_node->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));

	//m_manipulable_node->AddChild(m_layer_node);
	m_manipulable_node->AddChild(m_render_node);
	m_render_node->AddChild(geometry);
	m_layer_node->AddChild(geometry);

	UpdateCamera();

	m_hue = new ParametersText(L"HUE", 0);
	//addChild(m_hue);

	m_parameters.push_back(m_hue);

	//m_texture = new osg::Texture2D;
	//m_texture->setTextureSize(1920, 1080);
	//m_texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
	//m_texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
	//m_texture->setInternalFormat(GL_R32F);
	//m_texture->setSourceFormat(GL_RED);
	//m_texture->setSourceType(GL_FLOAT);
	//m_texture->bindToImageUnit(0, osg::Texture::READ_WRITE);
	//m_texture->setResizeNonPowerOfTwoHint(false);
	//
	//m_image = new osg::Image();
	//int size = 1920 * 1080;
	//float* data = new float[size];
	//for (int i = 0; i < size; ++i)
	//	* (data + i) = 0.0f;
	//m_image->setImage(1920, 1080, 1, GL_R32F, GL_RED, GL_FLOAT, (unsigned char*)data, osg::Image::USE_NEW_DELETE);
	//m_texture->setImage(m_image);
	//
	//m_render_node->getOrCreateStateSet()->setTextureAttributeAndModes(0, m_texture.get());
	//m_layer_node->getOrCreateStateSet()->setTextureAttributeAndModes(0, m_texture.get());
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