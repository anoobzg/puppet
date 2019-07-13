#include "scene.h"
#include <osg\Geometry>
#include <osg\PolygonMode>
#include <osg\BlendFunc>
#include <osg\BlendEquation>
#include <osgWrapper/GeometryCreator.h>
#include <osg/Depth>

Scene::Scene(osg::Geometry* geometry)
	:m_visual_robot_panel(true), m_visual_back_panel(true)
{
	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);
	

	m_robot_color = new osg::Uniform("color", osg::Vec4f(1.0f, 1.0f, 0.0f, 0.1f));
	m_back_color = new osg::Uniform("color", osg::Vec4f(0.0f, 0.0f, 0.0f, 0.0f));
	m_layer_node = new OSGWrapper::AttributeUtilNode();
	m_layer_node->SetRenderProgram("robot_background");
	//m_layer_node->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL));
	m_layer_node->AddUniform(m_back_color);
	m_layer_node->setComputeBoundingSphereCallback(new osg::Node::ComputeBoundingSphereCallback());
	osg::Depth* d = new osg::Depth();
	d->setWriteMask(true);
	//d->setFunction(osg::Depth::NEVER);
	m_layer_node->getOrCreateStateSet()->setAttributeAndModes(d, osg::StateAttribute::ON);
	osg::Vec2Array* coord_array = new osg::Vec2Array();
	coord_array->push_back(osg::Vec2f(-1.0f, -1.0f));
	coord_array->push_back(osg::Vec2f(1.0f, -1.0f));
	coord_array->push_back(osg::Vec2f(1.0f, 1.0f));
	coord_array->push_back(osg::Vec2f(-1.0f, 1.0f));
	osg::DrawArrays* draw_array = new osg::DrawArrays(GL_QUADS, 0, 4);
	osg::Geometry* quad = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array);
	m_layer_node->AddChild(quad);

	m_render_node = new OSGWrapper::AttributeUtilNode();
	m_render_node->SetMode(GL_BLEND, state_on);
	m_render_node->SetMode(GL_DEPTH_TEST, state_off);
	m_render_node->SetRenderProgram("robot_shader");
	m_render_node->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));
	m_render_node->AddUniform(m_robot_color);
	//osg::Depth* dd = new osg::Depth();
	//dd->setWriteMask(true);
	//m_render_node->SetAttribute(dd);

	addChild(m_layer_node);
	addChild(m_manipulable_node);
	//m_manipulable_node->AddChild(m_layer_node);
	m_manipulable_node->AddChild(m_render_node);
	m_render_node->AddChild(geometry);
	//m_layer_node->AddChild(geometry);

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

	m_robot_color_panel = new OSGWrapper::ColorPanel(100.0f, 0.5f, 0.5f);
	addChild(m_robot_color_panel);

	m_back_color_panel = new OSGWrapper::ColorPanel(0.0f, 0.0f, 0.0f);
	m_back_color_panel->SetAlpha(0.0f);
	m_back_color_panel->SetOrigin(osg::Vec2f(0.0f, 500.0f));
	addChild(m_back_color_panel);

	m_robot_color_panel->SetColorUniform(m_robot_color);
	m_back_color_panel->SetColorUniform(m_back_color);
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
	if (m_visual_robot_panel  && m_robot_color_panel->OnMouse(ea))
		return true;

	if (m_visual_back_panel && m_back_color_panel->OnMouse(ea))
		return true;

	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool Scene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_R))
	{
		m_visual_robot_panel = !m_visual_robot_panel;
		removeChild(m_robot_color_panel);
		if (m_visual_robot_panel) addChild(m_robot_color_panel);
	}
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_B))
	{
		m_visual_back_panel = !m_visual_back_panel;
		removeChild(m_back_color_panel);
		if (m_visual_back_panel) addChild(m_back_color_panel);
	}

	return true;
}