#include "shortest_path_scene.h"
#include <osg\Depth>
#include <osg\LineWidth>
#include <osg\PolygonOffset>
#include <osg\Point>
#include <osgWrapper\GeometryCreator.h>
#include <osgWrapper\RenderView.h>

#include <iostream>
#include <fstream>

#include "surface.h"
Scene::Scene(int argc, char* argv[], RenderView* view)
{
	m_surface.reset(new Surface(argv[1]));

	m_picker = new ColorIndexPicker(view->getCamera(), GetWidth(), GetHeight());
	m_manipulable_node = new ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new AttributeUtilNode();
	m_render_node->SetRenderProgram("preview_phong430");
	m_manipulable_node->AddChild(m_render_node);
	m_manipulable_node->AddChild(m_picker);
	m_polygon_mode = new osg::PolygonMode();
	m_render_node->SetAttribute(m_polygon_mode);

	m_path_node = new AttributeUtilNode();
	m_path_node->SetRenderProgram("purecolor430");
	m_path_node->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f)));
	m_path_node->SetAttribute(new osg::LineWidth(3.0f));
	m_path_node->SetAttribute(new osg::Point(10.0f));
	m_path_node->SetAttribute(new osg::Depth(osg::Depth::ALWAYS));
	//m_path_node->SetAttributeMode(new osg::PolygonOffset(-1.0f, -1.0f));
	//m_path_node->SetMode(GL_POLYGON_OFFSET_FILL, state_on);
	//osg::Depth* depth = new osg::Depth;
	//depth->setFunction(osg::Depth::LEQUAL);
	//m_path_node->SetAttribute(depth);

	m_manipulable_node->AddChild(m_path_node);
	m_path_node->AddChild(m_surface->GetPath());

	m_render_node->AddChild(m_surface->GetGeode());
	m_render_node->AddChild(m_path_node);

	m_picker->SetNode(m_surface->GetGeode());
	ResetCamera();
}

Scene::~Scene()
{

}

void Scene::OnEnter()
{
}

void Scene::OnEnterOut()
{

}

bool Scene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool Scene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_Z)
	{
		 
	}
	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_P)
	{
		static bool line_mode = false;
		line_mode = !line_mode;

		m_polygon_mode->setMode(osg::PolygonMode::FRONT_AND_BACK, line_mode ? osg::PolygonMode::LINE : osg::PolygonMode::FILL);
	}

	return true;
}

bool Scene::OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	int w = ea.getWindowWidth();
	int h = ea.getWindowHeight();

	return true;
}

bool Scene::OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() == osgGA::GUIEventAdapter::DOUBLECLICK && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
	{
		unsigned mesh_id = 0;
		unsigned primitive_id = 0;
		m_picker->Pick(ea.getX(), ea.getY(), mesh_id, primitive_id);
		if (primitive_id > 0)
		{
			std::cout << "pick." << std::endl;

			m_surface->AddControlPoint(primitive_id - 1, false);
		}
	}

	return true;
}

void Scene::ResetCamera()
{
	const osg::BoundingSphere& sphere = getBound();
	osg::Vec3f center = sphere.center();
	float radius = 0.5f * sphere.radius();

	float fovy = GetFovy();
	float len = radius / sin(fovy * 3.1415926f / 180.0f / 2.0f);
	osg::Vec3f eye = center + osg::Vec3f(0.0f, 0.0f, -1.0f) * len;
	osg::Vec3f up = osg::Vec3f(0.0f, -1.0f, 0.0f);
	osg::Matrixf view_matrix = osg::Matrixf::lookAt(eye, center, up);

	SetViewMatrix(view_matrix);
	SetNearFar(len - 3.1f * radius, len + 3.1f * radius);
}

