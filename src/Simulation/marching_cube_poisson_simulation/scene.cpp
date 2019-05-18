#include "scene.h"
#include <osg\Depth>
#include <osgWrapper\GeometryCreator.h>

#include <iostream>
#include <fstream>
Scene::Scene(int argc, char* argv[])
{
	m_input_file = argv[3];

	m_root = new AttributeUtilNode();
	m_root->AddUniform(new osg::Uniform("model_matrix", osg::Matrixf::identity()));
	m_root->AddUniform(new osg::Uniform("view_matrix", osg::Matrixf::identity()));
	m_projection_uniform = new osg::Uniform("projection_matrix", osg::Matrixf::ortho2D(0.0, 1080.0, 0.0, 720.0));
	m_root->AddUniform(m_projection_uniform);

	m_root->SetMode(GL_DEPTH_TEST, state_off);

	m_panel = new AttributeUtilNode();
	m_panel->SetRenderProgram("color430");

	m_grid = new AttributeUtilNode();
	m_grid->SetRenderProgram("purecolor430");
	m_grid->AddUniform(new osg::Uniform("color", osg::Vec4(0.85f, 0.85f, 0.85f, 1.0f)));

	m_line = new Line();
	m_line->getOrCreateStateSet()->addUniform(new osg::Uniform("color", osg::Vec4(0.0f, 0.0f, 1.0f, 0.0f)), state_on);
}

Scene::~Scene()
{

}

void Scene::OnEnter()
{
	addChild(m_root);
	m_root->AddChild(m_panel);
	m_root->AddChild(m_grid);

	m_grid->AddChild(m_base.GetGridGeode());
	m_grid->AddChild(m_base.GetMCGeode());
	m_grid->AddChild(m_line);
	m_grid->AddChild(m_base.GetPointsGeode());
	m_panel->AddChild(m_base.GetPanelGeode());

	LoadPoints(m_input_file.c_str());
}

void Scene::OnEnterOut()
{

}

bool Scene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	osgGA::GUIEventAdapter::EventType type = ea.getEventType();
	osg::Vec3f p1 = osg::Vec3f(-1000000.0f, -1000000.0f, -1000000.0f);
	osg::Vec3f p2 = osg::Vec3f(-1000000.0f, -1000000.0f, -1000000.0f);
	if (type == osgGA::GUIEventAdapter::PUSH && (ea.getButton() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
	{
		float x = ea.getX();
		float y = ea.getY();

		p1 = osg::Vec3f(x, y, 0.0f);
		p2 = osg::Vec3f(x, y, 0.0f);
		m_line->Set(p1, p2);
	}
	else if (type == osgGA::GUIEventAdapter::DRAG && (ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
	{
		float x = ea.getX();
		float y = ea.getY();

		p2 = osg::Vec3f(x, y, 0.0f);
		m_line->SetP2(p2);
	}
	else if (type == osgGA::GUIEventAdapter::RELEASE && (ea.getButton() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
	{
		osg::Vec3f v1, v2;
		m_line->Get(v1, v2);
		m_base.AddPoint(v1, v2);

		m_line->Set(p1, p2);
	}

	return true;
}

bool Scene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getKey() == osgGA::GUIEventAdapter::KEY_C)
		Clear();
	else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_O)
		Open();
	else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_S)
		Save();
	return true;
}

bool Scene::OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	int w = ea.getWindowWidth();
	int h = ea.getWindowHeight();

	m_projection_uniform->set(osg::Matrixf::ortho2D(0.0, (double)w, 0.0, (double)h));
	return true;
}

void Scene::LoadPoints(const char* file)
{
	std::fstream in;
	in.open(file, std::ios::in | std::ios::binary);
	if (in.is_open())
	{
		unsigned count = 0;
		in.read((char*)&count, sizeof(unsigned));
		for (unsigned i = 0; i < count; ++i)
		{
			float v[4];
			in.read((char*)v, 4 * sizeof(float));

			m_base.AddPoint(v[0], v[1], v[2], v[3]);
		}
	}
	in.close();
}

void Scene::Clear()
{
	m_base.Clear();
}

void Scene::Open()
{

}

void Scene::Save()
{

}