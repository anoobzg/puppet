#include "pscene.h"
#include <osg\Depth>
#include <osgWrapper\GeometryCreator.h>

#include <iostream>
#include <fstream>
PScene::PScene(int argc, char* argv[])
{
	m_input_file = argv[3];

	OcCell::min_cell_len = (float)atof(argv[1]);

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

	LoadPoints(m_input_file.c_str());

	m_poisson.Generate();
}

PScene::~PScene()
{

}

void PScene::OnEnter()
{
	addChild(m_root);
	m_root->AddChild(m_panel);
	m_root->AddChild(m_grid);

	m_grid->AddChild(m_poisson.GetGridGeode());
	m_grid->AddChild(m_poisson.GetPoissonGeode());
	m_grid->AddChild(m_poisson.GetPointsGeode());
	m_panel->AddChild(m_poisson.GetPanelGeode());
}

void PScene::OnEnterOut()
{

}

bool PScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	return true;
}

bool PScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	return true;
}

bool PScene::OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	int w = ea.getWindowWidth();
	int h = ea.getWindowHeight();

	m_projection_uniform->set(osg::Matrixf::ortho2D(0.0, (double)w, 0.0, (double)h));
	return true;
}

void PScene::LoadPoints(const char* file)
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

			m_poisson.AddPoint(v[0], v[1], v[2], v[3]);
		}
	}
	in.close();
}