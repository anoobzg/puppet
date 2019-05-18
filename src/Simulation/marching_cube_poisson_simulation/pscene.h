#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\AttributeUtilNode.h>
#include "poisson.h"

using namespace OSGWrapper;
class PScene : public RenderScene
{
public:
	PScene(int argc, char* argv[]);
	~PScene();

	void OnEnter();
	void OnEnterOut();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

protected:
	void LoadPoints(const char* file);
private:
	osg::ref_ptr<AttributeUtilNode> m_root;
	osg::ref_ptr<AttributeUtilNode> m_panel;
	osg::ref_ptr<AttributeUtilNode> m_grid;

	osg::ref_ptr<osg::Uniform> m_projection_uniform;
	std::string m_input_file;

	Poisson m_poisson;
};
