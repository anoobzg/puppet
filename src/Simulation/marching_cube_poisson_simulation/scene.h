#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\AttributeUtilNode.h>
#include "base.h"
#include "line.h"

using namespace OSGWrapper;
class Scene : public RenderScene
{
public:
	Scene(int argc, char* argv[]);
	~Scene();

	void OnEnter();
	void OnEnterOut();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
protected:
	void LoadPoints(const char* file);

	void Clear();
	void Open();
	void Save();
private:
	osg::ref_ptr<AttributeUtilNode> m_root;
	osg::ref_ptr<AttributeUtilNode> m_panel;
	osg::ref_ptr<AttributeUtilNode> m_grid;

	Base m_base;

	osg::ref_ptr<osg::Uniform> m_projection_uniform;
	osg::ref_ptr<Line> m_line;

	std::string m_input_file;
};