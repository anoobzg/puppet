#pragma once
#include <osgWrapper\RenderScene.h>

#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AttributeUtilNode.h>

#include "Mesh.h"
#include "GemTransform.h"

#include "SegModule.h"
using namespace OSGWrapper;
using namespace LauncaGeometry;
class ConSegScene : public RenderScene
{
public:
	ConSegScene(Mesh& mesh, SegModule& module);
	~ConSegScene();

protected:
	void ResetCamera();
	void Setup();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
private:
	void UpdateOneStep();
private:
	Mesh& m_mesh;
	SegModule& m_module;

	osg::ref_ptr<ManipulableNode> m_manipulable_node;
	osg::ref_ptr<Manipulator> m_manipulator;
	osg::ref_ptr<AttributeUtilNode> m_render_node;
	osg::ref_ptr<osg::Geode> m_geode;

	osg::ref_ptr<osg::Uniform> m_uniform;
	osg::ref_ptr<osg::FloatArray> m_color_array;
};