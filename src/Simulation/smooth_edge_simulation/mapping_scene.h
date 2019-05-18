#pragma once
#include <osgWrapper\RenderScene.h>

#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AttributeUtilNode.h>

#include "Mesh.h"

using namespace LauncaGeometry;
using namespace OSGWrapper;
class MappingScene : public RenderScene
{
public:
	MappingScene(Mesh& mesh);
	~MappingScene();

protected:
	void ResetCamera();

	void Setup();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

private:
	void ShowPolygonMesh(osg::Geode* geode);
	void ShowEdgeStrip(osg::Geode* geode);

	void Strip();
	osg::Geode* BuildStrip(Mesh& mesh, std::vector<unsigned>& triangles);
private:
	Mesh& m_mesh;

	osg::ref_ptr<ManipulableNode> m_manipulable_node;
	osg::ref_ptr<Manipulator> m_manipulator;
	osg::ref_ptr<AttributeUtilNode> m_render_node;
	osg::ref_ptr<AttributeUtilNode> m_edge_node;

	osg::ref_ptr<osg::Geode> m_mesh_geode;
};