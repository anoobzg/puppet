#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\AttributeUtilNode.h>

#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\ColorIndexPicker.h>

#include <osg\PolygonMode>
#include <memory>

#include "Mesh.h"
#include "PathModule.h"

using namespace LauncaGeometry;
using namespace OSGWrapper;

class Scene : public RenderScene
{
public:
	Scene(Mesh& mesh, RenderView* view);
	~Scene();

	void OnEnter();
	void OnEnterOut();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

private:
	Mesh & m_mesh;
	std::shared_ptr<Surface> m_surface;
	std::auto_ptr<PathModule> m_pathModule;

	osg::ref_ptr<ManipulableNode> m_manipulable_node;
	osg::ref_ptr<Manipulator> m_manipulator;
	osg::ref_ptr<AttributeUtilNode> m_render_node;
	osg::ref_ptr<ColorIndexPicker> m_picker;
	osg::ref_ptr<osg::PolygonMode> m_polygon_mode;

	void ResetCamera();
	void _resetCurvature();
	void _calRayIntersectTri(unsigned meshId, unsigned primitiveId, int nSX, int nSY, float outUVWCoord[3], float outClickCoord[3]);
	osg::Geode* CreateMeshGeode(Mesh& mesh);
	bool CheckCollide(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool Drag(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool Hover(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	void GetRay(float sx, float sy, osg::Vec3f& eye, osg::Vec3f& center);
};