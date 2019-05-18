#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\AttributeUtilNode.h>

#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\ColorIndexPicker.h>

#include "Mesh.h"
#include "PointCloud.h"

using namespace OSGWrapper;
using namespace LauncaGeometry;

class Point2MeshReconSegmentScene : public RenderScene
{
public:
	Point2MeshReconSegmentScene(RenderView* view, Mesh& mesh, Mesh& mc_mesh, PointCloud& cloud);
	~Point2MeshReconSegmentScene();

protected:
	void ResetCamera();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void ToggleClip();

	void UpdateClipPlane(unsigned start_index, unsigned end_index);
	void RemoveClipPlane();

	bool CheckCollide(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool Drag(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool Release(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void AddClipPlane(const osg::Vec3f& center);
	void ExitMode();

	osg::Geometry* CreatePoint();

	void ToggleMC();
	void TogglePoisson();
private:
	Mesh& m_mesh;
	Mesh& m_mc_mesh;
	PointCloud& m_cloud;

	osg::ref_ptr<ManipulableNode> m_manipulable_node;
	osg::ref_ptr<Manipulator> m_manipulator;
	osg::ref_ptr<AttributeUtilNode> m_mesh_render_node;
	osg::ref_ptr<ManipulableNode> m_clip_mani_node;
	osg::ref_ptr<AttributeUtilNode> m_clip_plane_node;
	osg::ref_ptr<ColorIndexPicker> m_picker;

	osg::ref_ptr<ManipulableNode> m_surface_node;
	osg::ref_ptr<AttributeUtilNode> m_surface_render_node;
	osg::ref_ptr<osg::Geometry> m_mesh_geometry;
	osg::ref_ptr<osg::Geometry> m_mc_geometry;
	osg::ref_ptr<osg::Geometry> m_point_geometry;

	osg::ref_ptr<osg::Uniform> m_use_clip;
	osg::ref_ptr<osg::Uniform> m_center;

	unsigned m_plane_start_index;
	float m_plane_position[3];
	float m_plane_normal[3];

	bool m_mc_visual;
	bool m_poisson_visual;
};
