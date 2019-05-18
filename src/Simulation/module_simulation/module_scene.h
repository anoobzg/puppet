#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\AttributeUtilNode.h>

#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\ColorIndexPicker.h>

#include <osg\PolygonMode>

#include "curvature_geode.h"
#include "scene_logic_interface.h"

#include "control_ball.h"
using namespace OSGWrapper;
class Surface;
class Collider;
class ModuleScene : public RenderScene, public SurfaceTopoCallback
{
public:
	ModuleScene(Surface& surface, RenderView* view);
	~ModuleScene();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
protected:
	void ShowSurface(Mesh& mesh);
	void SurfaceCurvatureChanged(unsigned verrtex_number, float* curvature);
	void ControlPointAdded(ControlPoint& control_point);
	void ControlPointDeleted(ControlPoint& control_point);
	void ControlPointModified(ControlPoint& control_point);
	void PathAdded(Path& path);
	void PathRemoved(Path& path);

private:
	void ResetCamera();

	bool Hover(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool CheckCollide(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	void ScreenRay(float x, float y, float* ray_position, float* ray_direction);
	bool Drag(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool Delete(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void SelectControlPoint(unsigned control_point_handle);
	void HoverControlPoint(unsigned control_point_handle);
private:
	Surface& m_surface;
	std::auto_ptr<Collider> m_collider;

	osg::ref_ptr<ManipulableNode> m_manipulable_node;
	osg::ref_ptr<Manipulator> m_manipulator;
	osg::ref_ptr<AttributeUtilNode> m_render_node;
	osg::ref_ptr<ColorIndexPicker> m_picker;

	osg::ref_ptr<osg::PolygonMode> m_polygon_mode;

	osg::ref_ptr<AttributeUtilNode> m_control_node;
	osg::ref_ptr<AttributeUtilNode> m_control_path_node;
	osg::ref_ptr<osg::Geode> m_control_points_geode;
	osg::ref_ptr<osg::Geode> m_control_points_path_geode;

	osg::ref_ptr<CurvatureGeode> m_curvature_geode;

	std::map<unsigned, osg::ref_ptr<ControlBall>> m_control_balls;
	ControlBall* m_hove_ball;
	ControlBall* m_selected_ball;

	float m_proper_point_radius;
};