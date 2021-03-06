#pragma once
#include <osgWrapper/RenderScene.h>
#include <osgWrapper/ManipulableNode.h>
#include <osgWrapper/Manipulator.h>
#include <osgWrapper/AttributeUtilNode.h>
#include "lassonode.h"
#include <osg\Point>

enum lasso_type
{
	elasso_quad,
	elasso_circle,
	elasso_free,
	elasso_none
};
class LassoScene : public OSGWrapper::RenderScene
{
public:
	LassoScene();
	~LassoScene();

	void Set(osg::Vec3Array* coord_array, osg::Vec3Array* normal_array);

	void EnterCircleEraseMode();
	void EnterQuadEraseMode();
	void EnterFreeEraseMode();
	void EnterNoneMode();
	void Delete();
	void Reverse();
	void Flip();
	void Get(std::vector<bool>& flags);
protected:
	void ResetCamera();
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnFrame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void EnterOutCurrentMode(bool in);
private:
	osg::ref_ptr<osg::Vec3Array> m_coord_array;
	osg::ref_ptr<osg::Vec3Array> m_normal_array;
	osg::ref_ptr<osg::FloatArray> m_flag_array;
	osg::ref_ptr<osg::DrawArrays> m_draw_array;
	osg::ref_ptr<osg::Geometry> m_geometry;

	osg::ref_ptr<OSGWrapper::ManipulableNode> m_manipulable_node;
	osg::ref_ptr<OSGWrapper::Manipulator> m_manipulator;
	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_render_node;
	osg::ref_ptr<LassoNode> m_lasso_node;

	lasso_type m_lasso_type;

	osg::ref_ptr<osg::Point> m_point;
};