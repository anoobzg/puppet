#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AttributeUtilNode.h>
#include "icpnode.h"
#include "task.h"
#include "load_calib.h"
#include "screengraph.h"

class ICPScene : public OSGWrapper::RenderScene
{
public:
	ICPScene(trimesh::CameraData& data, trimesh::TriMesh& source, trimesh::TriMesh& target);
	~ICPScene();

protected:
	void UpdateCamera();
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnFrame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
protected:
	trimesh::CameraData& m_data;

	osg::ref_ptr<OSGWrapper::ManipulableNode> m_manipulable_node;
	osg::ref_ptr<OSGWrapper::Manipulator> m_manipulator;
	osg::ref_ptr<ICPNode> m_source_node;
	osg::ref_ptr<ICPNode> m_target_node;
	osg::ref_ptr<ScreenGraph> m_screen_graph;
	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_space_lines;

	osg::ref_ptr<Task> m_task;
};