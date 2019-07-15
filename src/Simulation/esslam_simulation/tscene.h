#pragma once
#include "RenderThread.h"
#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\AttributeUtilNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AnimationScheduler.h>
#include <osgWrapper/ScreenLineText.h>
#include "slam_osg.h"
#include "framenode.h"

class TScene : public simtool::RenderThreadBaseScene , public esslam::IOSGTracer
{
public:
	TScene();
	virtual ~TScene();

protected:
	void OnFrameLocated(int effect_num, const std::vector<trimesh::point3>& vertexes,
		const std::vector<trimesh::point3>& normals, const std::vector<trimesh::point3>& colors,
		const trimesh::xform& xf, bool lost);
private:
	void Convert(osg::Matrixf& matrix, const trimesh::xform& xf);
	void UpdateCamera();
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
protected:
	osg::ref_ptr<OSGWrapper::ManipulableNode> m_manipulable_node;
	osg::ref_ptr<OSGWrapper::Manipulator> m_manipulator;
	osg::ref_ptr<FrameNode> m_frame;
};