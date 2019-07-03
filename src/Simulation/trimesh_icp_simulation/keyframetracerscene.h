#pragma once
#include "renderthread.h"
#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\AttributeUtilNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AnimationScheduler.h>

class KeyFrameTracerScene : public RenderThreadBaseScene
{
public:
	KeyFrameTracerScene();
	virtual ~KeyFrameTracerScene();

	void AddKeyFrame(osg::Geometry* geometry, const osg::Matrixf& matrix);
	void Locate(osg::Geometry* geometry, const osg::Matrixf& matrix);
private:
	void UpdateCamera();
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
protected:
	osg::ref_ptr<OSGWrapper::ManipulableNode> m_manipulable_node;
	osg::ref_ptr<OSGWrapper::Manipulator> m_manipulator;
	osg::ref_ptr<OSGWrapper::AnimationScheduler> m_animation_scheduler;

	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_keyframe_node;
	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_current_node;
};