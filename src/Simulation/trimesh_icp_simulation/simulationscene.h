#pragma once
#include "renderthread.h"
#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\AttributeUtilNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AnimationScheduler.h>
#include "patchnode.h"

class SimulationScene : public RenderThreadBaseScene
{
public:
	SimulationScene();
	virtual ~SimulationScene();

	void ShowOneFrame(osg::Geometry* geometry, const osg::Matrixf& matrix);
private:
	void UpdateCamera();
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
protected:
	osg::ref_ptr<OSGWrapper::ManipulableNode> m_manipulable_node;
	osg::ref_ptr<OSGWrapper::Manipulator> m_manipulator;
	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_render_node;
	osg::ref_ptr<OSGWrapper::AnimationScheduler> m_animation_scheduler;

	osg::ref_ptr<PatchNode> m_current_node;
};