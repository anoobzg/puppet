#pragma once
#include <base/threading/thread.h>
#include <base/synchronization/waitable_event.h>
#include <osgWrapper/RenderScene.h>
#include <osgWrapper/RenderView.h>
#include <osgViewer/CompositeViewer>

class RenderThreadBaseScene : public OSGWrapper::RenderScene
{
	friend class RenderThread;
public:
	RenderThreadBaseScene();
	virtual ~RenderThreadBaseScene();

protected:
	base::Lock render_lock;
};

class RenderThread : public base::Thread
{
public:
	RenderThread(base::WaitableEvent& e);
	virtual ~RenderThread();

	void StartRender(RenderThreadBaseScene* scene);
	void StopRender();
private:
	void RenderCircle();
protected:
	osg::ref_ptr<RenderThreadBaseScene> m_scene;
	osg::ref_ptr<OSGWrapper::RenderView> m_view;
	osgViewer::CompositeViewer m_viewer;

	base::WaitableEvent& m_exit_event;
};