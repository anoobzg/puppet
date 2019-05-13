#pragma once
#include <OpenThreads\Thread>
#include <osg\Export>
#include <osgWrapper\RenderScene.h>

namespace OSGWrapper
{

	class OSG_EXPORT SimpleRenderThread : public OpenThreads::Thread
	{
	public:
		SimpleRenderThread(OSGWrapper::RenderScene* scene);
		~SimpleRenderThread();

		void run();
		int cancel();

	private:
		osg::ref_ptr<OSGWrapper::RenderScene> m_scene;
	};
}