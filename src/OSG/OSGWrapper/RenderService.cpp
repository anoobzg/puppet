#include <osgWrapper\RenderService.h>
#include <osgWrapper\ProgramManager.h>
#include <iostream>
namespace OSGWrapper
{
void RenderService::SetupRenderService()
{
	if(!m_instance)
		m_instance = new RenderService();
}

void RenderService::DestroyRenderService()
{
	if(m_instance)
	{
		delete m_instance;
		m_instance = 0;
	}
}

RenderService& RenderService::Instance()
{
	if(!m_instance)
		SetupRenderService();

	return *m_instance;
}

#define MIN_REFRESH_INTERVAL 25 //milliseconds

RenderService* RenderService::m_instance = 0;
RenderService::RenderService() 
	: m_is_rendering_enabled(true)
	, m_render_one_frame(false)
{
	/// receive events only on demand
	/// take all the events from the evens queue at frame event, not just the latest ones.
	_runFrameScheme = osgViewer::ViewerBase::ON_DEMAND;

	//avoid threading problems
	setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
	setReleaseContextAtEndOfFrameHint(false);

	ProgramManager::Instance();
}

RenderService::~RenderService()
{
	StopRenderingThread();

	ProgramManager::Release();
}


int RenderService::Run()
{
	while (true)
	{
		//double time_1 = getFrameStamp()->getReferenceTime();
		if (!m_render_one_frame && ! m_is_rendering_enabled )
		{
			continue;
		}

		//make sure there no other thread changes the rendering data
		{
			//double time_3 = getFrameStamp()->getReferenceTime();

			if (_views.size() == 0)
			{
				/// wait for at least a renderer to be created
				/// !!! neighter realize() or frame() should be called before adding a renderer to the composite renderer
				continue;
			}

			//double time_4 = getFrameStamp()->getReferenceTime();
			frame();

			if (done())
				break;
			//double time_5 = getFrameStamp()->getReferenceTime();

			double time = getFrameStamp()->getReferenceTime();
			static double ref_time = -1.0;
			if(ref_time < 0.0)
				ref_time = time;
			double delta = time - ref_time;
			ref_time = time;

			if(delta > 0.04167)
			{
				std::cout<<"frame total time :"<<delta<<std::endl;
			}
		}//unlock buffer			
	}

	return 0;
}

void RenderService::StartRenderingThread()
{
}

void RenderService::StopRenderingThread()
{
}

void RenderService::Refresh()
{
}

void RenderService::EnableRendering( bool enable )
{
}

bool RenderService::IsRenderingEnabled() const
{
	return m_is_rendering_enabled;
}

void RenderService::RenderOneFrameSynchronous()
{
}

void RenderService::SetRenderThreadPriority(int nPriority)
{
}

void RenderService::Clear()
{
	osgViewer::ViewerBase::Views views;
	getViews(views);
	
	if(views.size() > 0)
	{
		for(unsigned i = 0; i < views.size(); ++i)
			removeView(views[i]);

		views.clear();
	}
}

float RenderService::ReferenceTime()
{
	return (float)getFrameStamp()->getReferenceTime();
}

}