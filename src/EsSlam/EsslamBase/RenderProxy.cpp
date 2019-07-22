#include "RenderProxy.h"
#include "../interface/slam_data.h"
#include <base\bind.h>

namespace esslam
{
	RenderProxy::RenderProxy()
		:base::Thread("RenderProxy"), m_visual_tracer(NULL)
	{

	}

	RenderProxy::~RenderProxy()
	{

	}

	void RenderProxy::SetVisualTracer(IVisualTracer* tracer)
	{
		m_visual_tracer = tracer;
	}

	void RenderProxy::StartProcess()
	{
		Start();
	}

	void RenderProxy::StopProcess()
	{
		Stop();
		m_visual_tracer = NULL;
	}

	void RenderProxy::OnCurrentFrame(CurrentFrameData* data)
	{
		task_runner()->PostTask(FROM_HERE, base::Bind(&RenderProxy::ProcessCurrentFrame, base::Unretained(this), data));
	}

	void RenderProxy::OnAppendNewPoints(NewAppendData* data)
	{
		task_runner()->PostTask(FROM_HERE, base::Bind(&RenderProxy::ProcessNewData, base::Unretained(this), data));
	}

	void RenderProxy::ProcessCurrentFrame(CurrentFrameData* data)
	{
		if (m_visual_tracer)
		{
			TriMeshPtr& mesh = data->mesh;
			FrameData* frame_data = new FrameData();
			frame_data->lost = data->lost;
			for (int i = 0; i < 16; ++i)
				frame_data->xf[i] = (float)mesh->global[i];
			size_t size = mesh->vertices.size();
			frame_data->position.resize(size);
			frame_data->normals.resize(size);
			frame_data->distances.resize(size);

			memcpy(&frame_data->position.at(0), &mesh->vertices.at(0), sizeof(float) * 3 * size);
			memcpy(&frame_data->normals.at(0), &mesh->normals.at(0), sizeof(float) * 3 * size);
			for (size_t i = 0; i < size; ++i)
			{
				frame_data->distances.at(i) = mesh->vertices.at(i).x;
			}

			if (data->fix)
			{
				std::cout << "Fix Frame" << std::endl;
				m_visual_tracer->OnFixFrame(frame_data);
			}
			else
				m_visual_tracer->OnFrameLocated(frame_data);
		}

		delete data;
	}

	void RenderProxy::ProcessNewData(NewAppendData* data)
	{
		if (m_visual_tracer)
		{
			m_visual_tracer->OnNewPoints(data);
		}else
			delete data;
	}
}