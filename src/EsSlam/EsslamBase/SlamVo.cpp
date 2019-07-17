#include "SlamVo.h"
#include <base\bind.h>
#include "timestamp.h"

namespace esslam
{

	SlamVO::SlamVO()
		:base::Thread("SlamVO"), m_profiler(NULL)
		, m_visual_processor(NULL)
	{

	}

	SlamVO::~SlamVO()
	{

	}

	void SlamVO::StartVO(const SlamParameters& parameters)
	{
		m_parameters = parameters;

		m_vo_impl.Setup(m_parameters);
		m_vo_locator.Setup(m_parameters);
		m_vo_fusion.Setup(m_parameters);
		m_vo_fusion.StartFusion();
		m_vo_fusion.SetKeyFrameAdder(this);

		bool start = Start();
	}

	void SlamVO::StopVO()
	{
		Stop();
		m_vo_fusion.StopFusion();
	}

	void SlamVO::SetVOProfiler(VOProfiler* profiler)
	{
		m_profiler = profiler;
	}

	void SlamVO::OnFrame(trimesh::TriMesh* mesh)
	{
		if (mesh)
			task_runner()->PostTask(FROM_HERE, base::Bind(&SlamVO::ProcessFrame, base::Unretained(this), mesh));
	}

	void SlamVO::ProcessFrame(trimesh::TriMesh* mesh)
	{
		if (m_profiler)
		{
			m_profiler->OnMesh(*mesh);
			m_profiler->OnBeforeLocate();
		}

		TriMeshPtr mesh_ptr(mesh);

		LocateData locate_data;
		locate_data.lost = true;
		locate_data.locate_type = 0;

		if (m_parameters.reader_param.use_t_vo)
		{
			m_vo_locator.Locate(mesh_ptr, locate_data);

			if (m_visual_processor)
			{
				CurrentFrameData* data = new CurrentFrameData();
				data->lost = locate_data.lost;
				data->mesh = mesh_ptr;
				m_visual_processor->OnCurrentFrame(data);
			}

			if (!locate_data.lost)
				m_vo_fusion.Fusion(mesh_ptr, locate_data.locate_type == 1);
		}
		else
		{
			m_vo_impl.ProcessOneFrame(mesh_ptr, locate_data);
		}
		if (m_profiler)
		{
			m_profiler->OnLocateResult(locate_data);
			m_profiler->OnAfterLocate();
		}
	}

	void SlamVO::SetVisualProcessor(VisualProcessor* processor)
	{
		m_visual_processor = processor;
		m_vo_impl.SetVisualProcessor(processor);
		m_vo_fusion.SetVisualProcessor(processor);
	}
	
	void SlamVO::SetLocateTracer(LocateTracer* tracer)
	{
		m_vo_impl.SetLocateTracer(tracer);
	}
	
	void SlamVO::SetICPTracer(trimesh::ProjectionICPTracer* tracer)
	{
		m_vo_impl.SetProjectionICPTracer(tracer);
	}

	void SlamVO::Clear()
	{
		if (m_parameters.reader_param.use_t_vo)
		{
			m_vo_locator.Clear();
			m_vo_fusion.Clear();
		}else
			m_vo_impl.Clear();
	}

	void SlamVO::Build(IBuildTracer& tracer)
	{
		if (m_parameters.reader_param.use_t_vo)
		{
			std::map<int, int> nooverlap;
			m_vo_fusion.Build(tracer, nooverlap);
			m_vo_locator.Build(tracer, nooverlap);
		}
		else
			m_vo_impl.Build(tracer);
	}

	void SlamVO::SetFixMode()
	{
		m_vo_locator.SetFixMode();
		m_vo_impl.SetFixMode();
	}

	void SlamVO::ProcessAddKeyFrame(TriMeshPtr mesh)
	{
		m_vo_locator.AddKeyFrame(mesh);
	}

	void SlamVO::AddKeyFrame(TriMeshPtr mesh)
	{
		task_runner()->PostTask(FROM_HERE, base::Bind(&SlamVO::ProcessAddKeyFrame, base::Unretained(this), mesh));
	}
}