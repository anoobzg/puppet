#include "../interface/slam_data.h"
#include "Esslam.h"
#include "Reader.h"
#include "SlamVo.h"
#include "DebugCenter.h"
#include "RenderProxy.h"
#include "compute_boundingbox.h"
#include "Profiler.h"

namespace esslam
{
	Esslam::Esslam()
		:m_scan_type(e_load_from_file), m_running(false)
	{
		m_reader.reset(new Reader());
		m_vo.reset(new SlamVO());
		m_render_proxy.reset(new RenderProxy());
	}

	Esslam::~Esslam()
	{

	}

	void Esslam::SetupParameters(const SetupParameter& parameter)
	{
		m_state_lock.Acquire();
		if (!m_running)
		{
			m_scan_type = parameter.type;
			const std::string& file = parameter.default_config;
			m_parameters.LoadFromFile(file);
			if (m_scan_type != e_load_from_file)
			{
				m_parameters.icp_param.calib_file = parameter.calib_file;
			}
			if (m_scan_type == e_fix)
			{
				m_vo->SetFixMode();
			}
		}
		m_state_lock.Release();
	}

	void Esslam::SetOSGTracer(IOSGTracer* tracer)
	{

	}

	void Esslam::SetVisualTracer(IVisualTracer* tracer)
	{
		m_state_lock.Acquire();
		if (!m_running) m_render_proxy->SetVisualTracer(tracer);
		m_state_lock.Release();
	}

	void Esslam::SetReadTracer(IReadTracer* tracer)
	{
		m_state_lock.Acquire();
		if (!m_running) m_reader->SetReadTracer(tracer);
		m_state_lock.Release();
	}

	void Esslam::Start()
	{
		m_state_lock.Acquire();
		m_running = true;
		m_state_lock.Release();

		PROFILE_START

		StartInner();
	}

	void Esslam::StartInner()
	{
		const DebugParameters& debug_param = m_parameters.debug_param;
		if (debug_param.debug)
		{
			m_debug_center.reset(new DebugCenter());
			m_debug_center->SetParameters(debug_param);

			if(debug_param.profile_detail) m_vo->SetVOProfiler(m_debug_center->GetVOProfiler());
			m_vo->SetLocateTracer(m_debug_center->GetLocateTracer());
			m_vo->SetICPTracer(m_debug_center->GetProjectionICPTracer());
		}

		m_render_proxy->StartProcess();
		m_vo->SetVisualProcessor(m_render_proxy.get());
		m_vo->StartVO(m_parameters);
		if (m_scan_type == e_load_from_file)
		{
			m_reader->SetVO(m_vo.get());
			m_reader->StartRead(m_parameters.reader_param);
		}
	}

	void Esslam::Stop()
	{
		m_state_lock.Acquire();
		m_running = false;
		m_state_lock.Release();

		if (m_scan_type == e_load_from_file)
			m_reader->StopRead();

		m_vo->StopVO();
		m_render_proxy->StopProcess();
		if (m_debug_center)
		{
			m_debug_center->Save();
			m_debug_center.reset();
		}

		PROFILE_OUTPUT
	}

	void Esslam::Clear()
	{
		m_vo->Clear();
	}

	void Esslam::SetImageData(HandleScanImageData* data)
	{

	}

	void Esslam::SetModelData(BuildModelData* data)
	{
		m_state_lock.Acquire();
		
		int size = (int)data->points.size();
		if(m_running && size > 0 && data->width > 0 && data->height > 0)
		{
			trimesh::TriMesh* mesh = new trimesh::TriMesh();
			mesh->vertices.swap(data->points);
			mesh->normals.swap(data->normals);
			mesh->grid.swap(data->grid);
			mesh->grid_width = data->width;
			mesh->grid_height = data->height;

			trimesh::ComputeBoundingbox(mesh->vertices, mesh->bbox);
			m_vo->OnFrame(mesh);
		}
		else
			std::cout << "GONE" << std::endl;
		m_state_lock.Release();

		delete data;
	}

	HHScanData* Esslam::GetScanData()
	{
		return NULL;
	}
	void Esslam::SetScanData(HHScanData* data)
	{
		m_state_lock.Acquire();
		if (m_running)
		{

		}
		else
		{

		}
		m_state_lock.Release();
	}

	//void Esslam::SetData(int size, float* position, float* normal,
	//	int grid_width, int grid_height, int* grid_indices)
	//{
	//	m_state_lock.Acquire();
	//
	//	if (m_running && size > 0 && grid_width > 0 && grid_height > 0)
	//	{
	//		trimesh::TriMesh* mesh = new trimesh::TriMesh();
	//		mesh->vertices.resize(size);
	//		mesh->normals.resize(size);
	//		mesh->grid_width = grid_width;
	//		mesh->grid_height = grid_height;
	//		mesh->grid.resize(grid_width * grid_height);
	//		memcpy(&mesh->vertices[0], position, 3 * sizeof(float) * size);
	//		memcpy(&mesh->normals[0], normal, 3 * sizeof(float) * size);
	//		memcpy(&mesh->grid[0], grid_indices, grid_width * grid_height * sizeof(int));
	//
	//		trimesh::ComputeBoundingbox(mesh->vertices, mesh->bbox);
	//		m_vo->OnFrame(mesh);
	//	}
	//	m_state_lock.Release();
	//}

	void Esslam::Build(IBuildTracer* tracer)
	{
		if (tracer) m_vo->Build(*tracer);
	}
}