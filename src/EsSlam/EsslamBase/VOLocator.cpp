#include "VOLocator.h"
#include "load_calib.h"
#include <ppl.h>
#include "../interface/slam_tracer.h"

namespace esslam
{
	VOLocator::VOLocator()
		:m_fix_mode(false), m_use_fast_icp(false)
	{
		m_fx = 0.0f;
		m_fy = 0.0f;
		m_cx = 0.0f;
		m_cy = 0.0f;
	}

	VOLocator::~VOLocator()
	{

	}

	void VOLocator::Setup(const SlamParameters& parameters)
	{
		m_icp_parameters = parameters.icp_param;
		m_use_fast_icp = m_icp_parameters.use_fast == 1;
		trimesh::CameraData camera_data;
		if (!load_camera_data_from_file(m_icp_parameters.calib_file, camera_data))
		{
			std::cout << "Cabli Data Error." << std::endl;
		}

		m_fx = camera_data.m_fx;
		m_fy = camera_data.m_fy;
		m_cx = camera_data.m_cx;
		m_cy = camera_data.m_cy;

		std::cout << "fx fy cx cy " << m_fx << " " << m_fy << " " << m_cx << " " << m_cy << std::endl;
		m_icp.reset(new trimesh::ProjectionICP(m_fx, m_fy, m_cx, m_cy));
	}

	void VOLocator::Locate(TriMeshPtr& mesh, LocateData& locate_data)
	{
		m_state.IncFrame();
		mesh->frame = m_state.Frame();

		if (m_fix_mode)
			return ProcessOneFrameFix(mesh, locate_data);

		if (mesh->vertices.size() < m_icp_parameters.least_frame_count)
			return;

		LocateOneFrame(mesh, locate_data);
	}

	void VOLocator::LocateOneFrame(TriMeshPtr& mesh, LocateData& locate_data)
	{
		if (m_state.FirstFrame())
		{//first frame
			locate_data.lost = false;
			m_state.SetFirstFrame(false);
			m_last_mesh = mesh;
		}
		else
		{
			bool result = Frame2Frame(mesh);
			if (result)
			{
				locate_data.locate_type = 0;
				std::cout << mesh->frame << " continous success." << std::endl;
			}
			else
			{
				locate_data.locate_type = 1;
				result = Relocate(mesh);
			}

			if (result) m_last_mesh = mesh;
			if (!result) std::cout << mesh->frame << " lost." << std::endl;
			locate_data.lost = !result;
		}
	}

	void VOLocator::ProcessOneFrameFix(TriMeshPtr& mesh, LocateData& locate_data)
	{
		locate_data.locate_type = 0;
		locate_data.lost = false;
	}

	void VOLocator::SetFixMode()
	{
		m_fix_mode = true;
	}

	bool VOLocator::Frame2Frame(TriMeshPtr& mesh)
	{
		//frame 2 frame
		trimesh::xform xf;
		m_icp->SetSource(mesh.get());
		m_icp->SetTarget(m_last_mesh.get());

		float err_p = 0.0f;
		if (m_use_fast_icp) err_p = m_icp->FastDo(xf);
		else err_p = m_icp->Do(xf);

		if (err_p > 0.1f || err_p < 0.0f)
			return false;

		mesh->global = xf * m_last_mesh->global;
		return true;
	}

	void VOLocator::AddKeyFrame(TriMeshPtr& mesh)
	{
		m_key_frames.push_back(mesh);
	}

	void VOLocator::Clear()
	{
		m_state.Reset();
		m_key_frames.clear();
		m_last_mesh = NULL;
	}

	void VOLocator::Build(IBuildTracer& tracer, std::map<int, int>& nooverlap)
	{
		std::vector<int> keyframes;
		for (size_t i = 0; i < m_key_frames.size(); ++i)
			keyframes.push_back(m_key_frames.at(i)->frame);
		for (std::map<int, int>::iterator it = nooverlap.begin(); it != nooverlap.end(); ++it)
		{
			if ((*it).second > 3000)
				keyframes.push_back((*it).first);
			//std::cout << (*it).first << " " << (*it).second << std::endl;
		}

		tracer.OnKeyFrames(keyframes);
	}

	bool VOLocator::Relocate(TriMeshPtr& mesh)
	{
		TriMeshPtr dest_mesh;
		trimesh::xform xf;
		size_t size = m_key_frames.size();

		if (size > 0)
		{
			std::vector<float> errors(size, -1.0f);
			std::vector<trimesh::xform> matrixes(size);
			Concurrency::parallel_for<size_t>(0, size, [this, &mesh, &matrixes, &errors](size_t i) {
				trimesh::ProjectionICP icp(m_fx, m_fy, m_cx, m_cy);
				icp.SetSource(mesh.get());
				icp.SetTarget(m_key_frames.at(i).get());
				//errors.at(i) = icp.DefaultTimesDo(matrixes.at(i), 10);
				errors.at(i) = icp.Do(matrixes.at(i));
			});

			float min_error = FLT_MAX;
			int index = -1;
			for (size_t i = 0; i < size; ++i)
			{
				float e = errors.at(i);
				if (e <= 0.1f && e >= 0.0f && e < min_error)
				{
					min_error = e;
					index = (int)i;
				}
			}

			if (index >= 0 && index < (int)size)
			{
				dest_mesh = m_key_frames.at(index);
				xf = matrixes.at(index);
			}
		}

		if (dest_mesh)
		{
			mesh->global = xf * dest_mesh->global;

			trimesh::xform init_matrix = mesh->global;
			std::cout << mesh->frame << " relocate success. (" << dest_mesh->frame << ") at " << size << "key frames" << std::endl;

			return true;
		}
		else
		{
			return false;
		}
	}
}