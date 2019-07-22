#include "../interface/slam_data.h"
#include "../interface/slam_tracer.h"

#include "VoImpl.h"
#include "load_calib.h"
#include <ppl.h>
#include <set>
#include <map>
namespace esslam
{

	VOImpl::VOImpl()
		:m_visual_processor(NULL),m_locate_tracer(NULL),
		m_icp_tracer(NULL), m_profiler(NULL), m_fix_mode(false)
	{
		m_fx = 0.0f;
		m_fy = 0.0f;
		m_cx = 0.0f;
		m_cy = 0.0f;
	}

	VOImpl::~VOImpl()
	{

	}

	void VOImpl::Setup(const SlamParameters& parameters)
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

		const OctreeParameters& oct_param = parameters.octree_param;
		int cell_depth = oct_param.cell_depth;
		if (cell_depth < 2) cell_depth = 2;
		else if (cell_depth > 6) cell_depth = 6;

		float cell_resolution = 0.0f;
		if (cell_depth == 2) cell_resolution = 1.6f;
		if (cell_depth == 3) cell_resolution = 0.8f;
		if (cell_depth == 4) cell_resolution = 0.4f;
		if (cell_depth == 5) cell_resolution = 0.2f;
		if (cell_depth == 6) cell_resolution = 0.1f;
		m_octree.reset(new Octree(cell_depth, cell_resolution));

		m_layers.resize(8000000, -1);

		m_xforms.reserve(2000);
	}

	void VOImpl::SetVisualProcessor(VisualProcessor* processor)
	{
		m_visual_processor = processor;
	}

	void VOImpl::ProcessOneFrame(TriMeshPtr& mesh, LocateData& locate_data)
	{
		m_state.IncFrame();
		mesh->frame = m_state.Frame();
		m_xforms.push_back(trimesh::xform());

		if (m_fix_mode)
			return ProcessOneFrameFix(mesh, locate_data);

		if (mesh->vertices.size() < m_icp_parameters.least_frame_count)
			return;

		LocateOneFrame(mesh, locate_data);

		if (m_visual_processor)
		{
			CurrentFrameData* data = new CurrentFrameData();
			data->lost = locate_data.lost;
			data->fix = false;
			data->mesh = mesh;
			m_visual_processor->OnCurrentFrame(data);
		}

		if (!locate_data.lost)
			FusionFrame(mesh, locate_data);
	}

	void VOImpl::SetLocateTracer(LocateTracer* tracer)
	{
		m_locate_tracer = tracer;
	}

	void VOImpl::SetVOProfiler(VOProfiler* profiler)
	{
		m_profiler = profiler;
	}

	void VOImpl::LocateOneFrame(TriMeshPtr& mesh, LocateData& locate_data)
	{
		if (m_state.FirstFrame())
		{//first frame
			locate_data.lost = false;
			m_state.SetFirstFrame(false);
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

			if (!result) std::cout << mesh->frame << " lost." << std::endl;
			locate_data.lost = !result;
		}
	}

	void VOImpl::FusionFrame(TriMeshPtr& mesh, const LocateData& locate_data)
	{
		m_xforms.at(mesh->frame) = mesh->global;

		if (!m_octree->m_initialized)
			m_octree->Initialize(mesh->bbox.center());

		std::vector<int> indexes(mesh->vertices.size(), -1);
		m_octree->Insert(mesh->vertices, mesh->normals, mesh->global, indexes);

		int new_count = 0;
		size_t vsize = mesh->vertices.size();
		for (size_t i = 0; i < vsize; ++i)
		{
			int index = indexes.at(i);
			if (index >= 0 && (m_layers.at(index) > 0 || m_layers.at(index) == -1))
			{
				++new_count;
			}
		}

		bool use_as_keyframe = false;
		if ((float)new_count / (float)vsize > 0.5f)
			use_as_keyframe = true;

		int flag = use_as_keyframe ? 0 : mesh->frame;
		for (size_t i = 0; i < vsize; ++i)
		{
			int index = indexes.at(i);
			if (index >= 0 && (m_layers.at(index) != 0))
				m_layers.at(index) = flag;
		}
		
		SetLastMesh(mesh, use_as_keyframe);

		int new_num = m_octree->m_current_point_index - m_octree->m_last_point_index;
		if (use_as_keyframe) std::cout << "Use as keyframe. " << new_count << " "<<new_num<< std::endl;
		//std::cout << "Octree nodes " << m_octree->m_current_index <<
		//	" points " << m_octree->m_current_point_index << std::endl;

		if (m_visual_processor && new_num > 0)
		{
			NewAppendData* new_data = new NewAppendData();
			new_data->position.resize(new_num);
			new_data->normals.resize(new_num);
			for (int i = m_octree->m_last_point_index, j = 0; i < m_octree->m_current_point_index; ++i, ++j)
			{
				const trimesh::vec3& v = m_octree->m_trimesh.vertices.at(i);
				const trimesh::vec3& n = m_octree->m_trimesh.normals.at(i);
				new_data->position.at(j) = v;
				new_data->normals.at(j) = n;
			}
			m_visual_processor->OnAppendNewPoints(new_data);
			/*PatchRenderData* patch_data = new PatchRenderData();
			patch_data->indices.swap(indexes);
			for(int i = 0; i < 16; ++i)
				patch_data->xf[i] = (float)mesh->global[i];
			patch_data->step = locate_data.locate_type;
			size_t size = patch_data->indices.size();
			patch_data->normals.resize(size * 3);
			patch_data->points.resize(size * 3);
			for (size_t i = 0; i < size; ++i)
			{
				int index = patch_data->indices.at(i);
				if (index >= 0)
				{
					const trimesh::vec3& v = m_octree->m_trimesh.vertices.at(index);
					const trimesh::vec3& n = m_octree->m_trimesh.normals.at(index);
					float* tp = &patch_data->points.at(3 * i);
					float* tn = &patch_data->normals.at(3 * i);
					*tp++ = v[0]; *tp++ = v[1]; *tp = v[2];
					*tn++ = n[0]; *tn++ = n[1]; *tn = n[2];
				}
			}*/

			//m_visual_tracer->OnFrame(patch_data);
		}
	}

	bool VOImpl::Frame2Frame(TriMeshPtr& mesh)
	{
		//frame 2 frame
		trimesh::xform xf;
		m_icp->SetSource(mesh.get());
		m_icp->SetTarget(m_last_mesh.get());

		if (m_locate_tracer) m_locate_tracer->OnBeforeF2F();

		float err_p = 0.0f;
		if (m_use_fast_icp) err_p = m_icp->FastDo(xf);
		else err_p = m_icp->Do(xf);

		if (m_locate_tracer) m_locate_tracer->OnAfterF2F();
		if (err_p > 0.1f || err_p < 0.0f)
			return false;

		mesh->global = xf * m_last_mesh->global;
		//frame 2 model
		return Frame2Model(mesh);
	}

	bool VOImpl::Frame2Model(TriMeshPtr& mesh, bool relocate)
	{
		m_icp->SetSource(mesh.get());
		m_icp->SetTarget(&m_octree->m_trimesh);

		if (m_locate_tracer) m_locate_tracer->OnBeforeF2M();

		float error = m_icp->FMQuickDo(mesh->global, 7);

		if (m_locate_tracer) m_locate_tracer->OnAfterF2M();

		if (relocate)
		{
			if (error > 0.3f || error < 0.0f)
				return false;
		}
		else
		{
			if (error > 0.1f || error < 0.0f)
				return false;
		}
		return true;
	}

	bool VOImpl::Relocate(TriMeshPtr& mesh)
	{
		TriMeshPtr dest_mesh;
		trimesh::xform xf;
		size_t size = m_key_frames.size();

		if (size > 0)
		{
			if (m_locate_tracer) m_locate_tracer->OnBeforeRelocate();

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

			if (m_locate_tracer) m_locate_tracer->OnAfterRelocate();
		}

		if (dest_mesh)
		{
			mesh->global = xf * dest_mesh->global;

			trimesh::xform init_matrix = mesh->global;
			bool f2m = Frame2Model(mesh, true);
			if (f2m)
				std::cout << mesh->frame << " relocate success. (" << dest_mesh->frame << ") at " << size << "key frames" << std::endl;
			else
			{
				if (m_locate_tracer)
					m_locate_tracer->OnLocateFailed(mesh, init_matrix, &m_octree->m_trimesh);
			}
			return f2m;
		}
		else
		{
			return false;
		}
	}

	void VOImpl::SetLastMesh(TriMeshPtr& mesh, bool use_as_keyframe)
	{
		m_last_mesh = mesh;
		m_last_mesh->clear_grid();

		if (use_as_keyframe) m_key_frames.push_back(mesh);
	}

	void VOImpl::SetProjectionICPTracer(trimesh::ProjectionICPTracer* tracer)
	{
		m_icp_tracer = tracer;
	}

	void VOImpl::Clear()
	{
		m_state.Reset();
		m_key_frames.clear();
		m_last_mesh = NULL;
		m_octree->Clear();
		m_layers.clear();
	}

	void VOImpl::Build(IBuildTracer& tracer)
	{
		trimesh::TriMesh& mesh = m_octree->m_trimesh;
		if (mesh.vertices.size() > 0 && mesh.normals.size() > 0)
		{
			int size = (int)mesh.vertices.size();
			tracer.OnPoints(size, (float*)&mesh.vertices[0], (float*)&mesh.normals[0], 0);
		}
		else
			tracer.OnPoints(0, 0, 0, 0);

		std::vector<int> keyframes;
		int point_size = m_octree->m_current_point_index;
		std::map<int, int> nooverlap;
		for (int i = 0; i < point_size; ++i)
		{
			int index = m_layers.at(i);
			if (index > 0)
			{
				std::map<int, int>::iterator it = nooverlap.find(index);
				if (it != nooverlap.end()) ++(*it).second;
				else nooverlap.insert(std::pair<int, int>(index, 1));
			}
		}
		for (size_t i = 0; i < m_key_frames.size(); ++i)
			keyframes.push_back(m_key_frames.at(i)->frame);
		for (std::map<int, int>::iterator it = nooverlap.begin(); it != nooverlap.end(); ++it)
		{
			if ((*it).second > 3000)
				keyframes.push_back((*it).first);
			//std::cout << (*it).first << " " << (*it).second << std::endl;
		}

		tracer.OnXform(m_xforms);
		tracer.OnKeyFrames(keyframes);
	}

	void VOImpl::SetFixMode()
	{
		m_fix_mode = true;
	}

	void VOImpl::ProcessOneFrameFix(TriMeshPtr& mesh, LocateData& locate_data)
	{
		locate_data.locate_type = 0;
		locate_data.lost = false;

		if (m_visual_processor)
		{
			CurrentFrameData* data = new CurrentFrameData();
			data->lost = locate_data.lost;
			data->mesh = mesh;
			m_visual_processor->OnCurrentFrame(data);
		}

		FusionFrame(mesh, locate_data);
	}
}