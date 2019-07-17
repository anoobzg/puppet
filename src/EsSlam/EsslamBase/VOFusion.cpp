#include "VOFusion.h"
#include <base\bind.h>
#include "load_calib.h"
#include "../interface/slam_data.h"

namespace esslam
{
	VOFusion::VOFusion()
		:base::Thread("VOFusion"), m_visual_processor(NULL)
	{
		m_xforms.resize(2000);
	}

	VOFusion::~VOFusion()
	{

	}

	void VOFusion::Build(IBuildTracer& tracer, std::map<int, int>& nooverlap)
	{
		trimesh::TriMesh& mesh = m_octree->m_trimesh;
		if (mesh.vertices.size() > 0 && mesh.normals.size() > 0)
		{
			int size = (int)mesh.vertices.size();
			tracer.OnPoints(size, (float*)&mesh.vertices[0], (float*)&mesh.normals[0], 0);
		}
		else
			tracer.OnPoints(0, 0, 0, 0);

		int point_size = m_octree->m_current_point_index;
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

		tracer.OnXform(m_xforms);
	}

	void VOFusion::Clear()
	{
		m_octree->Clear();
		m_layers.clear();
	}

	void VOFusion::Setup(const SlamParameters& parameters)
	{
		m_icp_parameters = parameters.icp_param;
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
	}

	void VOFusion::StartFusion()
	{
		Start();
	}

	void VOFusion::StopFusion()
	{
		Stop();
	}

	void VOFusion::SetKeyFrameAdder(KeyFrameAdder* adder)
	{
		m_adder = adder;
	}

	void VOFusion::Fusion(TriMeshPtr mesh, bool relocate)
	{
		task_runner()->PostTask(FROM_HERE, base::Bind(&VOFusion::DoFusion, base::Unretained(this), mesh, relocate));
	}

	bool VOFusion::Frame2Model(TriMeshPtr mesh, bool relocate)
	{
		m_icp->SetSource(mesh.get());
		m_icp->SetTarget(&m_octree->m_trimesh);

		float error = m_icp->FMQuickDo(mesh->global, 7);

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

	void VOFusion::SetVisualProcessor(VisualProcessor* processor)
	{
		m_visual_processor = processor;
	}

	void VOFusion::DoFusion(TriMeshPtr mesh, bool relocate)
	{
		if (m_octree->m_initialized)
		{
			bool fm = Frame2Model(mesh, relocate);

			if (!fm)
			{
				std::cout << "Frame 2 Model Failed." << std::endl;
				return;
			}
		}

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

		if (use_as_keyframe && m_adder) m_adder->AddKeyFrame(mesh);

		int new_num = m_octree->m_current_point_index - m_octree->m_last_point_index;
		//if (use_as_keyframe) std::cout << "Use as keyframe. " << new_count << " " << new_num << std::endl;
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
		}
	}
}