#include "voimpl.h"
#include "load_calib.h"
#include <ppl.h>

VOImpl::VOImpl()
{
	m_fx = 0.0f;
	m_fy = 0.0f;
	m_cx = 0.0f;
	m_cy = 0.0f;
}

VOImpl::~VOImpl()
{

}

void VOImpl::Setup(const ICPParamters& parameters)
{

	trimesh::CameraData camera_data;
	if (!load_camera_data_from_file(parameters.calib_file, camera_data))
	{
		std::cout << "Cabli Data Error." << std::endl;
	}

	m_fx = camera_data.m_fx;
	m_fy = camera_data.m_fy;
	m_cx = camera_data.m_cx;
	m_cy = camera_data.m_cy;

	m_icp.reset(new trimesh::ProjectionICP(m_fx, m_fy, m_cx, m_cy));
}

void VOImpl::SetVOTracer(VOTracer* tracer)
{
	m_tracer = tracer;
}

void VOImpl::ProcessOneFrame(TriMeshPtr& mesh, LocateData& locate_data)
{
	m_state.IncFrame();
	mesh->frame = m_state.Frame();

	LocateOneFrame(mesh, locate_data);

	if (!locate_data.lost)
		FusionFrame(mesh);
}

void VOImpl::LocateOneFrame(TriMeshPtr& mesh, LocateData& locate_data)
{
	if (m_state.FirstFrame())
	{//first frame
		locate_data.lost = false;
		m_state.SetFirstFrame(false);
		std::cout << "0  --->  0" << std::endl;
	}
	else
	{
		locate_data.lost = !Frame2Frame(mesh);
	}
}

void VOImpl::FusionFrame(TriMeshPtr& mesh)
{
	SetLastMesh(mesh);
	if (m_tracer)
	{
		RenderData* render_data = new RenderData();
		render_data->mesh = mesh;
		m_tracer->OnFrame(render_data);
	}
}

bool VOImpl::Frame2Frame(TriMeshPtr& mesh)
{
	TriMeshPtr dest_mesh;
	trimesh::xform xf;

	m_icp->SetSource(mesh.get());
	m_icp->SetTarget(m_last_mesh.get());
	float err_p = m_icp->Do(xf);

	if (err_p <= 0.1f && err_p >= 0.0f)
	{
		dest_mesh = m_last_mesh;
	}
	else
	{
		size_t size = m_key_frames.size();
		if (size > 0)
		{
			std::vector<float> errors(size, -1.0f);
			std::vector<trimesh::xform> matrixes(size);
			Concurrency::parallel_for<size_t>(0, size, [this, &mesh, &matrixes, &errors](size_t i) {
				trimesh::ProjectionICP icp(m_fx, m_fy, m_cx, m_cy);
				icp.SetSource(mesh.get());
				icp.SetTarget(m_key_frames.at(i).get());
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
	}

	if (dest_mesh)
	{
		mesh->global = xf * dest_mesh->global;
		std::cout << mesh->frame << "  --->  " <<dest_mesh->frame<< std::endl;
		return true;
	}
	else
	{
		std::cout << mesh->frame<<"  --->  -1" << std::endl;
		return false;
	}
}

void VOImpl::SetLastMesh(TriMeshPtr& mesh)
{
	m_last_mesh = mesh;
	m_last_mesh->clear_grid();

	static int count = 0;
	if ((count + 1) % 3 == 0) m_key_frames.push_back(mesh);
	++count;
}