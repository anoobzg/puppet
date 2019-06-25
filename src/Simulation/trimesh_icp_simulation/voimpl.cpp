#include "voimpl.h"
#include "load_calib.h"

VOImpl::VOImpl()
{
	
}

VOImpl::~VOImpl()
{

}

void VOImpl::Setup(const ICPParamters& parameters)
{
	float fx = 0.0f;
	float fy = 0.0f;
	float cx = 0.0f;
	float cy = 0.0f;
	trimesh::CameraData camera_data;
	if (!load_camera_data_from_file(parameters.calib_file, camera_data))
	{
		std::cout << "Cabli Data Error." << std::endl;
	}

	fx = camera_data.m_fx;
	fy = camera_data.m_fy;
	cx = camera_data.m_cx;
	cy = camera_data.m_cy;

	m_icp.reset(new trimesh::ProjectionICP(fx, fy, cx, cy));
}

bool VOImpl::Frame2Frame(TriMeshPtr& mesh, trimesh::xform& xf)
{
	m_icp->SetSource(mesh.get());
	m_icp->SetTarget(m_last_mesh.get());
	float err_p = m_icp->Do(xf);

	if (err_p >= 0.1f || err_p < 0.0f)
	{
		m_xf = trimesh::xform::identity();
		xf = m_xf;
		return false;
	}
	
	m_xf = m_xf * xf;
	xf = m_xf;
	return true;
}

void VOImpl::SetLastMesh(TriMeshPtr& mesh)
{
	m_last_mesh = mesh;
}