#include "slamvo.h"
#include <base\bind.h>
#include "load_calib.h"
#include "timestamp.h"

SlamVO::SlamVO()
	:base::Thread("SlamVO"), m_tracer(NULL)
{

}

SlamVO::~SlamVO()
{

}

void SlamVO::StartVO(const ICPParamters& parameters)
{
	m_parameters = parameters;

	trimesh::CameraData camera_data;
	if (!load_camera_data_from_file(parameters.calib_file, camera_data))
	{
		std::cout << "Cabli Data Error." << std::endl;
	}

	float fx = camera_data.m_fx;
	float fy = camera_data.m_fy;
	float cx = camera_data.m_cx;
	float cy = camera_data.m_cy;
	m_icp.reset(new trimesh::ProjectionICP(fx, fy, cx, cy));

	if (m_parameters.profile)
	{
		m_writer.reset(new CSVWriter());
		m_writer->PushHead("count");
		m_writer->PushHead("type");
		m_writer->PushHead("time");
	}
	bool start = Start();
}

void SlamVO::StopVO()
{
	if (m_writer)
		m_writer->Output(m_parameters.profile_file);
	Stop();
	m_tracer = NULL;
}

void SlamVO::OnFrame(trimesh::TriMesh* mesh)
{
	if(mesh)
		task_runner()->PostTask(FROM_HERE, base::Bind(&SlamVO::ProcessFrame, base::Unretained(this), mesh));
}

void SlamVO::ProcessFrame(trimesh::TriMesh* mesh)
{
	trimesh::timestamp t0 = trimesh::now();
	int type = 0;

	RenderData* data = new RenderData();
	if (m_last_mesh)
	{
		trimesh::xform xf;
		m_icp->SetSource(mesh);
		m_icp->SetTarget(m_last_mesh.get());
		float err_p = m_icp->Do(xf);

		if (err_p < 0.0f) m_xf = trimesh::xform::identity();
		else m_xf = m_xf * xf;
	}else
	{
		m_xf = trimesh::xform::identity();
	}

	m_last_mesh.reset(mesh);
	data->mesh = m_last_mesh;
	data->xf = m_xf;

	trimesh::timestamp t1 = trimesh::now();
	if (m_writer)
	{
		m_writer->PushData((double)mesh->vertices.size());
		m_writer->PushData((double)type);
		m_writer->PushData((double)(t1 - t0));
	}
	if (m_tracer) m_tracer->OnFrame(data);
}

void SlamVO::SetVOTracer(VOTracer* tracer)
{
	m_tracer = tracer;
}