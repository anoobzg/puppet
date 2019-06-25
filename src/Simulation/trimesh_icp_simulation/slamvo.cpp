#include "slamvo.h"
#include <base\bind.h>
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

	m_vo_impl.Setup(m_parameters);
	if (m_parameters.profile)
	{
		m_writer.reset(new CSVWriter());
		m_writer->PushHead("count");
		m_writer->PushHead("type");
		m_writer->PushHead("time");
	}

	m_state.SetFirstFrame(true);
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
	m_state.IncFrame();
	mesh->frame = m_state.Frame();
	if (m_writer) m_writer->TickStart();

	LocateData locate_data;
	locate_data.lost = false;
	locate_data.locate_type = 0;
	locate_data.frame_count = 0;
	TriMeshPtr mesh_ptr(mesh);

	LocateOneFrame(mesh_ptr, locate_data);

	if (!locate_data.lost)
		FusionFrame(mesh_ptr);

	if (m_writer)
	{
		m_writer->PushData((double)locate_data.frame_count);
		m_writer->PushData((double)locate_data.locate_type);
		m_writer->TickEnd();
	}
}

void SlamVO::LocateOneFrame(TriMeshPtr& mesh, LocateData& locate_data)
{
	locate_data.frame_count = (int)mesh->vertices.size();
	if (m_state.FirstFrame())
	{//first frame
		m_state.SetFirstFrame(false);
		std::cout << "0  --->  0" << std::endl;
	}
	else
	{
		locate_data.lost = !m_vo_impl.Frame2Frame(mesh);
	}
}

void SlamVO::FusionFrame(TriMeshPtr& mesh)
{
	m_vo_impl.SetLastMesh(mesh);
	if (m_tracer)
	{
		RenderData* render_data = new RenderData();
		render_data->mesh = mesh;
		m_tracer->OnFrame(render_data);
	}
}

void SlamVO::SetVOTracer(VOTracer* tracer)
{
	m_tracer = tracer;
}