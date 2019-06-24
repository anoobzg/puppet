#include "reader.h"
#include <base\bind.h>
#include "timestamp.h"
#include <boost\format.hpp>
#include "compute_boundingbox.h"

Reader::Reader()
	:base::Thread("Reader"), m_vo(nullptr), m_stop(false)
	,m_current_index(0)
{

}

Reader::~Reader()
{

}

void Reader::StartRead(const ReaderParameters& parameters)
{
	m_parameters = parameters;

	if (m_parameters.profile)
	{
		m_writer.reset(new CSVWriter());
		m_writer->PushHead("time");
	}

	m_stop = false;
	m_current_index = 0;
	bool start = Start();
	if (start) task_runner()->PostTask(FROM_HERE, base::Bind(&Reader::Read, base::Unretained(this)));
}

void Reader::StopRead()
{
	m_stop = true;
	m_vo = nullptr;
	if (m_writer) m_writer->Output(m_parameters.profile_file);

	Stop();
}

void Reader::Read()
{
	trimesh::timestamp last_time = trimesh::now();

	while (!m_stop)
	{
		trimesh::timestamp t0 = trimesh::now();
		trimesh::TriMesh* mesh = LoadOneFrame();

		if(mesh)
			trimesh::ComputeBoundingbox(mesh->vertices, mesh->bbox);
		else
			m_stop = true;

		if (m_vo) m_vo->OnFrame(mesh);

		trimesh::timestamp now_time = trimesh::now();
		
		if (m_writer) m_writer->PushData((double)(now_time - t0));
		float dt = now_time - last_time;
		last_time = now_time;
		if (dt < m_parameters.time && dt > 0.0f)
			::Sleep(DWORD(1000.0f * (m_parameters.time - dt)));
	}
}

void Reader::SetVO(VO* vo)
{
	m_vo = vo;
}

trimesh::TriMesh* Reader::LoadOneFrame()
{
	std::string name = m_parameters.directory + "//" + 
		boost::str(boost::format(m_parameters.pattern.c_str())%m_current_index);
	++m_current_index;
	return trimesh::TriMesh::read(name);
}