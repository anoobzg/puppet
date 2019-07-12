#include "Reader.h"
#include <base\bind.h>
#include <boost\format.hpp>
#include "compute_boundingbox.h"
#include "timestamp.h"
#include "..\interface\slam_tracer.h"

namespace esslam
{
	Reader::Reader()
		:base::Thread("Reader"), m_stop(false)
		, m_current_index(0), m_vo(NULL), m_tracer(NULL)
	{

	}

	Reader::~Reader()
	{

	}

	void Reader::StartRead(const ReaderParameters& parameters)
	{
		m_parameters = parameters;

		m_stop = false;
		m_current_index = 0;
		bool start = Start();
		if (start) task_runner()->PostTask(FROM_HERE, base::Bind(&Reader::Read, base::Unretained(this)));
	}

	void Reader::StopRead()
	{
		m_stop = true;

		Stop();

		m_vo = NULL;
	}

	void Reader::SetVO(VO* vo)
	{
		m_vo = vo;
	}

	void Reader::SetReadTracer(IReadTracer* tracer)
	{
		m_tracer = tracer;
	}

	void Reader::Read()
	{
		trimesh::timestamp last_time = trimesh::now();

		while (!m_stop)
		{
			trimesh::TriMesh* mesh = LoadOneFrame();

			if (mesh)
				trimesh::ComputeBoundingbox(mesh->vertices, mesh->bbox);
			else
				m_stop = true;

			if (m_vo) m_vo->OnFrame(mesh);

			trimesh::timestamp now_time = trimesh::now();
			float dt = now_time - last_time;
			last_time = now_time;
			if (dt < m_parameters.time && dt > 0.0f)
				::Sleep(DWORD(1000.0f * (m_parameters.time - dt)));
		}

		if (m_tracer) m_tracer->OnFinished();
	}

	trimesh::TriMesh* Reader::LoadOneFrame()
	{
		std::string name = m_parameters.directory + "//" +
			boost::str(boost::format(m_parameters.pattern.c_str()) % m_current_index);
		++m_current_index;
		return trimesh::TriMesh::read(name);
	}

}