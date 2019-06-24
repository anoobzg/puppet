#include "cmdicp.h"
#include "ICP.h"
#include "projectionicp.h"
#include "timestamp.h"
#include <assert.h>
#include "csvwriter.h"

class Tracer : public trimesh::ProjectionICPTracer
{
public:
	Tracer() {}
	~Tracer() {}

	void OnError(float error)
	{
		std::cout << "error " << error << std::endl;
	}
};

void cmd_test_icp(trimesh::TriMesh* source, trimesh::TriMesh* target,
	const trimesh::CameraData& data)
{
	TNT::Matrix<float> e_left;
	trimesh::xform xf1;
	trimesh::xform xf2;
	float m_fx = data.m_fx;
	float m_fy = data.m_fy;
	float m_cx = data.m_cx;
	float m_cy = data.m_cy;

	trimesh::timestamp t0 = trimesh::now();
	float err = trimesh::ICP(target, source, xf1, xf2, m_fx, m_fy, m_cx, m_cy, 0.0f, 0, e_left);
	trimesh::timestamp t1 = trimesh::now();

	trimesh::xform xf2_p;

	trimesh::timestamp t2 = trimesh::now();
	Tracer tracer;
	trimesh::ProjectionICP icp(m_fx, m_fy, m_cx, m_cy);
	icp.SetSource(source);
	icp.SetTarget(target);
	icp.SetTracer(&tracer);
	float err_p = icp.Do(xf2_p);
	trimesh::timestamp t3 = trimesh::now();

	assert(err_p == err);
	std::cout << "Normal ICP: " <<err<<" | "<<(t1 - t0) << std::endl;
	std::cout << "Test ICP: " <<err_p<<" | "<<(t3 - t2) << std::endl;
}

class AnalysisTracer : public trimesh::ProjectionICPTracer
{
public:
	AnalysisTracer():m_writer(nullptr) {}
	~AnalysisTracer() {}

	void SetWriter(CSVWriter* writer)
	{
		m_writer = writer;
	}

	void OnError(float error)
	{
		m_writer->PushData((double)error);
		//std::cout << "error " << error << std::endl;
	}

	CSVWriter* m_writer;
};

void cmd_analysis_icp(trimesh::TriMesh* source, trimesh::TriMesh* target, const trimesh::CameraData& data,
	const std::string& error_csv_file, const std::string& time_csv_file)
{
	float m_fx = data.m_fx;
	float m_fy = data.m_fy;
	float m_cx = data.m_cx;
	float m_cy = data.m_cy;

	CSVWriter writer;
	{//error
		writer.PushHead("error");
		AnalysisTracer tracer;
		tracer.SetWriter(&writer);
		trimesh::xform xf2_p;
		trimesh::ProjectionICP icp(m_fx, m_fy, m_cx, m_cy);
		icp.SetSource(source);
		icp.SetTarget(target);
		icp.SetTracer(&tracer);
		icp.Do(xf2_p);
		writer.Output(error_csv_file);
	}
	writer.Clear();
	{//time
		writer.PushHead("source_count");
		writer.PushHead("target_count");
		writer.PushHead("time");
		size_t source_size = source->vertices.size();
		size_t target_size = target->vertices.size();
		for (int i = 0; i < 100; ++i)
		{
			writer.PushData((double)source_size);
			writer.PushData((double)target_size);
			trimesh::timestamp t1 = trimesh::now();
			trimesh::xform xf2_p;
			trimesh::ProjectionICP icp(m_fx, m_fy, m_cx, m_cy);
			icp.SetSource(source);
			icp.SetTarget(target);
			icp.Do(xf2_p);
			trimesh::timestamp t2 = trimesh::now();
			writer.PushData((double)(t2 - t1));
		}

		writer.Output(time_csv_file);
	}
	
}