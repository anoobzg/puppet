#pragma once
#include "interface.h"
#include <base\threading\thread.h>
#include "csvwriter.h"
#include "vostate.h"
#include "voimpl.h"

class SlamVO : public base::Thread , public VO
{
public:
	SlamVO();
	virtual ~SlamVO();

	void StartVO(const ICPParamters& parameters);
	void StopVO();

	void OnFrame(trimesh::TriMesh* mesh);
	void SetVOTracer(VOTracer* tracer);
protected:
	void ProcessFrame(trimesh::TriMesh* mesh);
	void LocateOneFrame(TriMeshPtr& mesh, LocateData& locate_data);
	void FusionFrame(TriMeshPtr& mesh);
private:
	VOTracer* m_tracer;
	ICPParamters m_parameters;
	std::unique_ptr<CSVWriter> m_writer;
	VOState m_state;
	VOImpl m_vo_impl;
};