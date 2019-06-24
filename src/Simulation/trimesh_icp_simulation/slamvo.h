#pragma once
#include "interface.h"
#include <base\threading\thread.h>
#include "projectionicp.h"
#include "csvwriter.h"

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

private:
	VOTracer* m_tracer;

	std::unique_ptr<trimesh::ProjectionICP> m_icp;
	
	TriMeshPtr m_last_mesh;
	trimesh::xform m_xf;

	ICPParamters m_parameters;

	std::unique_ptr<CSVWriter> m_writer;
};