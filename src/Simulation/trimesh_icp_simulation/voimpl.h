#pragma once
#include "TriMesh.h"
#include "Xform.h"
#include "interface.h"
#include "projectionicp.h"
#include "csvwriter.h"
#include "vostate.h"

class VOImpl
{
public:
	VOImpl();
	~VOImpl();

	void Setup(const ICPParamters& parameters);
	void SetVOTracer(VOTracer* tracer);

	void ProcessOneFrame(TriMeshPtr& mesh, LocateData& locate_data);

protected:
	void LocateOneFrame(TriMeshPtr& mesh, LocateData& locate_data);
	void FusionFrame(TriMeshPtr& mesh);

	bool Frame2Frame(TriMeshPtr& mesh);
	void SetLastMesh(TriMeshPtr& mesh);
protected:
	VOTracer* m_tracer;
	TriMeshPtr m_last_mesh;
	std::unique_ptr<trimesh::ProjectionICP> m_icp;

	std::vector<TriMeshPtr> m_key_frames;

	float m_fx;
	float m_fy;
	float m_cx;
	float m_cy;

	VOState m_state;
};