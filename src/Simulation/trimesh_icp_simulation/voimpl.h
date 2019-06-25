#pragma once
#include "TriMesh.h"
#include "Xform.h"
#include "interface.h"
#include "projectionicp.h"

class VOImpl
{
public:
	VOImpl();
	~VOImpl();

	void Setup(const ICPParamters& parameters);
	bool Frame2Frame(TriMeshPtr& mesh);
	void SetLastMesh(TriMeshPtr& mesh);
protected:
	TriMeshPtr m_last_mesh;
	std::unique_ptr<trimesh::ProjectionICP> m_icp;

	std::vector<TriMeshPtr> m_key_frames;

	float m_fx;
	float m_fy;
	float m_cx;
	float m_cy;
};