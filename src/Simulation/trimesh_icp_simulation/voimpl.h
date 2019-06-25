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
	bool Frame2Frame(TriMeshPtr& mesh, trimesh::xform& xf);
	void SetLastMesh(TriMeshPtr& mesh);
protected:
	TriMeshPtr m_last_mesh;
	trimesh::xform m_xf;
	std::unique_ptr<trimesh::ProjectionICP> m_icp;
};