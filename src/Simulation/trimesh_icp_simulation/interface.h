#pragma once
#include "TriMesh.h"
#include "Xform.h"

#include "slamparameters.h"
#include <memory>

typedef std::shared_ptr<trimesh::TriMesh> TriMeshPtr;
class VO
{
public:
	virtual ~VO() {}

	virtual void OnFrame(trimesh::TriMesh* mesh) = 0;
};

struct RenderData
{
	TriMeshPtr mesh;
	trimesh::xform xf;
};

class VOTracer
{
public:
	virtual ~VOTracer() {}

	virtual void OnFrame(RenderData* data) = 0;
};