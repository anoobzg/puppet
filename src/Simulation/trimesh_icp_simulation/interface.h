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

struct LocateData
{
	bool lost;
	int locate_type;
};

struct RenderData
{
	TriMeshPtr mesh;
	bool lost;
	int step;
};

class VOTracer
{
public:
	virtual ~VOTracer() {}

	virtual void OnFrame(RenderData* data) = 0;
};

class ReadTracer
{
public:
	virtual ~ReadTracer() {}

	virtual void OnBeforeRead() = 0;
	virtual void OnAfterRead() = 0;
};

class VOProfiler
{
public:
	virtual ~VOProfiler(){}
	virtual void OnBeforeLocate() = 0;
	virtual void OnAfterLocate() = 0;
	virtual void OnMesh(const trimesh::TriMesh& mesh) = 0;
	virtual void OnLocateResult(const LocateData& locate_data) = 0;
};