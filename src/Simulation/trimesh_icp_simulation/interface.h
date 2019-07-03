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

struct PatchRenderData
{
	std::vector<int> indices;
	std::vector<trimesh::vec3> points;
	std::vector<trimesh::vec3> normals;
	trimesh::xform xf;
	bool lost;
	int step;
};

struct KeyFrameData
{
	bool locate;
	bool use_as_key_frame;
	TriMeshPtr mesh;
};

class VOTracer
{
public:
	virtual ~VOTracer() {}

	virtual void OnFrame(RenderData* data) = 0;
	virtual void OnFrame(PatchRenderData* data) = 0;
};

class KeyFrameTracer
{
public:
	virtual ~KeyFrameTracer() {}

	virtual void OnKeyFrame(KeyFrameData* data) = 0;
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

class LocateTracer
{
public:
	virtual ~LocateTracer() {};
	virtual void OnFF() = 0;
	virtual void OnBeforeF2F() = 0;
	virtual void OnAfterF2F() = 0;
	virtual void OnFM() = 0;
	virtual void OnBeforeF2M() = 0;
	virtual void OnAfterF2M() = 0;
	virtual void OnRelocate() = 0;
	virtual void OnBeforeRelocate() = 0;
	virtual void OnAfterRelocate() = 0;

	virtual void OnLocateFailed(TriMeshPtr mesh, const trimesh::xform& init_matrix, trimesh::TriMesh* all_mesh) = 0;
};