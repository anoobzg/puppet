#pragma once
#include "TriMesh.h"
#include "Xform.h"
#include <memory>

namespace esslam
{

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

	struct CurrentFrameData
	{
		bool lost;
		TriMeshPtr mesh;
	};

	struct NewData
	{
		std::vector<trimesh::vec3> positions;
		std::vector<trimesh::vec3> normals;
		std::vector<unsigned char> colors;
	};

	class VOProfiler
	{
	public:
		virtual ~VOProfiler() {}
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

	struct NewAppendData;
	class VisualProcessor
	{
	public:
		virtual ~VisualProcessor() {}
		virtual void OnCurrentFrame(CurrentFrameData* data) = 0;
		virtual void OnAppendNewPoints(NewAppendData* data) = 0;
	};
}