#pragma once
#include "TriMesh.h"
#include "Xform.h"

namespace trimesh
{
	struct PtPair
	{
		trimesh::vec p1, n1, p2, n2;
		PtPair(const trimesh::point &p1_, const trimesh::vec &n1_,
			const trimesh::point &p2_, const trimesh::vec &n2_) :
			p1(p1_), n1(n1_), p2(p2_), n2(n2_)
		{}
		PtPair()
		{}
	};

	enum ICP_iter_type
	{
		ICP_POINT_TO_POINT, ICP_POINT_TO_PLANE
	};

	class ProjectionICPTracer
	{
	public:
		virtual ~ProjectionICPTracer() {}
		virtual void OnICPStart() {}
		virtual void OnICPStop() {}
		virtual void OnStepStart() {}
		virtual void OnStepStop() {}
		virtual void OnError(float error) {}
		virtual void OnPreStepCorrespondences(const std::vector<PtPair>& correspondences) {}
		virtual void OnStepCorrespondences(const std::vector<PtPair>& correspondences) {}
		virtual void OnMatrix(const trimesh::xform& xf) {}
	};
}