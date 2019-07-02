#pragma once
#include "TriMesh.h"
#include "Xform.h"
#include "interface.h"
#include "projectionicp.h"
#include "csvwriter.h"
#include "vostate.h"
#include "octree.h"

class VOImpl
{
public:
	VOImpl();
	~VOImpl();

	void Setup(const SlamParameters& parameters);
	void SetVOTracer(VOTracer* tracer);
	void SetLocateTracer(LocateTracer* tracer);

	void ProcessOneFrame(TriMeshPtr& mesh, LocateData& locate_data);
protected:
	void LocateOneFrame(TriMeshPtr& mesh, LocateData& locate_data);
	void FusionFrame(TriMeshPtr& mesh, const LocateData& locate_data);

	bool Frame2Frame(TriMeshPtr& mesh);
	bool Frame2Model(TriMeshPtr& mesh);
	bool Relocate(TriMeshPtr& mesh);
	void SetLastMesh(TriMeshPtr& mesh, bool use_as_keyframe);
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

	std::unique_ptr<Octree> m_octree;
	std::vector<int> m_layers;

	bool m_use_fast_icp;
	LocateTracer* m_locate_tracer;
};