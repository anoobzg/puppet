#pragma once
#include "../interface/slam_interface.h"
#include "InnerInterface.h"
#include "projectionicp.h"
#include "VoState.h"
#include "SlamParameters.h"
#include <map>

namespace esslam
{
	class VOLocator
	{
	public:
		VOLocator();
		~VOLocator();

		void Setup(const SlamParameters& parameters);
		void Locate(TriMeshPtr& mesh, LocateData& vo_locate_data);
		void SetFixMode();
		void AddKeyFrame(TriMeshPtr& mesh);
		void Clear();
		void Build(IBuildTracer& tracer, std::map<int, int>& nooverlap);
	protected:
		void LocateOneFrame(TriMeshPtr& mesh, LocateData& locate_data);
		void ProcessOneFrameFix(TriMeshPtr& mesh, LocateData& locate_data);

		bool Frame2Frame(TriMeshPtr& mesh);
		bool Relocate(TriMeshPtr& mesh);
	protected:
		ICPParamters m_icp_parameters;
		bool m_use_fast_icp;

		TriMeshPtr m_last_mesh;
		std::unique_ptr<trimesh::ProjectionICP> m_icp;

		std::vector<TriMeshPtr> m_key_frames;

		float m_fx;
		float m_fy;
		float m_cx;
		float m_cy;

		VOState m_state;

		bool m_fix_mode;
	};

}