#pragma once
#include <base\threading\thread.h>
#include "Octree.h"
#include "SlamParameters.h"
#include "InnerInterface.h"
#include "projectionicp.h"
#include "../interface/slam_tracer.h"
#include <map>

namespace esslam
{
	class ESSLAM_API VOFusion : public base::Thread
	{
	public:
		VOFusion();
		virtual ~VOFusion();

		void Clear();
		void Build(IBuildTracer& tracer, std::map<int, int>& nooverlap);
		void Setup(const SlamParameters& parameters);
		void StartFusion();
		void StopFusion();

		void Fusion(TriMeshPtr mesh, bool relocate);
		void SetVisualProcessor(VisualProcessor* processor);
		void SetKeyFrameAdder(KeyFrameAdder* adder);
	private:
		void DoFusion(TriMeshPtr mesh, bool relocate);
		bool Frame2Model(TriMeshPtr mesh, bool relocate);
	protected:
		std::unique_ptr<Octree> m_octree;
		std::vector<int> m_layers;

		ICPParamters m_icp_parameters;
		std::unique_ptr<trimesh::ProjectionICP> m_icp;

		float m_fx;
		float m_fy;
		float m_cx;
		float m_cy;

		VisualProcessor* m_visual_processor;
		KeyFrameAdder* m_adder;

		std::vector<trimesh::xform> m_xforms;
	};
}