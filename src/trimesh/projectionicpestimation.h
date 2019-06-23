#pragma once
#include "projectionicpheader.h"

namespace trimesh
{
	class ProjectionICPEstimation
	{
	public:
		ProjectionICPEstimation();
		~ProjectionICPEstimation();

		inline void SetDistThreshMult(float value)
		{
			m_dist_thresh_mult = value;
		}

		inline void SetNormDotThresh(float value)
		{
			m_normdot_thresh = value;
		}

		inline void SetIterType(ICP_iter_type iter_type)
		{
			m_iter_type = iter_type;
		}

		float Estimate(std::vector<PtPair>& correspondences, trimesh::xform& source_form);
		void SetTracer(ProjectionICPTracer* tracer);
	protected:
		float m_dist_thresh_mult;
		float m_normdot_thresh;
		ICP_iter_type m_iter_type;
		ProjectionICPTracer* m_tracer;
	};
}