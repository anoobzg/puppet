#pragma once
#include "projectionicp.h"
#include "csvwriter.h"
#include "EsslamBaseExport.h"

namespace esslam
{
	class ESSLAM_API ProjectionICPTracerImpl : public trimesh::ProjectionICPTracer
	{
	public:
		ProjectionICPTracerImpl();
		virtual ~ProjectionICPTracerImpl();
		void OnError(float error);
		void OnPreStepCorrespondences(const std::vector<trimesh::PtPair>& correspondences);
		void OnStepCorrespondences(const std::vector<trimesh::PtPair>& correspondences);

		void Save(const std::string& file);
	protected:
		string_util::CSVWriter m_writer;
	};
}