#pragma once
#include "InnerInterface.h"
#include "csvwriter.h"
#include "EsslamBaseExport.h"

namespace esslam
{
	class ESSLAM_API VOProfilerImpl : public VOProfiler
	{
	public:
		VOProfilerImpl();
		virtual ~VOProfilerImpl();

		void OnBeforeLocate();
		void OnAfterLocate();
		void OnMesh(const trimesh::TriMesh& mesh);
		void OnLocateResult(const LocateData& locate_data);

		void Write(const std::string& file);
	protected:
		string_util::CSVWriter m_writer;
	};
}