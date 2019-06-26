#pragma once
#include "interface.h"
#include "csvwriter.h"

class VOProfilerImpl : public VOProfiler
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
	CSVWriter m_writer;
};