#pragma once
#include "interface.h"
#include "csvwriter.h"

class LocateTracerImpl : public LocateTracer
{
public:
	LocateTracerImpl();
	virtual ~LocateTracerImpl();

	void OnFF();
	void OnBeforeF2F();
	void OnAfterF2F();
	void OnFM();
	void OnBeforeF2M();
	void OnAfterF2M();
	void OnRelocate();
	void OnBeforeRelocate();
	void OnAfterRelocate();

	void OnLocateFailed(TriMeshPtr mesh, const trimesh::xform& init_matrix, trimesh::TriMesh* all_mesh);
	void SaveFF(const std::string& file);
	void SaveFM(const std::string& file);
	void SaveRelocate(const std::string& file);

	void SetSaveLocatedFailedData(bool save);
	void SetSaveDirectory(const std::string& directory);
private:
	CSVWriter m_ff_writer;
	CSVWriter m_fm_writer;
	CSVWriter m_re_writer;

	bool m_save;
	std::string m_directory;
};