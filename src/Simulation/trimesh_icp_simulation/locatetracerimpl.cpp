#include "locatetracerimpl.h"

LocateTracerImpl::LocateTracerImpl()
	:m_save(false)
{
	m_ff_writer.PushHead("time");
	m_fm_writer.PushHead("time");
	m_re_writer.PushHead("time");
}

LocateTracerImpl::~LocateTracerImpl()
{

}

void LocateTracerImpl::OnFF()
{

}

void LocateTracerImpl::OnBeforeF2F()
{
	m_ff_writer.TickStart();
}

void LocateTracerImpl::OnAfterF2F()
{
	m_ff_writer.TickEnd();
}

void LocateTracerImpl::OnFM()
{

}

void LocateTracerImpl::OnBeforeF2M()
{
	m_fm_writer.TickStart();
}

void LocateTracerImpl::OnAfterF2M()
{
	m_fm_writer.TickEnd();
}

void LocateTracerImpl::OnRelocate()
{

}

void LocateTracerImpl::OnBeforeRelocate()
{
	m_re_writer.TickStart();
}

void LocateTracerImpl::OnAfterRelocate()
{
	m_re_writer.TickEnd();
}

void LocateTracerImpl::SaveFF(const std::string& file)
{
	m_ff_writer.Output(file);
}

void LocateTracerImpl::SaveFM(const std::string& file)
{
	m_fm_writer.Output(file);
}

void LocateTracerImpl::SaveRelocate(const std::string& file)
{
	m_re_writer.Output(file);
}

void LocateTracerImpl::OnLocateFailed(TriMeshPtr mesh, const trimesh::xform& init_matrix, trimesh::TriMesh* all_mesh)
{
	if (m_save)
	{
		static int index = 0;
		char name[128];
		sprintf(name, "frame_%d.ply", index);
		std::string frame_file(name);
		sprintf(name, "model_%d.ply", index);
		std::string model_file(name);
		sprintf(name, "matrix_%d", index);
		std::string matrix_file(name);

		mesh->write("norm:" + m_directory + frame_file);
		all_mesh->write("norm:" + m_directory + model_file);

		std::fstream out;
		std::string out_file = m_directory + "\\" + matrix_file;
		out.open(out_file.c_str(), std::ios::binary | std::ios::out);
		if (out.is_open())
		{
			out.write((const char*)init_matrix.data(), sizeof(double) * 16);
		}
		out.close();

		++index;
		//std::cout << "Save Locate Failed Data. " << std::endl;
	}
}

void LocateTracerImpl::SetSaveLocatedFailedData(bool save)
{
	m_save = save;
}

void LocateTracerImpl::SetSaveDirectory(const std::string& directory)
{
	m_directory = directory;
}