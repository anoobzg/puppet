#include "simulation_entry.h"
#include "renderthread.h"
#include "render.h"
#include "slammer.h"
#include <boost\format.hpp>
#include "compute_boundingbox.h"
#include "base/at_exit.h"

int simulation_entry(int argc, char* argv[])
{
	base::AtExitManager exit_manager;

	base::WaitableEvent e(base::WaitableEvent::ResetPolicy::AUTOMATIC,
		base::WaitableEvent::InitialState::NOT_SIGNALED);

	Render render;
	RenderThread render_thread(e);
	render_thread.StartRender(render.GetScene());

	::Sleep(1000);
	std::string config_file;
	if (argc >= 3) config_file = argv[2];

	render.StartRender();

	Slammer slam;
	slam.Start(config_file, &render);
	e.Wait();

	slam.Stop();
	render.StopRender();
	render_thread.StopRender();
	return EXIT_SUCCESS;
}

int vo(int argc, char* argv[])
{
	std::string config_file;
	if (argc >= 3) config_file = argv[2];

	SlamParameters parameters;
	parameters.LoadFromFile(config_file);
	VOImpl m_vo_impl;
	m_vo_impl.Setup(parameters);

	int index = 0;
	const ReaderParameters& read_parameters = parameters.reader_param;
	const DebugParameters& debug_parameters = parameters.debug_param;
	while (true)
	{
		std::string name = read_parameters.directory + "//" +
			boost::str(boost::format(read_parameters.pattern.c_str()) % index);
		TriMeshPtr mesh(trimesh::TriMesh::read(name));

		if (!mesh)
			break;

		trimesh::ComputeBoundingbox(mesh->vertices, mesh->bbox);
		LocateData data;
		data.lost = true;
		data.locate_type = 0;
		m_vo_impl.ProcessOneFrame(mesh, data);

		if (!data.lost)
		{
			TriMeshPtr out_mesh(new trimesh::TriMesh());
			trimesh::xform m = mesh->global;
			size_t size = mesh->vertices.size();
			out_mesh->vertices.resize(size);
			for (size_t i = 0; i < size; ++i)
				out_mesh->vertices.at(i) = m * mesh->vertices.at(i);

			std::string file = debug_parameters.out_directory + "//" +
				boost::str(boost::format(read_parameters.pattern.c_str()) % index);
			out_mesh->write(file);
		}

		++index;
	}

	return EXIT_SUCCESS;
}