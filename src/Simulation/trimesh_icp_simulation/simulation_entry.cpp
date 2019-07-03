#include "simulation_entry.h"
#include "renderthread.h"
#include "render.h"
#include "keyframerender.h"
#include "slammer.h"
#include <boost\format.hpp>
#include "compute_boundingbox.h"
#include "base/at_exit.h"

int simulation_entry(int argc, char* argv[])
{
	base::AtExitManager exit_manager;

	base::WaitableEvent e(base::WaitableEvent::ResetPolicy::AUTOMATIC,
		base::WaitableEvent::InitialState::NOT_SIGNALED);

	std::string config_file;
	bool use_keyframe_tracer = false;
	if (argc >= 3) config_file = argv[2];
	if (argc >= 4 && !strcmp("trace_keyframe", argv[3])) use_keyframe_tracer = true;

	Render render;
	RenderThread render_thread(e);
	render_thread.StartRender(render.GetScene());

	KeyFrameRender keyframe_render;
	RenderThread keyframe_thread(e);
	
	KeyFrameTracer* keyframe_tracer = NULL;
	::Sleep(1000);
	render.StartRender();
	if (use_keyframe_tracer)
	{
		keyframe_thread.StartRender(keyframe_render.GetScene());
		keyframe_render.StartRender();
		keyframe_tracer = &keyframe_render;
	}

	Slammer slam;
	slam.Start(config_file, &render, keyframe_tracer);
	e.Wait();

	slam.Stop();
	render.StopRender();
	keyframe_render.StopRender();
	render_thread.StopRender();
	keyframe_thread.StopRender();
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