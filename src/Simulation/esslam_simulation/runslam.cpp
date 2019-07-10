#include "runslam.h"
#include <base/at_exit.h>
#include <base/synchronization/waitable_event.h>
#include "RenderThread.h"
#include "render.h"

using namespace simtool;
int run_slam(esslam::IESSlam& slam, int argc, char* argv[])
{
	base::AtExitManager exit_manager;

	base::WaitableEvent e(base::WaitableEvent::ResetPolicy::AUTOMATIC,
		base::WaitableEvent::InitialState::NOT_SIGNALED);

	std::string config_file;
	if (argc >= 2) config_file = argv[1];

	Render render;
	RenderThread render_thread(e);
	render_thread.StartRender(render.GetScene());
	::Sleep(1000);
	render.StartRender();
	slam.SetVisualTracer(&render);

	esslam::SetupParameter parameters;
	parameters.default_config = config_file;
	parameters.type = esslam::e_load_from_file;

	slam.SetupParameters(parameters);
	slam.Start();
	e.Wait();

	slam.Stop();
	render.StopRender();
	render_thread.StopRender();
	return EXIT_SUCCESS;
}