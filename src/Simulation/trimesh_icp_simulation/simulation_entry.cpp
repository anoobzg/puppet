#include "simulation_entry.h"
#include "renderthread.h"
#include "render.h"
#include "slammer.h"
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