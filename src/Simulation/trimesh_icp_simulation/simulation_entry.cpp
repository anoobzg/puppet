#include "simulation_entry.h"
#include "renderthread.h"
#include "simulationscene.h"
#include "slammer.h"
#include "base/at_exit.h"

int simulation_entry(int argc, char* argv[])
{
	base::AtExitManager exit_manager;

	base::WaitableEvent e(base::WaitableEvent::ResetPolicy::AUTOMATIC,
		base::WaitableEvent::InitialState::NOT_SIGNALED);

	osg::ref_ptr<SimulationScene> scene = new SimulationScene();
	RenderThread render(e);
	render.StartRender(scene);

	Slammer slam;
	slam.Start();
	e.Wait();

	render.StopRender();
	slam.Stop();
	return EXIT_SUCCESS;
}