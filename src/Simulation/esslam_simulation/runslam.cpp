#include "runslam.h"
#include <base/at_exit.h>
#include <base/synchronization/waitable_event.h>
#include "RenderThread.h"
#include "render.h"

using namespace simtool;

class Exiter : public esslam::IReadTracer
{
public:
	Exiter(base::WaitableEvent& e)
		:ee(e)
	{

	}

	virtual ~Exiter()
	{

	}

	void OnFinished()
	{
		ee.Signal();
	}

private:
	base::WaitableEvent& ee;
};

class Builder : public esslam::IBuildTracer
{
public:
	Builder() {}
	virtual ~Builder() {}

	void OnPoints(int size, float* position, float* normal, unsigned char* color)
	{

	}

	void OnXform(const std::vector<trimesh::xform>& xforms)
	{

	}

	void OnKeyFrames(const std::vector<int>& keyframes)
	{
		std::cout << "All KeyFrames. " << keyframes.size() << std::endl;
		for (size_t i = 0; i < keyframes.size(); ++i)
			std::cout << keyframes.at(i) << " " << std::endl;
	}

};

int run_slam(esslam::IESSlam& slam, int argc, char* argv[])
{
	base::AtExitManager exit_manager;

	base::WaitableEvent e(base::WaitableEvent::ResetPolicy::AUTOMATIC,
		base::WaitableEvent::InitialState::NOT_SIGNALED);

	std::string config_file;
	if (argc >= 2) config_file = argv[1];
	bool cmd_mode = false;
	if (argc >= 3 && !strcmp(argv[2], "cmd")) cmd_mode = true;

	if (cmd_mode)
	{
		Exiter exer(e);

		esslam::SetupParameter parameters;
		parameters.default_config = config_file;
		parameters.type = esslam::e_load_from_file;

		slam.SetReadTracer(&exer);
		slam.SetupParameters(parameters);
		slam.Start();
		e.Wait();

		slam.Stop();

		Builder builder;
		slam.Build(&builder);
		slam.Clear();
	}
	else
	{
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
	}
	
	return EXIT_SUCCESS;
}