#ifndef RENDER_SERVICE
#define RENDER_SERVICE
#include <osgViewer/CompositeViewer>

namespace OSGWrapper
{

class OSG_EXPORT RenderService : public osgViewer::CompositeViewer
{
	RenderService();

public:
	static void SetupRenderService();
	static void DestroyRenderService();

	static RenderService& Instance();

	~RenderService();

	void Refresh();

	void StartRenderingThread();
	void StopRenderingThread();
	void EnableRendering(bool bEnable);
	bool IsRenderingEnabled() const;

	void RenderOneFrameSynchronous();

	/*virtual*/ int Run();

	void SetRenderThreadPriority(int nPriority);

	void Clear();

	float ReferenceTime();
private:
	bool m_is_rendering_enabled;

	bool m_render_one_frame;

	static RenderService* m_instance;
};

}
#endif // RENDER_SERVICE