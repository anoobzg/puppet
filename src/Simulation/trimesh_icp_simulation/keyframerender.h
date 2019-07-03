#pragma once
#include "interface.h"
#include <base\threading\thread.h>
#include "keyframetracerscene.h"

class KeyFrameRender : public base::Thread , public KeyFrameTracer
{
public:
	KeyFrameRender();
	virtual ~KeyFrameRender();

	void StartRender();
	void StopRender();

	void OnKeyFrame(KeyFrameData* data);
	KeyFrameTracerScene* GetScene();
protected:
	void ProcessKeyFrame(KeyFrameData* data);
private:
	osg::ref_ptr<KeyFrameTracerScene> m_scene;
};