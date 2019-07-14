#pragma once
#include <base\threading\thread.h>
#include "tscene.h"

class TRender : public base::Thread
{
public:
	TRender();
	virtual ~TRender();

	void StartRender();
	void StopRender();

	TScene* GetScene();
protected:
	osg::ref_ptr<TScene> m_scene;
};