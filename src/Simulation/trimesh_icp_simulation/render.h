#pragma once
#include "interface.h"
#include <base\threading\thread.h>
#include "simulationscene.h"

class Render : public base::Thread, public VOTracer
{
public:
	Render();
	virtual ~Render();

	void OnFrame(RenderData* data);

	void StartRender();
	void StopRender();

	SimulationScene* GetScene();
protected:
	void ShowOneFrame(RenderData* data);
	void Convert(osg::Matrixf& matrix, const trimesh::xform& xf);
private:
	osg::ref_ptr<SimulationScene> m_scene;
};