#pragma once
#include "slam_interface.h"
#include <base\threading\thread.h>
#include "simulationscene.h"
#include "chunkgeometry.h"

class Render : public base::Thread, public esslam::IVisualTracer
{
public:
	Render();
	virtual ~Render();

	void OnFrame(esslam::PatchRenderData* data);

	void StartRender();
	void StopRender();

	SimulationScene* GetScene();
protected:
	void ShowPatch(esslam::PatchRenderData* data);
	void Convert(osg::Matrixf& matrix, float* xf);
	void BuildPatch(esslam::PatchRenderData* data);
private:
	osg::ref_ptr<SimulationScene> m_scene;

	std::vector<osg::ref_ptr<ChunkGeometry>> m_geometries;
};