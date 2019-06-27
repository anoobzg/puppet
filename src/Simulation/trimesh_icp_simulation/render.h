#pragma once
#include "interface.h"
#include <base\threading\thread.h>
#include "simulationscene.h"
#include "chunkgeometry.h"

class Render : public base::Thread, public VOTracer
{
public:
	Render();
	virtual ~Render();

	void OnFrame(RenderData* data);
	void OnFrame(PatchRenderData* data);

	void StartRender();
	void StopRender();

	SimulationScene* GetScene();
protected:
	void ShowOneFrame(RenderData* data);
	void ShowPatch(PatchRenderData* data);
	void Convert(osg::Matrixf& matrix, const trimesh::xform& xf);
	void BuildPatch(PatchRenderData* data);
private:
	osg::ref_ptr<SimulationScene> m_scene;

	std::vector<osg::ref_ptr<ChunkGeometry>> m_geometries;
};