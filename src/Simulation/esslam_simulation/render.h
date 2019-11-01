#pragma once
#include "slam_tracer.h"
#include <base\threading\thread.h>
#include "simulationscene.h"
#include "chunkgeometry.h"

class Render : public base::Thread, public esslam::IVisualTracer
{
public:
	Render();
	virtual ~Render();

	void OnFrame(esslam::PatchRenderData* data);
	void OnFrameLocated(esslam::FrameData* data);
	void OnNewPoints(esslam::NewAppendData* data);
	void OnFixFrame(esslam::FrameData* data);

	void StartRender();
	void StopRender();

	SimulationScene* GetScene();
protected:
	void ShowPatch(esslam::PatchRenderData* data);
	void Convert(osg::Matrixf& matrix, float* xf);
	void BuildPatch(esslam::PatchRenderData* data);
	void ShowCurrentFrame(esslam::FrameData* data);
	void AppendNewPoints(esslam::NewAppendData* data);
private:
	osg::ref_ptr<SimulationScene> m_scene;

	std::vector<osg::ref_ptr<ChunkGeometry>> m_geometries;

	int m_current_index;
	osg::ref_ptr<osg::Vec3Array> m_coord_array;
	osg::ref_ptr<osg::Vec3Array> m_normal_array;
	osg::ref_ptr<osg::Vec4Array> m_color_array;
	osg::ref_ptr<osg::DrawArrays> m_draw_arrays;
	osg::ref_ptr<osg::Geometry> m_geometry;
};