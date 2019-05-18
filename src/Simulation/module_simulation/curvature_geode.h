#pragma once
#include <osgWrapper\GeodeCreator.h>
#include "Mesh.h"

using namespace LauncaGeometry;
class CurvatureGeode : public osg::Geode
{
public:
	CurvatureGeode();
	~CurvatureGeode();

	void Reload(Mesh& mesh);
	void UpdateCurvature(unsigned vertex_number, float* curvature);

private:
	osg::ref_ptr<osg::FloatArray> m_curvature_array;
};