#pragma once
#pragma once
#include <osgWrapper\GeodeCreator.h>
#include "Mesh.h"

using namespace LauncaGeometry;
class HarmonicGeode : public osg::Geode
{
public:
	HarmonicGeode();
	~HarmonicGeode();

	void Reload(Mesh& mesh);
	void UpdateHarmonic(unsigned vertex_number, float* harmonic);

private:
	osg::ref_ptr<osg::FloatArray> m_harmonic_array;
};