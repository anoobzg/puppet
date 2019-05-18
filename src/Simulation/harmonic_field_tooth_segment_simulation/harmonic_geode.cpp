#include "harmonic_geode.h"
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>
#include <OSGBuilder\MeshGeodeBuilder.h>

using namespace OSGWrapper;
using namespace OSGBuilder;
HarmonicGeode::HarmonicGeode()
{

}

HarmonicGeode::~HarmonicGeode()
{

}

void HarmonicGeode::Reload(Mesh& mesh)
{
	float* harmonic = new float[mesh.vertex_number];
	for (unsigned i = 0; i < mesh.vertex_number; ++i)
		*(harmonic + i) = 0.5f;

	m_harmonic_array = (osg::FloatArray*)OSGWrapper::ArrayCreator::CreateFloatArray(mesh.vertex_number, harmonic);
	delete[] harmonic;

	osg::Geometry* geometry = MeshGeodeBuilder::Build(mesh, OSGBuilder::MGT_TRIANGLE, OSGBuilder::MGT_POSITION | OSGBuilder::MGT_NORMAL);
	if (geometry)
	{
		geometry->setVertexAttribArray(2, m_harmonic_array, osg::Array::BIND_PER_VERTEX);
		addDrawable(geometry);
		geometry->setCullingActive(false);
		setCullingActive(false);
	}
}

void HarmonicGeode::UpdateHarmonic(unsigned vertex_number, float* harmonic)
{
	if (vertex_number != m_harmonic_array->size())
		return;

	for (unsigned i = 0; i < vertex_number; ++i)
		m_harmonic_array->at(i) = *harmonic++;
	m_harmonic_array->dirty();
}