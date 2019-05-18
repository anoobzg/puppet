#include "curvature_geode.h"
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>
#include "MeshGeodeBuilder.h"

using namespace OSGWrapper;
using namespace OSGBuilder;
CurvatureGeode::CurvatureGeode()
{

}

CurvatureGeode::~CurvatureGeode()
{

}

void CurvatureGeode::Reload(Mesh& mesh)
{
	float* curvature = new float[mesh.vertex_number];
	memset(curvature, 0, sizeof(float) * mesh.vertex_number);

	m_curvature_array = (osg::FloatArray*)OSGWrapper::ArrayCreator::CreateFloatArray(mesh.vertex_number, curvature);
	delete[] curvature;

	osg::Geometry* geometry = MeshGeodeBuilder::Build(mesh, OSGBuilder::MGT_TRIANGLE, OSGBuilder::MGT_POSITION | OSGBuilder::MGT_NORMAL);
	if (geometry)
	{
		geometry->setVertexAttribArray(2, m_curvature_array, osg::Array::BIND_PER_VERTEX);
		addDrawable(geometry);
		geometry->setCullingActive(false);
		setCullingActive(false);
	}
}

void CurvatureGeode::UpdateCurvature(unsigned vertex_number, float* curvature)
{
	if (vertex_number != m_curvature_array->size())
		return;

	for (unsigned i = 0; i < vertex_number; ++i)
		m_curvature_array->at(i) = *curvature++;
	m_curvature_array->dirty();
}