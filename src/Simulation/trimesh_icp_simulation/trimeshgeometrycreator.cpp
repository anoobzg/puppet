#include "trimeshgeometrycreator.h"
#include <osgWrapper\GeometryCreator.h>
#include <osgWrapper\ArrayCreator.h>

osg::Geometry* Create(trimesh::TriMesh* mesh)
{
	if (!mesh || mesh->vertices.size() == 0) return NULL;

	size_t vertex_num = mesh->vertices.size();
	osg::Array* coord_array = OSGWrapper::ArrayCreator::CreateVec3Array(vertex_num, (float*)&mesh->vertices[0]);
	osg::Array* normal_array = OSGWrapper::ArrayCreator::CreateVec3Array(vertex_num, (float*)&mesh->normals[0]);
	osg::DrawArrays* draw_array = new osg::DrawArrays(GL_POINTS, 0, vertex_num);
	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array, normal_array);
	return geometry;
}

void T2OConvert(osg::Matrixf& matrix, const trimesh::xform& xf)
{
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			matrix(i, j) = xf(j, i);
}
