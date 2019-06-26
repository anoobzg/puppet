#include "pointsnode.h"
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>

PointsNode::PointsNode(trimesh::TriMesh& mesh)
{
	size_t vertex_num = mesh.vertices.size();
	osg::Array* coord_array = OSGWrapper::ArrayCreator::CreateVec3Array(vertex_num, (float*)&mesh.vertices[0]);
	osg::DrawArrays* draw_array = new osg::DrawArrays(GL_POINTS, 0, vertex_num);
	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array);
	AddChild(geometry);
}

PointsNode::~PointsNode()
{

}