#include "icpnode.h"
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>

ICPNode::ICPNode(trimesh::TriMesh& mesh)
	:m_mesh(mesh)
{
	size_t vertex_num = mesh.vertices.size();
	osg::Array* coord_array = OSGWrapper::ArrayCreator::CreateVec3Array(vertex_num, (float*)&mesh.vertices[0]);
	osg::Array* normal_array = OSGWrapper::ArrayCreator::CreateVec3Array(vertex_num, (float*)&mesh.normals[0]);
	osg::DrawArrays* draw_array = new osg::DrawArrays(GL_POINTS, 0, vertex_num);
	m_geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array, normal_array);
	AddChild(m_geometry);

	SetRenderProgram("trimeshicptest");
	m_color = new osg::Uniform("color", osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
	AddUniform(m_color);

	m_bounding_box = new BoundingBox();
	AddChild(m_bounding_box);

	m_matrix = osg::Matrixf::identity();
	m_matrix_uniform = new osg::Uniform("icp_matrix", m_matrix);
	AddUniform(m_matrix_uniform);
	m_bounding_box->UpdateBoundingBox(m_geometry->getBoundingBox(), m_matrix);
}

ICPNode::~ICPNode()
{

}

trimesh::TriMesh& ICPNode::GetMesh()
{
	return m_mesh;
}

void ICPNode::SetColor(const osg::Vec4f& color)
{
	m_color->set(color);
}

void ICPNode::UpdateMatrix(const osg::Matrixf& matrix)
{
	m_matrix_uniform->set(matrix);
	m_matrix = matrix;
	m_bounding_box->UpdateBoundingBox(m_geometry->getBoundingBox(), m_matrix);
}