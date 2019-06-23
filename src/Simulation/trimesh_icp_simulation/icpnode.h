#pragma once
#include <osg\Geometry>
#include <osgWrapper\AttributeUtilNode.h>
#include <osg\Matrixf>
#include "boundingbox.h"
#include "TriMesh.h"

class ICPNode : public OSGWrapper::AttributeUtilNode
{
public:
	ICPNode(trimesh::TriMesh& mesh);
	~ICPNode();

	trimesh::TriMesh& GetMesh();
	void SetColor(const osg::Vec4f& color);
	void UpdateMatrix(const osg::Matrixf& matrix);
protected:
	trimesh::TriMesh& m_mesh;
	osg::ref_ptr<osg::Geometry> m_geometry;
	osg::ref_ptr<BoundingBox> m_bounding_box;
	osg::ref_ptr<osg::Uniform> m_color;
	osg::ref_ptr<osg::Uniform> m_matrix_uniform;

	osg::Matrixf m_matrix;
};