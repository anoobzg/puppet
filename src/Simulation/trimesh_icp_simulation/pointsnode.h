#pragma once
#include <osgWrapper\AttributeUtilNode.h>
#include "TriMesh.h"

class PointsNode : public OSGWrapper::AttributeUtilNode
{
public:
	PointsNode(trimesh::TriMesh& mesh);
	~PointsNode();
};
