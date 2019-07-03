#pragma once
#include <osg\Geometry>
#include "TriMesh.h"

osg::Geometry* Create(trimesh::TriMesh* mesh);
void T2OConvert(osg::Matrixf& matrix, const trimesh::xform& xf);