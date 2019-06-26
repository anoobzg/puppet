#pragma once
#include "TriMesh.h"
#include "octreescene.h"
#include "pointsnode.h"

class OctreeWork
{
public:
	OctreeWork();
	~OctreeWork();

	void SetRenderScene(OctreeScene* scene);
	void Move(int i);
private:
	osg::ref_ptr<OctreeScene> m_scene;
public:
	std::vector<trimesh::TriMesh*> m_meshes;
	std::vector<osg::ref_ptr<PointsNode>> m_points;

	int m_current;
	osg::ref_ptr<PointsNode> m_current_node;
};