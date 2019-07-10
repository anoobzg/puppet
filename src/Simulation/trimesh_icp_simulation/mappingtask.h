#pragma once
#include "task.h"
#include "icpnode.h"
#include "projectionicp.h"
#include "load_calib.h"
#include "screengraph.h"

class MappingTask : public Task
{
public:
	MappingTask(trimesh::CameraData& data, trimesh::TriMesh& target, trimesh::TriMesh& source);
	virtual ~MappingTask();

	void SetAttributeNode(OSGWrapper::AttributeUtilNode* node);

	void SetSelf(bool self);
	bool Execute();
protected:
protected:
	trimesh::CameraData& m_data;
	trimesh::TriMesh& m_target;
	trimesh::TriMesh& m_source;

	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_lines;
	bool m_self;
};