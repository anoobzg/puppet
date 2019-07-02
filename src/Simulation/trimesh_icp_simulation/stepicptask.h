#pragma once
#include "task.h"
#include "icpnode.h"
#include "projectionicp.h"
#include "load_calib.h"
#include "screengraph.h"

class StepICPTask : public Task, public trimesh::ProjectionICPTracer
{
public:
	StepICPTask(trimesh::CameraData& data, ICPNode* target, ICPNode* source, ScreenGraph* screen_graph);
	virtual ~StepICPTask();

	bool Execute();
	void SetAttributeNode(OSGWrapper::AttributeUtilNode* node);
	void SetUseFast(bool use);
protected:
	void OnPreStepCorrespondences(const std::vector<trimesh::PtPair>& correspondences);
	void OnMatrix(const trimesh::xform& xf);
	void OnError(float error);
protected:
	trimesh::CameraData& m_data;
	osg::ref_ptr<ICPNode> m_target;
	osg::ref_ptr<ICPNode> m_source;
	osg::ref_ptr<ScreenGraph> m_screen_graph;
	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_lines;

	trimesh::ProjectionICP m_icp;

	osg::Matrixf m_last_matrix;
	bool m_use_fast;
};