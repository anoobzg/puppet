#pragma once
#include "task.h"
#include "icpnode.h"
#include "screengraph.h"

class ResetICPTask : public Task
{
public:
	ResetICPTask(ICPNode* source, ScreenGraph* screen_graph);
	virtual ~ResetICPTask();

	void SetAttributeNode(OSGWrapper::AttributeUtilNode* node);
	virtual bool Execute();
protected:
	osg::ref_ptr<ICPNode> m_source;
	osg::ref_ptr<ScreenGraph> m_graph;
	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_lines;
};