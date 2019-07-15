#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include "framegeometry.h"

class FrameNode : public OSGWrapper::AttributeUtilNode
{
public:
	FrameNode();
	virtual ~FrameNode();

	void UpdateGlobalMatrix(const osg::Matrixf& matrix);

	FrameGeometry* GetUsedGeometry();
	FrameGeometry* GetFreeGeometry();
	void Exchange(bool add);
	bool First();
private:
	osg::ref_ptr<osg::Uniform> m_matrix_uniform;
	osg::Matrixf m_matrix;

	osg::ref_ptr<FrameGeometry> m_used_geometry;
	osg::ref_ptr<FrameGeometry> m_free_geometry;
	bool m_first;
};