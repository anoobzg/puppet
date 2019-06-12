#include "lassonode.h"
#include <iostream>
#include <osgWrapper/ProgramManager.h>

#include "selectalloperation.h"
LassoNode::LassoNode()
	:m_frame(0)
{
	osg::Program* program = OSGWrapper::ProgramManager::Instance().Get("feed_selectall");
	if (program)
	{
		program->addTransformFeedBackVarying(std::string("feedback_attribute"));
		program->setTransformFeedBackMode(GL_INTERLEAVED_ATTRIBS);
	}
}

LassoNode::~LassoNode()
{

}

void LassoNode::Set(osg::Vec3Array* coord_array, osg::FloatArray* flag_array)
{
	m_feed_geometry = new FeedGeometry(coord_array, flag_array);

	//SelectAll();
}

bool LassoNode::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (m_operation) return m_operation->OnMouse(ea, aa);
	return false;
}

void LassoNode::OnFrame()
{
	++m_frame;
	if (m_operation && m_operation->ReleaseOnFrame(m_frame))
		ReleaseOperation();
	//std::cout << "frame number " << m_frame << std::endl;
}

void LassoNode::ReleaseOperation()
{
	RemoveAll();

	std::cout << m_operation->OperationName() << " Release on frame " << m_frame << std::endl;
	m_operation = 0;
}

void LassoNode::SelectAll()
{
	if (m_operation)
		return;

	m_operation = new SelectAllOperation(m_frame, m_feed_geometry);
	AddChild(m_operation);

	std::cout << "SelectAllOperation on frame " << m_frame << std::endl;
}

void LassoNode::DeselectAll()
{
	if (m_operation)
		return;
}
