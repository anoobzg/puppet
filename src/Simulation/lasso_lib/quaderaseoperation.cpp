#include "quaderaseoperation.h"
#include <osgWrapper\RenderScene.h>
#include <iostream>

QuadEraseOperation::QuadEraseOperation(int frame, FeedGeometry* geometry)
	:m_geometry(geometry), m_frame(frame)
{
	SetRenderProgram("feed_quaderase");
	AddChild(m_geometry);

	m_rectx_range = new osg::Uniform("rectx_range", osg::Vec2f());
	m_recty_range = new osg::Uniform("recty_range", osg::Vec2f());
	AddUniform(m_rectx_range);
	AddUniform(m_recty_range);

	SetInvalid();
}

QuadEraseOperation::~QuadEraseOperation()
{

}

const char* QuadEraseOperation::OperationName()
{
	return "QuadEraseOperation";
}

bool QuadEraseOperation::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	osg::Vec2f xy = osg::Vec2f(ea.getX(), ea.getY());
	//std::cout << "x :" << xy.x() << " y :" << xy.y() << std::endl;
	if (LEFT_MOUSE_PUSH(ea))
	{
		m_first_point = xy;
		m_last_point = xy;
		UpdateRange();
	}
	else if (LEFT_MOUSE_DRAG(ea))
	{
		m_last_point = xy;
		UpdateRange();
	}
	else if (LEFT_MOUSE_RELEASE(ea))
		/*SetInvalid()*/;
	return true;
}

void QuadEraseOperation::SetInvalid()
{
	m_rectx_range->set(osg::Vec2f(-1000.0f, -500.0f));
	m_recty_range->set(osg::Vec2f(-1000.0f, -500.0f));
}

void QuadEraseOperation::UpdateRange()
{
	float xmin = std::fmin(m_first_point.x(), m_last_point.x());
	float xmax = std::fmax(m_first_point.x(), m_last_point.x());
	float ymin = std::fmin(m_first_point.y(), m_last_point.y());
	float ymax = std::fmax(m_first_point.y(), m_last_point.y());
	m_rectx_range->set(osg::Vec2f(xmin, xmax));
	m_recty_range->set(osg::Vec2f(ymin, ymax));
}