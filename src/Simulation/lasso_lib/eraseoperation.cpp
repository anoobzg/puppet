#include "eraseoperation.h"
#include <osgWrapper\RenderScene.h>
#include <iostream>

EraseOperation::EraseOperation(int frame, FeedGeometry* geometry)
	:m_geometry(geometry), m_frame(frame)
{
	SetRenderProgram("feed_erase");
	m_pickxy = new osg::Uniform("pick_xy", osg::Vec2f(-2000.0f, -2000.0f));
	AddChild(m_geometry);
	AddUniform(m_pickxy);
}

EraseOperation::~EraseOperation()
{

}

const char* EraseOperation::OperationName()
{
	return "EraseOperation";
}

bool EraseOperation::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	osg::Vec2f xy = osg::Vec2f(ea.getX(), ea.getY());
	//std::cout << "x :" << xy.x() << " y :" << xy.y() << std::endl;
	if (LEFT_MOUSE_PUSH(ea))
		m_pickxy->set(xy);
	else if (LEFT_MOUSE_DRAG(ea))
		m_pickxy->set(xy);
	else if (LEFT_MOUSE_RELEASE(ea))
		m_pickxy->set(osg::Vec2f(-2000.0f, -2000.0f));
	return true;
}