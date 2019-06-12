#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>
#include "feedgeometry.h"

class Operation : public OSGWrapper::AttributeUtilNode
{
public:
	Operation();
	virtual ~Operation();

	virtual bool ReleaseOnFrame(int frame);
	virtual bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	virtual const char* OperationName();
};