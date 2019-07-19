#pragma once
#include <osgWrapper/UIItem.h>
#include "button.h"

class UI : public OSGWrapper::UIItem
{
public:
	UI();
	virtual ~UI();

protected:
	std::vector<osg::ref_ptr<Button>> m_buttons;
};