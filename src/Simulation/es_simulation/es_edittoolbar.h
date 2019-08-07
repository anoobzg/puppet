#pragma once
#include "es_button.h"

class ESEditToolbar
{
public:
	ESEditToolbar();
	~ESEditToolbar();

	void Setup(const osg::Vec2f& offset, const osg::Vec2f& size);
	EsButton* GetExitButton();
protected:
	osg::ref_ptr<EsButton> m_exit_button;
};