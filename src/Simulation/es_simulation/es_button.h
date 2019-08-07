#pragma once
#include <osgWrapper/Quad.h>

class EsButton : public OSGWrapper::Quad
{
public:
	EsButton();
	virtual ~EsButton();

	void SetRawColor(const osg::Vec4f& color);
protected:
	void OnDoubleClick();
	void OnUnHover();
	void OnHover();
	void OnPushDown();
	void OnPushUp();
	void OnClick();
protected:
	osg::Vec4f m_color;

	bool m_selected;
};