#pragma once
#include "ui_callback.h"
#include "es_ui.h"
#include <osgWrapper/RenderView.h>

class ESRender
{
public:
	ESRender(OSGWrapper::RenderView& view);
	~ESRender();

	void SetUICallback(UICallback* callback);

private:
	OSGWrapper::RenderView& m_view;

	osg::ref_ptr<ESUI> m_ui;
};