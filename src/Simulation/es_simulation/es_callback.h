#pragma once
#include "ui_callback.h"

class ESRender;
class EsCallback : public UICallback
{
public:
	EsCallback(ESRender& render);
	virtual ~EsCallback();

protected:
	ESRender& m_render;
};
