#include "es_render.h"

ESRender::ESRender(OSGWrapper::RenderView& view)
	:m_view(view)
{
	m_ui = new ESUI();
	view.GetUIRoot()->AddQuad(m_ui, true);
}

ESRender::~ESRender()
{

}

void ESRender::SetUICallback(UICallback* callback)
{

}