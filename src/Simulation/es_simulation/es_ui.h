#pragma once
#include <osgWrapper/UIQuad.h>
#include "es_maintoolbar.h"
#include "es_edittoolbar.h"

class ESUI : public OSGWrapper::UIQuad
{
public:
	ESUI();
	virtual ~ESUI();

	OSGWrapper::QuadAttributeUtilNode* Generate();
protected:
	ESMainToolbar m_maintoolbar;
	ESEditToolbar m_edittoolbar;
};