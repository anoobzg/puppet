#pragma once
#include "es_button.h"

class ESMainToolbar
{
public:
	ESMainToolbar();
	~ESMainToolbar();

	void Setup(const osg::Vec2f& offset, const osg::Vec2f& size);
	EsButton* GetScanButton();
	EsButton* GetClearButton();
protected:
	osg::ref_ptr<EsButton> m_scan_button;
	osg::ref_ptr<EsButton> m_clear_button;
};