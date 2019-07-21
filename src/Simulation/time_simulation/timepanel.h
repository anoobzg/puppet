#pragma once
#include <osgWrapper/UIItem.h>
#include "stampobject.h"

class TimePanel : public OSGWrapper::UIItem
{
public:
	TimePanel();
	virtual ~TimePanel();

	void Load(const char* usb, const char* build, const char* locate, const char* fusion);
	StampObject* Load(const char* file);
protected:
	void SetupItems();
private:
	std::auto_ptr<StampObject> m_usb_object;
	std::auto_ptr<StampObject> m_build_object;
	std::auto_ptr<StampObject> m_locate_object;
	std::auto_ptr<StampObject> m_fusion_object;
};