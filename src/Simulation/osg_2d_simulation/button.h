#pragma once
#include <osgWrapper/UIItem.h>

class Button : public OSGWrapper::UIItem
{
public:
	Button();
	virtual ~Button();

	void SetHover(const std::string& file);
	void SetNormal(const std::string& file);
	void SetSelect(const std::string& file);

protected:
	void OnUnHover();
	void OnHover();
	void OnPushDown();
	void OnPushUp();
	void OnClick();

	osg::Image* Load(const std::string& file);
private:
	osg::ref_ptr<osg::Texture2D> m_hover_image;
	osg::ref_ptr<osg::Texture2D> m_normal_image;
	osg::ref_ptr<osg::Texture2D> m_select_image;
};