#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include <osgWrapper/ScreenLineText.h>

class ParametersText : public OSGWrapper::AttributeUtilNode
{
public:
	ParametersText(const std::wstring& name, int line);
	~ParametersText();

	void Inc();
	void Dec();
private:
	void SetValue(float value);
private:
	osg::ref_ptr<OSGWrapper::ScreenLineText> m_name;
	osg::ref_ptr<OSGWrapper::ScreenLineText> m_value_text;

	osg::ref_ptr<osg::Uniform> m_value_uniform;
	float m_value;

	int m_line;
	const float m_height;

	float m_value_x;
};