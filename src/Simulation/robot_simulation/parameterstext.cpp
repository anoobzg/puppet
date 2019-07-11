#include "parameterstext.h"
#include <stdio.h>
#include <Windows.h>

ParametersText::ParametersText(const std::wstring& name, int line)
	:m_line(line), m_height(20.0f), m_value(0.0f)
{
	int size = (int)name.size();
	m_name = new OSGWrapper::ScreenLineText("D:\\Data\\Fonts\\msyhn_boot.ttf", size);
	m_value_text = new OSGWrapper::ScreenLineText("D:\\Data\\Fonts\\msyhn_boot.ttf", 7);

	AddChild(m_name);
	AddChild(m_value_text);
	SetMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	m_name->SetText(name);
	m_name->SetColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	m_name->SetOrigin(osg::Vec2f(0.0f, m_line * m_height));
	m_name->SetWidth(m_height); m_name->SetHeight(m_height);
	m_value_text->SetColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	m_value_text->SetOrigin(osg::Vec2f(size * m_height + 10.0f, m_line * m_height));
	m_value_text->SetWidth(m_height); m_value_text->SetHeight(m_height);
	
	m_value_uniform = new osg::Uniform("hue", 0.0f);
	//AddUniform(m_value_uniform);
	SetValue(m_value);
}

ParametersText::~ParametersText()
{

}

void ParametersText::Inc()
{

}

void ParametersText::Dec()
{

}

void ParametersText::SetValue(float value)
{
	m_value = value;
	m_value_uniform->set(m_value);

	char chars[64];
	sprintf(chars, "%f", m_value);

	wchar_t* m_wchar;
	int len = MultiByteToWideChar(CP_ACP, 0, chars, strlen(chars), NULL, 0);
	m_wchar = new wchar_t[len + 1];
	MultiByteToWideChar(CP_ACP, 0, chars, strlen(chars), m_wchar, len);
	m_wchar[len] = '\0';

	m_value_text->SetText(m_wchar);
	delete[]m_wchar;
}