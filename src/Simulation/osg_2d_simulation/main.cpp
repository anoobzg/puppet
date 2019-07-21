#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>
#include <osgWrapper/ScreenQuad.h>
#include <osgWrapper/ScreenDividingRule.h>
#include <osgWrapper/ScreenSingleText.h>
#include <osgWrapper\FreetypeFontManager.h>
#include <osgWrapper/ScreenDistanceIndicator.h>
#include "ui.h"

using namespace OSGWrapper;

class TestAnimation : public osg::NodeCallback
{
public:
	TestAnimation():origin(0.0f, 0.0f),height(100.0f){}
	~TestAnimation() {}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		OSGWrapper::ScreenQuad* quad = (OSGWrapper::ScreenQuad*)node;
		if (quad)
		{
			origin += osg::Vec2f(2.0f, 2.0f);
			height += 2.0f;

			quad->SetOrigin(origin);
			quad->SetHeight(height);
			if (origin.x() > 1920.0f) origin = osg::Vec2(0.0f, 0.0f);
			if (height > 600.0f) height = 100.0f;
		}
	}

	osg::Vec2 origin;
	float height;
};

class DividingRuleAnimation : public osg::NodeCallback
{
public:
	DividingRuleAnimation() :origin(0.0f, 0.0f), height(100.0f), divide1(5), divide2(5),d(0.0f){}
	~DividingRuleAnimation() {}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		OSGWrapper::ScreenDividingRule* quad = dynamic_cast<OSGWrapper::ScreenDividingRule*>(node);
		if (quad)
		{
			d += 0.01f;

			quad->SetDivideRatio(d);
			if (d > 1.0f) d = 0.0f;
		}
	}

	osg::Vec2 origin;
	float height;
	int divide1;
	int divide2;
	float d;
};

class IndicatorAnimation : public osg::NodeCallback
{
public:
	IndicatorAnimation() :x(0.0f){}
	~IndicatorAnimation() {}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		OSGWrapper::ScreenDistanceIndicator* indicator = dynamic_cast<OSGWrapper::ScreenDistanceIndicator*>(node);
		if (indicator)
		{
			float value = 30.0f * sinf(x) + 60.0f;
			indicator->SetCurrentValue(value);

			x += 0.03f;
		}
	}

	float x;
};

osg::Node* CreateContent()
{
	//OSGWrapper::ScreenDividingRule* rule = new OSGWrapper::ScreenDividingRule();
	//rule->setUpdateCallback(new DividingRuleAnimation());
	//rule->SetOrigin(osg::Vec2(400.0f, 200.0f));
	//rule->SetWidth(20.0f);
	//rule->SetHeight(500.0f);
	//return rule;
	//OSGWrapper::ScreenQuad* quad = new OSGWrapper::ScreenQuad();
	//quad->setUpdateCallback(new TestAnimation());
	//return quad;
	OSGWrapper::ScreenSingleText* text = new OSGWrapper::ScreenSingleText();
	text->SetOrigin(osg::Vec2f(400.0f, 100.0f));
	text->SetFont("D:\\Data\\Fonts\\pp.ttf");
	text->SetText(L'\x8d39');
	text->SetColor(osg::Vec4f(0.0f, 1.0f, 1.0f, 1.0f));
	return text;

	//OSGWrapper::ScreenDistanceIndicator* indicator = new OSGWrapper::ScreenDistanceIndicator();
	//indicator->setUpdateCallback(new IndicatorAnimation());
	//indicator->SetActiveColor(osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
	//indicator->SetDefaultColor(osg::Vec4f(0.5f, 0.5f, 0.5f, 1.0f));
	//return indicator;
}

int main(int argc, char* argv[])
{
	osg::ref_ptr<RenderView> view = new RenderView();
	view->SetGlobalUI(new UI());
	osg::ref_ptr<RenderScene> scene = new RenderScene();
	scene->addChild(CreateContent());

	view->SetBackgroundColor(osg::Vec4(0.2f, 0.2f, 0.2f, 1.0f));
	view->setUpViewInWindow(50, 50, 1680, 860);
	view->SetCurrentScene(scene);

	RenderService::Instance().addView(view);
	return RenderService::Instance().Run();
}
