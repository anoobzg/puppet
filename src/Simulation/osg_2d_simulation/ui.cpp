#include "ui.h"
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>
#include <osgWrapper\UtilCreator.h>

UI::UI()
{
	Button* b1 = new Button();
	b1->SetOffset(osg::Vec2(100.0f, 100.0f));
	b1->SetHover("../../../src/Simulation/osg_2d_simulation/delete_hover.png");
	b1->SetNormal("../../../src/Simulation/osg_2d_simulation/delete_normal.png");
	b1->SetSelect("../../../src/Simulation/osg_2d_simulation/delete_pressed.png");
	//Button* b2 = new Button();
	//b2->SetOffset(osg::Vec2(400.0f, 300.0f));

	m_buttons.push_back(b1);
	//m_buttons.push_back(b2);
	AddItem(b1);
	//AddItem(b2);

	SetOffset(osg::Vec2(300.0f, 20.0f));
}

UI::~UI()
{

}