#pragma once
#include "RenderThread.h"
#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\AttributeUtilNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AnimationScheduler.h>
#include <osgWrapper/ScreenLineText.h>

class TScene : public simtool::RenderThreadBaseScene
{
public:
	TScene();
	virtual ~TScene();
};