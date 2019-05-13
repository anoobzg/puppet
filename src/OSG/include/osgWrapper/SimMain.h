#pragma once
#include <osgWrapper\RenderView.h>
#include <osgWrapper\RenderScene.h>
#include <iostream>

namespace OSGWrapper
{
	int SimMain(int argc, char* argv[]);
}

int main(int argc, char* argv[])
{
	using namespace OSGWrapper;

	return SimMain(argc, argv);
}

OSGWrapper::RenderScene* CreateRenderScene(int argc, char* argv[]);