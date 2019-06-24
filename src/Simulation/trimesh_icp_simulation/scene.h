#pragma once
#include <osgWrapper\RenderScene.h>
#include "data.h"

class Scene : public OSGWrapper::RenderScene
{
public:
	Scene(Data& data);
	~Scene();

protected:
	Data& m_data;
};