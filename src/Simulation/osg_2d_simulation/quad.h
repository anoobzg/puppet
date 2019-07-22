#pragma once
#include <osgWrapper\UIQuad.h>

class Quad : public OSGWrapper::UIQuad
{
public:
	Quad();
	~Quad();

	OSGWrapper::QuadAttributeUtilNode* Generate();
};