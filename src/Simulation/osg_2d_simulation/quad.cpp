#include "quad.h"
#include <osgWrapper\UtilCreator.h>

Quad::Quad()
{

}

Quad::~Quad()
{

}

OSGWrapper::QuadAttributeUtilNode* Quad::Generate()
{
	OSGWrapper::QuadAttributeUtilNode* node = new OSGWrapper::QuadAttributeUtilNode(1);
	osg::Geometry* geometry = OSGWrapper::UtilCreator::CreateUnitQuad();
	node->AddChild(geometry);
	return node;
}