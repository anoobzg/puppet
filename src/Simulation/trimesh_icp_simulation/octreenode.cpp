#include "octreenode.h"

OctreeNode::OctreeNode()
{
	AddUniform(new osg::Uniform("color", osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f)));
}

OctreeNode::~OctreeNode()
{

}