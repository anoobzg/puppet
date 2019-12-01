#pragma once
#include <osgWrapper/Animation.h>
#include <osgWrapper/ManipulableNode.h>

class ScaleAnimation : public OSGWrapper::Animation
{
public:
	ScaleAnimation(OSGWrapper::ManipulableNode& node);
	~ScaleAnimation();

	void SetMatrix(const osg::Matrixf& end);
	void SetCenter(const osg::Vec3f& center);
	void OnPlay(float lambda);
private:
	OSGWrapper::ManipulableNode& m_node;

	osg::Matrixf m_start;
	osg::Matrixf m_end;

	osg::Vec3f m_center;
};