#include "scaleanimation.h"
#include <osgWrapper/OSGMathUtil.h>

using namespace OSGWrapper;
ScaleAnimation::ScaleAnimation(ManipulableNode& node)
	:m_node(node)
{
}

ScaleAnimation::~ScaleAnimation()
{
}

void ScaleAnimation::OnPlay(float lambda)
{
	//osg::Vec3f scale_start = m_start.getScale();
	//osg::Vec3f scale_end = m_end.getScale();
	//osg::Vec3f translate_start = m_start.getTrans();
	//osg::Vec3f translate_end = m_end.getTrans();
	//osg::Quat rotate_start = m_start.getRotate();
	//osg::Quat rotate_end = m_end.getRotate();
	//
	//lambda = pow(lambda, 0.25f);
	//osg::Vec3f scale = OSGMathUtil::Lerp(scale_start, scale_end, lambda);
	//osg::Vec3f translate = OSGMathUtil::Lerp(translate_start, translate_end, lambda);
	//osg::Quat rotation = OSGMathUtil::Lerp(rotate_start, rotate_end, lambda);
	//
	//m_node.SetMatrix(osg::Matrixf::rotate(rotation) * osg::Matrixf::scale(scale) * osg::Matrixf::translate(translate));

	
	osg::Matrixf final_matrix = m_node.GetLocalMatrix() * osg::Matrixf::translate(-m_center) *
		osg::Matrixf::scale(1.01f, 1.01f, 1.01f) * osg::Matrixf::translate(m_center);
	m_node.SetMatrix(final_matrix);
}

void ScaleAnimation::SetMatrix(const osg::Matrixf& end)
{
	m_start = m_node.GetLocalMatrix();
	m_end = end;
}

void ScaleAnimation::SetCenter(const osg::Vec3f& center)
{
	m_center = center;
}
