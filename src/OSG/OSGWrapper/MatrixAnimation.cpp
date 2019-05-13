#include <osgWrapper/MatrixAnimation.h>
#include <osgWrapper/OSGMathUtil.h>

namespace OSGWrapper
{
	MatrixAnimation::MatrixAnimation(ManipulableNode& node)
		:m_node(node)
	{
	}

	MatrixAnimation::~MatrixAnimation()
	{
	}

	void MatrixAnimation::OnPlay(float lambda)
	{
		osg::Vec3f scale_start = m_start.getScale();
		osg::Vec3f scale_end = m_end.getScale();
		osg::Vec3f translate_start = m_start.getTrans();
		osg::Vec3f translate_end = m_end.getTrans();
		osg::Quat rotate_start = m_start.getRotate();
		osg::Quat rotate_end = m_end.getRotate();

		lambda = pow(lambda, 0.25f);
		osg::Vec3f scale = OSGMathUtil::Lerp(scale_start, scale_end, lambda);
		osg::Vec3f translate = OSGMathUtil::Lerp(translate_start, translate_end, lambda);
		osg::Quat rotation = OSGMathUtil::Lerp(rotate_start, rotate_end, lambda);

		m_node.SetMatrix(osg::Matrixf::rotate(rotation) * osg::Matrixf::scale(scale) * osg::Matrixf::translate(translate));
	}

	void MatrixAnimation::SetMatrix(const osg::Matrixf& end)
	{
		m_start = m_node.GetLocalMatrix();
		m_end = end;
	}
}