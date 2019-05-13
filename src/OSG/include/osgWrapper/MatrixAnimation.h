#ifndef MATRIX_ANIMATION
#define MATRIX_ANIMATION
#include <osgWrapper\Animation.h>
#include <osgWrapper\ManipulableNode.h>

namespace OSGWrapper
{
	class OSG_EXPORT MatrixAnimation : public Animation
	{
	public:
		MatrixAnimation(ManipulableNode& node);
		~MatrixAnimation();

		void SetMatrix(const osg::Matrixf& end);
		void OnPlay(float lambda);
	private:
		ManipulableNode& m_node;

		osg::Matrixf m_start;
		osg::Matrixf m_end;
	};
}
#endif // MATRIX_ANIMATION