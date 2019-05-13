#ifndef MATRIX_DELTA_ANIMATION
#define MATRIX_DELTA_ANIMATION
#include <osgWrapper/Animation.h>
#include <osgWrapper/ManipulableNode.h>

namespace OSGWrapper
{
	class OSG_EXPORT MatrixDeltaAnimation : public Animation
	{
	public:
		MatrixDeltaAnimation(ManipulableNode& node);
		~MatrixDeltaAnimation();

		void OnPlay(float lambda);
	private:
		ManipulableNode& m_node;
	};
}
#endif // MATRIX_DELTA_ANIMATION