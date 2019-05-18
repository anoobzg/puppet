#ifndef GEOMETRY_TRANSFORM
#define GEOMETRY_TRANSFORM

#include "LauncaGeometryExport.h"

namespace LauncaGeometry
{
	class LAUNCA_GEOMETRY_API GemTransform
	{
	public:
		GemTransform();
		~GemTransform();

		void SetIdentity();
		void PostMult(GemTransform& trans);

		float m_matrix[16];
	};
}

#endif // GEOMETRY_TRANSFORM