#include "GemTransform.h"

namespace LauncaGeometry
{
	GemTransform::GemTransform()
	{
		SetIdentity();
	}

	GemTransform::~GemTransform()
	{
	}

	void GemTransform::SetIdentity()
	{
		for(unsigned i = 0; i < 4; ++i)
		{
			for(unsigned j = 0; j < 4; ++j)
			{
				float value = i == j ? 1.0f : 0.0f;
				m_matrix[4*i + j] = value;
			}
		}
	}

	void GemTransform::PostMult(GemTransform& trans)
	{
		float *l = m_matrix;
		float *r = trans.m_matrix;
		float m0 = l[0] * r[0] + l[1] * r[4] + l[2] * r[8];
		float m1 = l[0] * r[1] + l[1] * r[5] + l[2] * r[9];
		float m2 = l[0] * r[2] + l[1] * r[6] + l[2] * r[10];
		float m4 = l[4] * r[0] + l[5] * r[4] + l[6] * r[8];
		float m5 = l[4] * r[1] + l[5] * r[5] + l[6] * r[9];
		float m6 = l[4] * r[2] + l[5] * r[6] + l[6] * r[10];
		float m8 = l[8] * r[0] + l[9] * r[4] + l[10] * r[8];
		float m9 = l[8] * r[1] + l[9] * r[5] + l[10] * r[9];
		float m10 = l[8] * r[2] + l[9] * r[6] + l[10] * r[10];

		float m3 = l[0] * r[3] + l[1] * r[7] + l[2] * r[11] + l[3] * r[15];
		float m7 = l[4] * r[3] + l[5] * r[7] + l[6] * r[11] + l[7] * r[15];
		float m11 = l[8] * r[3] + l[9] * r[7] + l[10] * r[11] + l[11] * r[15];

		l[0] = m0;
		l[1] = m1;
		l[2] = m2;
		l[3] = m3;
		l[4] = m4;
		l[5] = m5;
		l[6] = m6;
		l[7] = m7;
		l[8] = m8;
		l[9] = m9;
		l[10] = m10;
		l[11] = m11;
	}
}