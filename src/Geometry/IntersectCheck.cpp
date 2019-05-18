#include "IntersectCheck.h"
#include <math.h>

float _vecDot(float vec3A[3], float vec3B[3])
{
	return vec3A[0] * vec3B[0] + vec3A[1] * vec3B[1] + vec3A[2] * vec3B[2];
}

void _vecCross(float vec3A[3], float vec3B[3], float outVec3[3])
{
	outVec3[0] = vec3A[1] * vec3B[2] - vec3A[2] * vec3B[1];
	outVec3[1] = vec3A[2] * vec3B[0] - vec3A[0] * vec3B[2];
	outVec3[2] = vec3A[0] * vec3B[1] - vec3A[1] * vec3B[0];

	float length = sqrtf(outVec3[0] * outVec3[0] + outVec3[1] * outVec3[1] + outVec3[2] * outVec3[2]);
	float rhLength = 1.0f / length;

	outVec3[0] = outVec3[0] * rhLength;
	outVec3[1] = outVec3[1] * rhLength;
	outVec3[2] = outVec3[2] * rhLength;
}

bool LauncaGeometry::IntersectChecker::Intersect(float* rayCenter, float* rayDirection, float* vertices)
{
	float m_vertices[3][3] =
	{
		vertices[0], vertices[1], vertices[2],
		vertices[3], vertices[4], vertices[5],
		vertices[6], vertices[7], vertices[8]
	};

	float fDistance = -1.0f;
	float FLOAT_DEVIATION = 1e-5f;

	// Get triangle plane equation
	float edgeVec1[3] = { m_vertices[1][0] - m_vertices[0][0],  m_vertices[1][1] - m_vertices[0][1] , m_vertices[1][2] - m_vertices[0][2] };
	float edgeVec2[3] = { m_vertices[2][0] - m_vertices[1][0],  m_vertices[2][1] - m_vertices[1][1] , m_vertices[2][2] - m_vertices[1][2] };

	float normalVec[3];
	_vecCross(edgeVec1, edgeVec2, normalVec);
	float planeVertex[3] = { m_vertices[0][0], m_vertices[0][1], m_vertices[0][2] };

	// Check ray is parallel to triangle plane
	if (fabs(_vecDot(rayDirection, normalVec)) < FLOAT_DEVIATION)
	{
		return false;
	}
	/*
	Calculate intersection between ray and triangle plane
	plane equation: (P - planeVertex) * normalVec = 0.0f
	ray equation: P = ray.position + t * ray.direction

	===> t = normalVec * (planeVertex - ray.position) / normalVec * ray.Direction
	*/

	float tempVec[3] =
	{
		planeVertex[0] - rayCenter[0],
		planeVertex[1] - rayCenter[1],
		planeVertex[2] - rayCenter[2],
	};

	fDistance = _vecDot(normalVec, tempVec) / _vecDot(normalVec, rayDirection);
	if (fDistance < 0.0f)
	{
		return false;
	}

	float intersectPosition[3];
	intersectPosition[0] = rayCenter[0] + rayDirection[0] * fDistance;
	intersectPosition[1] = rayCenter[1] + rayDirection[1] * fDistance;
	intersectPosition[2] = rayCenter[2] + rayDirection[2] * fDistance;

	// Calculate intersection's triangle Barycentric Coordination
	double D =
		static_cast<double>(m_vertices[0][0]) * static_cast<double>(m_vertices[1][1]) * static_cast<double>(m_vertices[2][2]) +
		static_cast<double>(m_vertices[1][0]) * static_cast<double>(m_vertices[2][1]) * static_cast<double>(m_vertices[0][2]) +
		static_cast<double>(m_vertices[2][0]) * static_cast<double>(m_vertices[0][1]) * static_cast<double>(m_vertices[1][2]) -
		static_cast<double>(m_vertices[2][0]) * static_cast<double>(m_vertices[1][1]) * static_cast<double>(m_vertices[0][2]) -
		static_cast<double>(m_vertices[1][0]) * static_cast<double>(m_vertices[0][1]) * static_cast<double>(m_vertices[2][2]) -
		static_cast<double>(m_vertices[0][0]) * static_cast<double>(m_vertices[2][1]) * static_cast<double>(m_vertices[1][2]);

	if (fabs(static_cast<float>(D)) < FLOAT_DEVIATION)
	{
		return false;
	}

	double rhD = 1.0 / D;

	double Du =
		intersectPosition[0] * static_cast<double>(m_vertices[1][1]) * static_cast<double>(m_vertices[2][2]) +
		static_cast<double>(m_vertices[1][0]) * static_cast<double>(m_vertices[2][1]) * intersectPosition[2] +
		static_cast<double>(m_vertices[2][0]) * intersectPosition[1] * static_cast<double>(m_vertices[1][2]) -
		static_cast<double>(m_vertices[2][0]) * static_cast<double>(m_vertices[1][1]) * intersectPosition[2] -
		static_cast<double>(m_vertices[1][0]) * intersectPosition[1] * static_cast<double>(m_vertices[2][2]) -
		intersectPosition[0] * static_cast<double>(m_vertices[2][1]) * static_cast<double>(m_vertices[1][2]);

	double Dv =
		static_cast<double>(m_vertices[0][0]) * intersectPosition[1] * static_cast<double>(m_vertices[2][2]) +
		intersectPosition[0] * static_cast<double>(m_vertices[2][1]) * static_cast<double>(m_vertices[0][2]) +
		static_cast<double>(m_vertices[2][0]) * static_cast<double>(m_vertices[0][1]) * intersectPosition[2] -
		static_cast<double>(m_vertices[2][0]) * intersectPosition[1] * static_cast<double>(m_vertices[0][2]) -
		intersectPosition[0] * static_cast<double>(m_vertices[0][1]) * static_cast<double>(m_vertices[2][2]) -
		static_cast<double>(m_vertices[0][0]) * static_cast<double>(m_vertices[2][1]) * intersectPosition[2];

	double Dw =
		static_cast<double>(m_vertices[0][0]) * static_cast<double>(m_vertices[1][1]) * intersectPosition[2] +
		static_cast<double>(m_vertices[1][0]) * intersectPosition[1] * static_cast<double>(m_vertices[0][2]) +
		intersectPosition[0] * static_cast<double>(m_vertices[0][1]) * static_cast<double>(m_vertices[1][2]) -
		intersectPosition[0] * static_cast<double>(m_vertices[1][1]) * static_cast<double>(m_vertices[0][2]) -
		static_cast<double>(m_vertices[1][0]) * static_cast<double>(m_vertices[0][1]) * intersectPosition[2] -
		static_cast<double>(m_vertices[0][0]) * intersectPosition[1] * static_cast<double>(m_vertices[1][2]);

	double u = Du * rhD;
	double v = Dv * rhD;
	double w = Dw * rhD;

	// Outside
	if (u < 0.0 || v < 0.0 || w < 0.0)
	{
		return false;
	}

	//Logger::Log(LogType::INFO, Format("Intersection UVW = (%f, %f, %f)", outU, outV, outW));
	/*Logger::Log(LogType::INFO, Format("Intersection position = (%f, %f, %f)",
	intersectPosition.fX, intersectPosition.fY, intersectPosition.fZ));*/
	return true;
}
