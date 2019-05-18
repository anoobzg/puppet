#include "CollideAlgrithm.h"
#include <math.h>

namespace LauncaGeometry
{

	bool ColliderAlgrithm::RayCollideTriangle(float* ray_position, float* ray_direction, float* v0, float* v1, float* v2,
		/* out */float* uvw, /* out */ float* intersectCoord)
	{
		const float FLOAT_DEVIATION = 1e-6f;
		float fDistance = -1.0f;

		uvw[0] = 0.0f;
		uvw[1] = 0.0f;
		uvw[2] = 0.0f;

		intersectCoord[0] = 0.0f;
		intersectCoord[1] = 0.0f;
		intersectCoord[2] = 0.0f;

		// Get triangle plane equation
		float edgeVec1[3] = { v1[0] - v0[0],  v1[1] - v0[1] , v1[2] - v0[2] };
		float edgeVec2[3] = { v2[0] - v1[0],  v2[1] - v1[1] , v2[2] - v1[2] };

		float normalVec[3] = 
		{
			edgeVec1[1] * edgeVec2[2] - edgeVec1[2] * edgeVec2[1],
			edgeVec1[2] * edgeVec2[0] - edgeVec1[0] * edgeVec2[2],
			edgeVec1[0] * edgeVec2[1] - edgeVec1[1] * edgeVec2[0]
		};

		float fLength = sqrtf(normalVec[0] * normalVec[0] + normalVec[1] * normalVec[1] + normalVec[2] * normalVec[2]);
		normalVec[0] /= fLength;
		normalVec[1] /= fLength;
		normalVec[2] /= fLength;
		
		float planeVertex[3] = { v0[0], v0[1], v0[2] };

		// Check ray is parallel to triangle plane
		float fRDotN = ray_direction[0] * normalVec[0] + ray_direction[1] * normalVec[1] + ray_direction[2] * normalVec[2];
		if (-FLOAT_DEVIATION < fRDotN && fRDotN < FLOAT_DEVIATION)
		{
			return false;
		}

		/*
		Calculate intersection between ray and triangle plane
			plane equation:		normalVec * (P - planeVertex) = 0.0f
			ray equation:		P = ray.position + t * ray.direction

		===> t = normalVec * (planeVertex - ray.position) / normalVec * ray.Direction
		*/

		float tempVec[3] = 
		{
			planeVertex[0] - ray_position[0],
			planeVertex[1] - ray_position[1],
			planeVertex[2] - ray_position[2]
		};

		float fNDotR = normalVec[0] * ray_direction[0] + normalVec[1] * ray_direction[1] + normalVec[2] * ray_direction[2];
		fDistance = (normalVec[0] * tempVec[0] + normalVec[1] * tempVec[1] + normalVec[2] * tempVec[2]) / fNDotR;
		
		if (fDistance < 0.0f)
		{
			return false;
		}

		intersectCoord[0] = ray_position[0] + ray_direction[0] * fDistance;
		intersectCoord[1] = ray_position[1] + ray_direction[1] * fDistance;
		intersectCoord[2] = ray_position[2] + ray_direction[2] * fDistance;

		double dv0[3] = { v0[0], v0[1], v0[2]};
		double dv1[3] = { v1[0], v1[1], v1[2] };
		double dv2[3] = { v2[0], v2[1], v2[2] };

		// Calculate intersection's triangle Barycentric Coordination
		double D =
			dv0[0] * dv1[1] * dv2[2] +
			dv1[0] * dv2[1] * dv0[2] +
			dv2[0] * dv0[1] * dv1[2] -
			dv2[0] * dv1[1] * dv0[2] -
			dv1[0] * dv0[1] * dv2[2] -
			dv0[0] * dv2[1] * dv1[2];

		if (-FLOAT_DEVIATION < D && D < FLOAT_DEVIATION)
		{
			return false;
		}

		double rhD = 1.0 / D;

		double Du =
			intersectCoord[0] * dv1[1] * dv2[2] +
			dv1[0] * dv2[1] * intersectCoord[2] +
			dv2[0] * intersectCoord[1] * dv1[2] -
			dv2[0] * dv1[1] * intersectCoord[2] -
			dv1[0] * intersectCoord[1] * dv2[2] -
			intersectCoord[0] * dv2[1] * dv1[2];

		double Dv =
			dv0[0] * intersectCoord[1] * dv2[2] +
			intersectCoord[0] * dv2[1] * dv0[2] +
			dv2[0] * dv0[1] * intersectCoord[2] -
			dv2[0] * intersectCoord[1] * dv0[2] -
			intersectCoord[0] * dv0[1] * dv2[2] -
			dv0[0] * dv2[1] * intersectCoord[2];

		double Dw =
			dv0[0] * dv1[1] * intersectCoord[2] +
			dv1[0] * intersectCoord[1] * dv0[2] +
			intersectCoord[0] * dv0[1] * dv1[2] -
			intersectCoord[0] * dv1[1] * dv0[2] -
			dv1[0] * dv0[1] * intersectCoord[2] -
			dv0[0] * intersectCoord[1] * dv1[2];

		double u = Du * rhD;
		double v = Dv * rhD;
		double w = Dw * rhD;

		uvw[0] = static_cast<float>(u);
		uvw[1] = static_cast<float>(v);
		uvw[2] = static_cast<float>(w);

		return !(u < 0.0 || v < 0.0 || w < 0.0);
	}

}