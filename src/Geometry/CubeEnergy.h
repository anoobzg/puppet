#pragma once
#include "Mesh.h"
#include "LauncaGeometryExport.h"
#include <float.h>
#include <math.h>
#include <memory.h>
#include <memory>
#include <vector>

namespace LauncaGeometry {
	struct Cube 
	{
		union Coord 
		{
			struct
			{ 
				float coord[3];
			};
			struct 
			{
				float fX;
				float fY;
				float fZ;
			};
		};

		Coord minCoord;
		Coord maxCoord;

		unsigned uHitCount;
		std::vector<std::shared_ptr<Cube>> children;

		Cube(float _minCoord[3], float _maxCoord[3]) : uHitCount(0)
		{
			memcpy(minCoord.coord, _minCoord, sizeof(float) * 3);
			memcpy(maxCoord.coord, _maxCoord, sizeof(float) * 3);
		}

		bool IsLeafCube()
		{
			return (children.size() == 0);
		}

		bool Intersect(float* rayCenter, float* rayDirection, float rayMaxLength)
		{
			// Check rayCenter is inside Cube
			if ((minCoord.fX <= rayCenter[0] && rayCenter[0] <= maxCoord.fX) &&
				(minCoord.fY <= rayCenter[1] && rayCenter[1] <= maxCoord.fY) &&
				(minCoord.fZ <= rayCenter[2] && rayCenter[2] <= maxCoord.fZ))
			{
				// Inside
				return true;
			};

			bool bSignedDirection = true;

			static auto _distance = [](float vertex1[3], float vertex2[3])
			{
				return sqrtf(
					(vertex1[0] - vertex2[0]) * (vertex1[0] - vertex2[0]) +
					(vertex1[1] - vertex2[1]) * (vertex1[1] - vertex2[1]) +
					(vertex1[2] - vertex2[2]) * (vertex1[2] - vertex2[2])
				);
			};
			
			auto distance0 = _distance(rayCenter, this->minCoord.coord);
			auto distance1 = _distance(rayCenter, this->maxCoord.coord);

			if (distance0 > rayMaxLength && distance1 > rayMaxLength)
			{
				return false;
			}

			const float FLOAT_DEVIATION = 1e-5f;
			const float FLOAT_INFINITE = FLT_MAX;

			bool bIsRayDirXZero = fabs(rayDirection[0]) < FLOAT_DEVIATION;
			bool bIsRayDirYZero = fabs(rayDirection[1]) < FLOAT_DEVIATION;
			bool bIsRayDirZZero = fabs(rayDirection[2]) < FLOAT_DEVIATION;

			if (bIsRayDirXZero)
			{
				if (rayCenter[0] < minCoord.fX || rayCenter[0] > maxCoord.fX)
				{
					return false;
				}
			}

			if (bIsRayDirYZero)
			{
				if (rayCenter[1] < minCoord.fY || rayCenter[1] > maxCoord.fY)
				{
					return false;
				}
			}

			if (bIsRayDirZZero)
			{
				if (rayCenter[2] < minCoord.fZ || rayCenter[2] > maxCoord.fZ)
				{
					return false;
				}
			}

			float fRhDirectionX = (bIsRayDirXZero) ? FLOAT_INFINITE : 1.0f / rayDirection[0];
			float fRhDirectionY = (bIsRayDirYZero) ? FLOAT_INFINITE : 1.0f / rayDirection[1];
			float fRhDirectionZ = (bIsRayDirZZero) ? FLOAT_INFINITE : 1.0f / rayDirection[2];

			float fMinX = (bIsRayDirXZero) ? -FLOAT_INFINITE : (minCoord.fX - rayCenter[0]) * fRhDirectionX;
			float fMaxX = (bIsRayDirXZero) ? FLOAT_INFINITE : (maxCoord.fX - rayCenter[0]) * fRhDirectionX;
			float fMinY = (bIsRayDirYZero) ? -FLOAT_INFINITE : (minCoord.fY - rayCenter[1]) * fRhDirectionY;
			float fMaxY = (bIsRayDirYZero) ? FLOAT_INFINITE : (maxCoord.fY - rayCenter[1]) * fRhDirectionY;
			float fMinZ = (bIsRayDirZZero) ? -FLOAT_INFINITE : (minCoord.fZ - rayCenter[2]) * fRhDirectionZ;
			float fMaxZ = (bIsRayDirZZero) ? FLOAT_INFINITE : (maxCoord.fZ - rayCenter[2]) * fRhDirectionZ;

			float fMinStep = fmax(fmax(fmin(fMinX, fMaxX), fmin(fMinY, fMaxY)), fmin(fMinZ, fMaxZ));
			float fMaxStep = fmin(fmin(fmax(fMinX, fMaxX), fmax(fMinY, fMaxY)), fmax(fMinZ, fMaxZ));

			if (!bSignedDirection)
			{
				// NoIntersect situation
				if (fMinStep * fMaxStep < 0.0f)
				{
					return false;
				}

				if (fMinStep < 0.0f && fMinStep > fMaxStep)
				{
					return false;
				}

				if (fMinStep > 0.0f && fMinStep > fMaxStep)
				{
					return false;
				}
			}

			else
			{
				// NoIntersect situation
				if (fMaxStep < 0.0f)
				{
					return false;
				}

				if (fMinStep > fMaxStep)
				{
					return false;
				}
			}

			return true;
		}
	};

	class LAUNCA_GEOMETRY_API CubeEnergy
	{
	public:
		CubeEnergy(const Mesh& mesh): m_mesh(mesh) {}
		bool GetSubCubeEnergy(unsigned sliceCount, float* outSubCubeMinCoord, float* outSubCubeMaxCoord, unsigned* outSubCubeEnergy);
		static bool CalTriangleNormal(float* vertexCoord, float* outNormal);

	private:
		const Mesh& m_mesh;
	};
}