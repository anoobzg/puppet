#pragma once
#include "LauncaGeometryExport.h"
#include <math.h>
#include <map>
#include <vector>

namespace LauncaGeometry
{
	enum CurvatureType
	{
		MeanCurvature = 0, 
		GuassCurvature = 1,
		MinCurvature = 2,
		MaxCurvature = 3,
	};

	struct CurvatureInfo
	{
		double fAngle;
		double fArea;
		double fTempVector[3];
		double fNormalVector[3];

		CurvatureInfo(double Angle, double Area, double TempVector[3], double NormalVector[3]) : fAngle(Angle), fArea(Area)
		{
			fTempVector[0] = TempVector[0];
			fTempVector[1] = TempVector[1];
			fTempVector[2] = TempVector[2];

			fNormalVector[0] = NormalVector[0];
			fNormalVector[1] = NormalVector[1];
			fNormalVector[2] = NormalVector[2];
		}
	};

	class Mesh;
	class LAUNCA_GEOMETRY_API MeshCurvature
	{
	public:
		MeshCurvature(const Mesh& mesh);
		MeshCurvature(unsigned vn, unsigned fn, float* vertex_position, unsigned* triangle_index);

		bool CalCurvature(CurvatureType eMeanType, /*out*/ float* pCurvatureData);
		
		static double _calAngle(double edge0, double edge1, double edge2)
		{
			double cosValue = (edge0 * edge0 + edge1 * edge1 - edge2 * edge2) / (2 * edge0 * edge1);
			cosValue = (cosValue > 1.0) ? 1.0 : ((cosValue < -1.0) ? -1.0 : cosValue);
			return acos(cosValue);
		}

		template<typename T>
		static double _distance(T vertex1[3], T vertex2[3])
		{
			return sqrt(
				(vertex1[0] - vertex2[0]) * (vertex1[0] - vertex2[0]) +
				(vertex1[1] - vertex2[1]) * (vertex1[1] - vertex2[1]) +
				(vertex1[2] - vertex2[2]) * (vertex1[2] - vertex2[2])
			);
		}

		template<typename T>
		static T _vecDot(T vec3A[3], T vec3B[3])
		{
			return vec3A[0] * vec3B[0] + vec3A[1] * vec3B[1] + vec3A[2] * vec3B[2];
		}

		template<typename T>
		static void _vecCross(T vec3A[3], T vec3B[3], T outVec3[3])
		{
			outVec3[0] = vec3A[1] * vec3B[2] - vec3A[2] * vec3B[1];
			outVec3[1] = vec3A[2] * vec3B[0] - vec3A[0] * vec3B[2];
			outVec3[2] = vec3A[0] * vec3B[1] - vec3A[1] * vec3B[0];

			double length = sqrt(outVec3[0] * outVec3[0] + outVec3[1] * outVec3[1] + outVec3[2] * outVec3[2]);
			double rhLength = 1.0f / length;

			outVec3[0] = outVec3[0] * rhLength;
			outVec3[1] = outVec3[1] * rhLength;
			outVec3[2] = outVec3[2] * rhLength;
		}
		
		void _setupEdgeInfo();
		

	protected:

		bool _calTriangleInfo(float vertex0[3], float vertex1[3], float vertex2[3], double outAngleResult[3], double outAreaResult[3], double outVectorResult[3][3], double outNormalResult[3]);
	
	private:
		std::map<std::pair<unsigned, unsigned>, std::vector<unsigned>> m_edge;
		//const Mesh &m_mesh;
		const float PI = 3.1415926f;

		unsigned VN;
		unsigned FN;
		float* m_vertex_position;
		unsigned* m_triangle_index;
	};
}