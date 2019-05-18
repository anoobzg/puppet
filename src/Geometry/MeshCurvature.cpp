#include "MeshCurvature.h"
#include "Mesh.h"
#include <map>
#include <vector>
#include <set>
#include <math.h>
#include <iostream>

LauncaGeometry::MeshCurvature::MeshCurvature(const Mesh & mesh):
	VN(mesh.vertex_number), FN(mesh.triangle_number), m_vertex_position(mesh.vertex_position), m_triangle_index(mesh.triangle_index)
{
}

LauncaGeometry::MeshCurvature::MeshCurvature(unsigned vn, unsigned fn, float* vertex_position, unsigned* triangle_index)
	:VN(vn), FN(fn), m_vertex_position(vertex_position), m_triangle_index(triangle_index)
{

}

bool LauncaGeometry::MeshCurvature::_calTriangleInfo(float vertex0[3], float vertex1[3], float vertex2[3], double outAngleResult[3], double outAreaResult[3], double outVectorResult[3][3], double outNormalResult[3])
{
	// calculate angles
	double vertex0EdgeLength = _distance(vertex0, vertex1);
	double vertex1EdgeLength = _distance(vertex1, vertex2);
	double vertex2EdgeLength = _distance(vertex2, vertex0);

	double vertex0Angle = _calAngle(vertex0EdgeLength, vertex2EdgeLength, vertex1EdgeLength);
	double vertex1Angle = _calAngle(vertex0EdgeLength, vertex1EdgeLength, vertex2EdgeLength);
	double vertex2Angle = _calAngle(vertex1EdgeLength, vertex2EdgeLength, vertex0EdgeLength);

	if (abs(vertex0Angle) < FLT_EPSILON ||
		abs(vertex1Angle) < FLT_EPSILON ||
		abs(vertex2Angle) < FLT_EPSILON)
	{
		return false;
	}

	outAngleResult[0] = vertex0Angle;
	outAngleResult[1] = vertex1Angle;
	outAngleResult[2] = vertex2Angle;

	// calculate normal
	double edgeVec[3][3];
	edgeVec[0][0] = vertex1[0] - vertex0[0];
	edgeVec[0][1] = vertex1[1] - vertex0[1];
	edgeVec[0][2] = vertex1[2] - vertex0[2];

	edgeVec[1][0] = vertex2[0] - vertex1[0];
	edgeVec[1][1] = vertex2[1] - vertex1[1];
	edgeVec[1][2] = vertex2[2] - vertex1[2];

	edgeVec[2][0] = vertex0[0] - vertex2[0];
	edgeVec[2][1] = vertex0[1] - vertex2[1];
	edgeVec[2][2] = vertex0[2] - vertex2[2];

	_vecCross(edgeVec[0], edgeVec[1], outNormalResult);

	// calculate temp vector
	outVectorResult[0][0] = (edgeVec[2][0] * (1 / tan(vertex1Angle)) - edgeVec[0][0] * (1 / tan(vertex2Angle))) / 4.0f;
	outVectorResult[0][1] = (edgeVec[2][1] * (1 / tan(vertex1Angle)) - edgeVec[0][1] * (1 / tan(vertex2Angle))) / 4.0f;
	outVectorResult[0][2] = (edgeVec[2][2] * (1 / tan(vertex1Angle)) - edgeVec[0][2] * (1 / tan(vertex2Angle))) / 4.0f;

	outVectorResult[1][0] = (edgeVec[0][0] * (1 / tan(vertex2Angle)) - edgeVec[1][0] * (1 / tan(vertex0Angle))) / 4.0f;
	outVectorResult[1][1] = (edgeVec[0][1] * (1 / tan(vertex2Angle)) - edgeVec[1][1] * (1 / tan(vertex0Angle))) / 4.0f;
	outVectorResult[1][2] = (edgeVec[0][2] * (1 / tan(vertex2Angle)) - edgeVec[1][2] * (1 / tan(vertex0Angle))) / 4.0f;

	outVectorResult[2][0] = (edgeVec[1][0] * (1 / tan(vertex0Angle)) - edgeVec[2][0] * (1 / tan(vertex1Angle))) / 4.0f;
	outVectorResult[2][1] = (edgeVec[1][1] * (1 / tan(vertex0Angle)) - edgeVec[2][1] * (1 / tan(vertex1Angle))) / 4.0f;
	outVectorResult[2][2] = (edgeVec[1][2] * (1 / tan(vertex0Angle)) - edgeVec[2][2] * (1 / tan(vertex1Angle))) / 4.0f;

	int nAcuteAngleIndex = ((vertex0Angle > 0.5f * PI) ? 0 :
		(vertex1Angle > 0.5f * PI) ? 1 :
		(vertex2Angle > 0.5f * PI) ? 2 : -1);

	if (nAcuteAngleIndex == -1)
	{
		double fTempAngle[3];
		fTempAngle[0] = vertex0Angle;
		fTempAngle[1] = vertex1Angle;
		fTempAngle[2] = vertex2Angle;

		double fTempTriArea[3];
		fTempTriArea[0] = 0.125f * vertex0EdgeLength * vertex0EdgeLength * (1.0f / tan(fTempAngle[2]));
		fTempTriArea[1] = 0.125f * vertex1EdgeLength * vertex1EdgeLength * (1.0f / tan(fTempAngle[0]));
		fTempTriArea[2] = 0.125f * vertex2EdgeLength * vertex2EdgeLength * (1.0f / tan(fTempAngle[1]));

		outAreaResult[0] = fTempTriArea[0] + fTempTriArea[2];
		outAreaResult[1] = fTempTriArea[0] + fTempTriArea[1];
		outAreaResult[2] = fTempTriArea[1] + fTempTriArea[2];
	}
	else
	{
		double fP = 0.5f * (vertex0EdgeLength + vertex1EdgeLength + vertex2EdgeLength);
		double fTriArea = sqrt(fP * (fP - vertex0EdgeLength) * (fP - vertex1EdgeLength) * (fP - vertex2EdgeLength));

		outAreaResult[0] = 0.25f * fTriArea;
		outAreaResult[1] = 0.25f * fTriArea;
		outAreaResult[2] = 0.25f * fTriArea;
		outAreaResult[nAcuteAngleIndex] = 0.5f * fTriArea;
	}

	// Check
	for (unsigned j = 0; j < 3; j++)
	{
		if (std::isnan(outAreaResult[j]))
		{
			throw "NaN value.";
		}

		if (std::isinf(outAreaResult[j]))
		{
			throw "INF value.";
		}
	}

	for (unsigned j = 0; j < 3; j++)
	{
		for (unsigned k = 0; k < 3; k++)
		{
			if (std::isnan(outVectorResult[j][k]))
			{
				throw "NaN value.";
			}

			if (std::isinf(outVectorResult[j][k]))
			{
				throw "INF value.";
			}
		}
	}

	return true;
}

bool LauncaGeometry::MeshCurvature::CalCurvature(CurvatureType eMeanType, float* pCurvature)
{
	static bool bInitEdgeInfo = false;
	if (!bInitEdgeInfo)
	{
		_setupEdgeInfo();
		bInitEdgeInfo = true;
	}

	//return false;
	// Border vertex
	std::set<unsigned> borderVertexIndexSet;
	for (auto& edge : m_edge)
	{
		if (edge.second.size() == 1)
		{
			borderVertexIndexSet.insert(edge.first.first);
			borderVertexIndexSet.insert(edge.first.second);
		}
	}

	if (eMeanType == MaxCurvature || eMeanType == MinCurvature)
	{
		float* pGaussionCurvature = new float[VN];
		float* pMeanCurvature = new float[VN];

		if (!CalCurvature(GuassCurvature, pGaussionCurvature))
		{
			return false;
		}

		if (!CalCurvature(MeanCurvature, pMeanCurvature))
		{
			return false;
		}

		unsigned nErrorDeltaCount = 0;
		for (unsigned i = 0; i < VN; i++)
		{
			if (borderVertexIndexSet.count(i) > 0)
			{
				pCurvature[i] = 0.0f;
				continue;
			}

			float delta = pMeanCurvature[i] * pMeanCurvature[i] - pGaussionCurvature[i];

			if (delta < 0.0f)
			{
				delta = 0.0f;
				nErrorDeltaCount++;
			}

			if (eMeanType == MaxCurvature)
			{
				pCurvature[i] = pMeanCurvature[i] + sqrtf(delta);
			}
			else if (eMeanType == MinCurvature)
			{
				pCurvature[i] = pMeanCurvature[i] - sqrtf(delta);
			}
		}

		delete[] pGaussionCurvature;
		delete[] pMeanCurvature;

		return true;
	}


	std::vector<std::vector<CurvatureInfo>> _vertexCurvatureInfo;
	_vertexCurvatureInfo.resize(VN);
	auto pCurrentTriangleData = reinterpret_cast<char*>(m_triangle_index);

	for (unsigned i = 0; i < FN; i++)
	{
		unsigned vertexIndexs[3];
		memcpy(vertexIndexs, pCurrentTriangleData, 3 * sizeof(unsigned));

		double fAngleResult[3];
		double fAreaResult[3];

		float position[3][3];
		memcpy(position[0], m_vertex_position + 3 * vertexIndexs[0], 3 * sizeof(float));
		memcpy(position[1], m_vertex_position + 3 * vertexIndexs[1], 3 * sizeof(float));
		memcpy(position[2], m_vertex_position + 3 * vertexIndexs[2], 3 * sizeof(float));

		double fVectorResult[3][3];
		double fNormalResult[3];

		if (_calTriangleInfo(position[0], position[1], position[2], fAngleResult, fAreaResult, fVectorResult, fNormalResult))
		{
			_vertexCurvatureInfo[vertexIndexs[0]].push_back({ fAngleResult[0], fAreaResult[0], fVectorResult[0], fNormalResult });
			_vertexCurvatureInfo[vertexIndexs[1]].push_back({ fAngleResult[1], fAreaResult[1], fVectorResult[1], fNormalResult });
			_vertexCurvatureInfo[vertexIndexs[2]].push_back({ fAngleResult[2], fAreaResult[2], fVectorResult[2], fNormalResult });
		}

		pCurrentTriangleData += 3 * sizeof(unsigned);
	}

	for (unsigned i = 0; i < _vertexCurvatureInfo.size(); i++)
	{
		if (borderVertexIndexSet.count(i) > 0)
		{
			pCurvature[i] = 0.0;
			continue;
		}

		float fTotalAngle = 0.0f;
		float fTotalArea = 0.0f;
		for (auto& info : _vertexCurvatureInfo[i])
		{
			fTotalAngle += info.fAngle;
			fTotalArea += info.fArea;
		}

		if (eMeanType == GuassCurvature)
		{
			pCurvature[i] = ((2 * PI - fTotalAngle) / fTotalArea);

			//if (borderVertexIndexSet.count(i) > 0) std::cout << pCurvature[i] << std::endl;
		}
		else if (eMeanType == MeanCurvature)
		{
			double vertexAvgNormal[3] = { 0.0, 0.0, 0.0 };
			for (auto& info : _vertexCurvatureInfo[i])
			{
				vertexAvgNormal[0] += info.fNormalVector[0];
				vertexAvgNormal[1] += info.fNormalVector[1];
				vertexAvgNormal[2] += info.fNormalVector[2];
			}

			double zeroVector[3] = { 0.0, 0.0, 0.0 };
			double fLength = _distance(zeroVector, vertexAvgNormal);

			vertexAvgNormal[0] /= fLength;
			vertexAvgNormal[1] /= fLength;
			vertexAvgNormal[2] /= fLength;

			double fTempVec[3] = {0.0f, 0.0f, 0.0f};

			for (auto& info : _vertexCurvatureInfo[i])
			{
				fTempVec[0] += info.fTempVector[0];
				fTempVec[1] += info.fTempVector[1];
				fTempVec[2] += info.fTempVector[2];
			}

			float fSignedValue = (_vecDot(fTempVec, vertexAvgNormal) > 0.0) ? 1.0f : -1.0f;
			fTempVec[0] /= fTotalArea;
			fTempVec[1] /= fTotalArea;
			fTempVec[2] /= fTotalArea;
			
			float value = _distance(fTempVec, zeroVector);

			pCurvature[i] = fSignedValue * value;
		}

		if (std::isnan(pCurvature[i]))
		{
			throw "NaN value.";
		}

		if (std::isinf(pCurvature[i]))
		{
			throw "INF value.";
		}
	}

	return true;
}

void LauncaGeometry::MeshCurvature::_setupEdgeInfo()
{
	{
		for (unsigned i = 0; i < FN; i++)
		{
			unsigned* pTriangleIndex = m_triangle_index + 3 * i;
			for (unsigned j = 0; j < 3; j++)
			{
				unsigned v0 = pTriangleIndex[j];
				unsigned v1 = pTriangleIndex[(j + 1) % 3];

				if (v0 > v1)
				{
					auto temp = v0;
					v0 = v1;
					v1 = temp;
				}

				std::pair<unsigned, unsigned> edge = std::make_pair(v0, v1);
				m_edge[edge].push_back(i);
			}
		}
	}
}
