#include "CubeEnergy.h"
#include <vector>
#include <iostream>
#include <set>

bool _constructCubeTree(LauncaGeometry::Cube& cube, unsigned uCurrentDepth, unsigned uMaxDepth)
{
	if (uCurrentDepth > uMaxDepth)
	{
		return true;
	}

	float coordDelta[3] =
	{
		0.5f * (cube.maxCoord.fX - cube.minCoord.fX),
		0.5f * (cube.maxCoord.fY - cube.minCoord.fY),
		0.5f * (cube.maxCoord.fZ - cube.minCoord.fZ)
	};

	// Construct subcube node
	for (unsigned i = 0; i < 8; i++)
	{
		bool bXTemp = static_cast<bool>(i & 0x1);
		bool bYTemp = static_cast<bool>(i & 0x2);
		bool bZTemp = static_cast<bool>(i & 0x4);

		float minCoord[3];
		float maxCoord[3];

		if (bXTemp)
		{
			minCoord[0] = cube.minCoord.fX + coordDelta[0];
			maxCoord[0] = cube.maxCoord.fX;
		}
		else
		{
			minCoord[0] = cube.minCoord.fX;
			maxCoord[0] = cube.minCoord.fX + coordDelta[0];
		}

		if (bYTemp)
		{
			minCoord[1] = cube.minCoord.fY + coordDelta[1];
			maxCoord[1] = cube.maxCoord.fY;
		}
		else
		{
			minCoord[1] = cube.minCoord.fY;
			maxCoord[1] = cube.minCoord.fY + coordDelta[1];
		}

		if (bZTemp)
		{
			minCoord[2] = cube.minCoord.fZ + coordDelta[2];
			maxCoord[2] = cube.maxCoord.fZ;
		}
		else
		{
			minCoord[2] = cube.minCoord.fZ;
			maxCoord[2] = cube.minCoord.fZ + coordDelta[2];
		}

		std::shared_ptr<LauncaGeometry::Cube> subcube = std::make_shared<LauncaGeometry::Cube>(minCoord, maxCoord);
		cube.children.push_back(subcube);
	}

	for (auto& subCube : cube.children)
	{
		_constructCubeTree(*subCube, uCurrentDepth + 1, uMaxDepth);
	}

	return true;
};

bool LauncaGeometry::CubeEnergy::GetSubCubeEnergy(unsigned sliceCount, float* outSubCubeMinCoord, float* outSubCubeMaxCoord, unsigned* outSubCubeEnergy)
{
	// Check valid
	if (m_mesh.vertex_number < 4)
	{
		std::cout << "LauncaGeometry::CubeEnergy::GetMaxEnergyCubeCenterCoord --> mesh vertex number < 4" << std::endl;
		return false;
	}

	// Get mesh BoundingBox
	float fMinCoord[3] = { m_mesh.vertex_position[0] , m_mesh.vertex_position[1], m_mesh.vertex_position[2] };
	float fMaxCoord[3] = { m_mesh.vertex_position[0] , m_mesh.vertex_position[1], m_mesh.vertex_position[2] };;
	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		float* pVertexCoord = m_mesh.vertex_position + 3 * i;
		fMinCoord[0] = fmin(fMinCoord[0], pVertexCoord[0]);
		fMinCoord[1] = fmin(fMinCoord[1], pVertexCoord[1]);
		fMinCoord[2] = fmin(fMinCoord[2], pVertexCoord[2]);

		fMaxCoord[0] = fmax(fMaxCoord[0], pVertexCoord[0]);
		fMaxCoord[1] = fmax(fMaxCoord[1], pVertexCoord[1]);
		fMaxCoord[2] = fmax(fMaxCoord[2], pVertexCoord[2]);
	}

	// Construct cube node Tree
	std::shared_ptr<LauncaGeometry::Cube> rootCube = std::make_shared<LauncaGeometry::Cube>(fMinCoord, fMaxCoord);
	float CoordDelta[3] =
	{
		fMaxCoord[0] - fMinCoord[0],
		fMaxCoord[1] - fMinCoord[1],
		fMaxCoord[2] - fMinCoord[2]
	};

	_constructCubeTree(*rootCube, 0, sliceCount);

	// Random select some triangle for energy calculate
	unsigned uStep = m_mesh.triangle_number / 200000;
	uStep = (uStep > 0) ? uStep : 1;

	for (unsigned i = 0; i < m_mesh.triangle_number; i += uStep)
	{
		unsigned* vertices = m_mesh.triangle_index + 3 * i;
		float vertexCoord[3][3];
		memcpy(vertexCoord[0], m_mesh.vertex_position + 3 * vertices[0], sizeof(float) * 3);
		memcpy(vertexCoord[1], m_mesh.vertex_position + 3 * vertices[1], sizeof(float) * 3);
		memcpy(vertexCoord[2], m_mesh.vertex_position + 3 * vertices[2], sizeof(float) * 3);

		// Calculate triangle normal
		float outNormal[3];
		CalTriangleNormal(reinterpret_cast<float*>(vertexCoord), outNormal);

		// Inverse normal
		outNormal[0] = -outNormal[0];
		outNormal[1] = -outNormal[1];
		outNormal[2] = -outNormal[2];

		// SubCube intersect check
		std::vector<std::shared_ptr<LauncaGeometry::Cube>> cubeStack;
		cubeStack.push_back(rootCube);

		while (cubeStack.size() > 0)
		{
			auto pCube = cubeStack.front();
			cubeStack.erase(cubeStack.begin());

			if (pCube->Intersect(vertexCoord[0], outNormal, 5))
			{
				pCube->uHitCount++;
				cubeStack.insert(cubeStack.end(), pCube->children.begin(), pCube->children.end());
			}
		}
	}

	std::set<std::shared_ptr<Cube>> cubeStack;
	cubeStack.insert(rootCube);

	std::vector<std::shared_ptr<Cube>> leafCubeVec;

	while (cubeStack.size() > 0)
	{
		auto pCube = *(cubeStack.begin());
		cubeStack.erase(cubeStack.begin());

		if (pCube->IsLeafCube())
		{
			leafCubeVec.push_back(pCube);
			continue;
		}

		for (auto child : pCube->children)
		{
			cubeStack.insert(child);
		}
	}

	for (auto leafCube : leafCubeVec)
	{
		memcpy(outSubCubeMinCoord, leafCube->minCoord.coord, sizeof(float) * 3);
		memcpy(outSubCubeMaxCoord, leafCube->maxCoord.coord, sizeof(float) * 3);
		outSubCubeEnergy[0] = leafCube->uHitCount;

		outSubCubeMinCoord +=  3;
		outSubCubeMaxCoord +=  3;
		outSubCubeEnergy ++;
	}

	return true;
}

bool LauncaGeometry::CubeEnergy::CalTriangleNormal(float* vertexCoord, float* outNormal)
{
	float edge[2][3] = 
	{
		vertexCoord[3] - vertexCoord[0], vertexCoord[4] - vertexCoord[1], vertexCoord[5] - vertexCoord[2],
		vertexCoord[6] - vertexCoord[3], vertexCoord[7] - vertexCoord[4], vertexCoord[8] - vertexCoord[5]
	};

	static auto _vecCross = [](float vec3A[3], float vec3B[3], float outVec3[3])
	{
		outVec3[0] = vec3A[1] * vec3B[2] - vec3A[2] * vec3B[1];
		outVec3[1] = vec3A[2] * vec3B[0] - vec3A[0] * vec3B[2];
		outVec3[2] = vec3A[0] * vec3B[1] - vec3A[1] * vec3B[0];

		float length = sqrtf(outVec3[0] * outVec3[0] + outVec3[1] * outVec3[1] + outVec3[2] * outVec3[2]);
		float rhLength = 1.0f / length;

		outVec3[0] = outVec3[0] * rhLength;
		outVec3[1] = outVec3[1] * rhLength;
		outVec3[2] = outVec3[2] * rhLength;
	};

	_vecCross(edge[0], edge[1], outNormal);

	return true;
}
