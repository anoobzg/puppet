#include <memory>
#include <iostream>
#include <algorithm>
#include <Eigenvalues> 
#include <iostream>
#include <fstream>

#include "CollideAlgrithm.h"
#include "MeshCurvature.h"
#include "ToothSegmentationModule.h"
#include "Base.h"
#include "CubeEnergy.h"
#include "harmonic_caculator.h"

using namespace LauncaGeometry;

double _distanceOfPointToPlane(const Mesh& mesh, unsigned uVertexIndex, float* planeNormal, float* planeCenter)
{
	float *vertexPosition = mesh.vertex_position + uVertexIndex * 3;

	double tempVec[3];
	tempVec[0] = vertexPosition[0] - planeCenter[0];
	tempVec[1] = vertexPosition[1] - planeCenter[1];
	tempVec[2] = vertexPosition[2] - planeCenter[2];

	double normal[3];
	normal[0] = planeNormal[0];
	normal[1] = planeNormal[1];
	normal[2] = planeNormal[2];

	return MeshCurvature::_vecDot(tempVec, normal);
};

float _distanceOfPointToPoint(const Mesh& mesh, unsigned uVertexIndex0, unsigned uVertexIndex1)
{
	float* vertexCoord0 = mesh.vertex_position + 3 * uVertexIndex0;
	float* vertexCoord1 = mesh.vertex_position + 3 * uVertexIndex1;

	float distances[3] =
	{
		vertexCoord0[0] - vertexCoord1[0],
		vertexCoord0[1] - vertexCoord1[1],
		vertexCoord0[2] - vertexCoord1[2],
	};

	return sqrtf(distances[0] * distances[0] + distances[1] * distances[1] + distances[2] * distances[2]);
}

bool _isDegenerateTriangle(float** _fVertexCoord)
{
	float vertexCoord[3][3];
	memcpy(vertexCoord, _fVertexCoord, sizeof(vertexCoord));

	static auto _isSameVertex = [](float* vertex0, float* vertex1)
	{
		if (vertex0[0] == vertex1[0] &&
			vertex0[1] == vertex1[1] &&
			vertex0[2] == vertex1[2])
		{
			return true;
		}

		return false;
	};

	if (_isSameVertex(vertexCoord[0], vertexCoord[1]) ||
		_isSameVertex(vertexCoord[0], vertexCoord[2]) ||
		_isSameVertex(vertexCoord[1], vertexCoord[2]))
	{
		return true;
	}

	return false;
}

std::vector<std::set<unsigned>> ToothSegmentationModule::_groupVertexSetByConnection(std::set<unsigned> vertexSet)
{
	std::vector<std::set<unsigned>> Result;
	if (!_groupVertexSetByConnection(vertexSet, Result))
	{
		throw "FATAL ERROR";
	}

	return Result;
};

bool ToothSegmentationModule::_groupVertexSetByConnection(std::set<unsigned> vertexSet, std::vector<std::set<unsigned>>& outResult)
{
	for (auto it = vertexSet.begin(); it != vertexSet.end();)
	{
		std::set<unsigned> groupVertexSet;
		std::set<unsigned> stack;
		stack.insert(*it);
		groupVertexSet.insert(*it);

		while (stack.size() > 0)
		{
			auto _it = stack.begin();
			groupVertexSet.insert(*_it);
			vertexSet.erase(*_it);

			auto& adjVertexSet = m_vertexAdjInfoVec[*_it];
			for (auto& adjVertex : adjVertexSet)
			{
				if (groupVertexSet.count(adjVertex) > 0)
				{
					continue;
				}

				if (vertexSet.count(adjVertex) > 0)
				{
					stack.insert(adjVertex);
				}
			}

			stack.erase(_it);
		}

		outResult.push_back(groupVertexSet);
		it = vertexSet.begin();
	}

	return true;
}

bool ToothSegmentationModule::_getMaxSizeVertexGroupByConnection(std::set<unsigned> vertexSet, std::set<unsigned>& outResult)
{
	unsigned uLimitSize = vertexSet.size() * 0.5f;

	for (auto it = vertexSet.begin(); it != vertexSet.end();)
	{
		std::set<unsigned> groupVertexSet;
		std::set<unsigned> stack;
		stack.insert(*it);
		groupVertexSet.insert(*it);

		while (stack.size() > 0)
		{
			auto _it = stack.begin();
			groupVertexSet.insert(*_it);
			vertexSet.erase(*_it);

			auto& adjVertexSet = m_vertexAdjInfoVec[*_it];
			for (auto& adjVertex : adjVertexSet)
			{
				if (groupVertexSet.count(adjVertex) > 0)
				{
					continue;
				}

				if (vertexSet.count(adjVertex) > 0)
				{
					stack.insert(adjVertex);
				}
			}

			stack.erase(_it);
		}

		if (groupVertexSet.size() > uLimitSize)
		{
			outResult = groupVertexSet;
			break;
		}

		it = vertexSet.begin();
	}

	return true;
}

bool ToothSegmentationModule::_getHeadAreaVertexGroupByConnection(std::set<unsigned> vertexSet, std::set<unsigned>& outResult)
{
	// Find max height vertex
	float fMaxHeight = 0.0f;
	int fMaxHeightVertexIndex = -1;
	for (auto vertexIndex : vertexSet)
	{
		if (fMaxHeight < m_vertexToPlaneDistance[vertexIndex])
		{
			fMaxHeight = m_vertexToPlaneDistance[vertexIndex];
			fMaxHeightVertexIndex = vertexIndex;
		}
	}

	if (fMaxHeightVertexIndex == -1)
	{
		std::cout << "[ERROR] Can not find max height vertex index." << std::endl;
		return false;
	}

	std::set<unsigned> groupVertexSet;
	std::set<unsigned> stack;
	stack.insert(fMaxHeightVertexIndex);
	groupVertexSet.insert(fMaxHeightVertexIndex);

	while (stack.size() > 0)
	{
		auto _it = stack.begin();
		groupVertexSet.insert(*_it);
		vertexSet.erase(*_it);

		auto& adjVertexSet = m_vertexAdjInfoVec[*_it];
		for (auto& adjVertexIndex : adjVertexSet)
		{
			if (groupVertexSet.count(adjVertexIndex) > 0)
			{
				continue;
			}

			if (vertexSet.count(adjVertexIndex) > 0)
			{
				stack.insert(adjVertexIndex);
			}
		}

		stack.erase(_it);
	}

	outResult = groupVertexSet;

	return true;
}

bool ToothSegmentationModule::_isOneConnectedRegion(std::set<unsigned> vertexSet)
{
	auto it = vertexSet.begin();
	std::set<unsigned> groupVertexSet;
	std::set<unsigned> stack;
	stack.insert(*it);
	groupVertexSet.insert(*it);

	while (stack.size() > 0)
	{
		auto _it = stack.begin();
		groupVertexSet.insert(*_it);
		vertexSet.erase(*_it);

		auto& adjVertexSet = m_vertexAdjInfoVec[*_it];
		for (auto& adjVertex : adjVertexSet)
		{
			if (groupVertexSet.count(adjVertex) > 0)
			{
				continue;
			}

			if (vertexSet.count(adjVertex) > 0)
			{
				stack.insert(adjVertex);
			}
		}

		stack.erase(_it);
	}

	return (vertexSet.size() == 0);
}

ToothSegmentationModule::ToothSegmentationModule(Mesh& mesh, ToothSegmentScene* pScene) : m_mesh(mesh), m_pScene(pScene), m_nSampleVertexCount(1000), m_harmonicCalculator(new HarmonicCaculator(mesh))
{
	_initMeshDensity();
	if (!m_shortestPathProxy.SetupGraph(m_mesh.vertex_number, m_mesh.vertex_position, m_mesh.triangle_number, m_mesh.triangle_index, true))
	{
		throw "Init shortestpath dll failed.";
	}

	_setupMeshInfo(m_mesh);
	_constructAlgPipeLine();
	_exportPlaneInfoAndFeatureInfo();
	_updateAlgorithmParameters("parameter.cfg");
}

bool ToothSegmentationModule::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_Z)
	{
		static float m_originalOffset = m_clipPlaneOffset;
		static bool bShowMaxEnergyPlane = true;

		m_clipPlaneOffset = (bShowMaxEnergyPlane) ? m_maxEnergyPlaneOffset : m_originalOffset;
		UpdateClipPlaneCenter();

		if (!m_pScene->SetPlaneGeode(false, false))
		{
			return false;
		}

		bShowMaxEnergyPlane = !bShowMaxEnergyPlane;
	}

	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_Q)
	{
		_exportEnergyVertexInfo(m_boundaryVertexVec, m_zeroGroups, m_oneGroups);
	}

	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_C)
	{
		m_clipPlaneOffset += 0.05f;
		UpdateClipPlaneCenter();
		unsigned uLoopCount;
		std::cout << "Energy:" << CalPlaneEnergy(m_mesh, m_clipPlaneNormal, m_clipPlaneCenter, uLoopCount, true, true) << std::endl;
		m_pScene->SetPlaneGeode(false, false);
	}

	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_V)
	{
		m_clipPlaneOffset -= 0.05f;
		UpdateClipPlaneCenter();
		unsigned uLoopCount;
		std::cout << "Energy:" << CalPlaneEnergy(m_mesh, m_clipPlaneNormal, m_clipPlaneCenter, uLoopCount, true, true) << std::endl;
		m_pScene->SetPlaneGeode(false, false);
	}

	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_Space)
	{
		m_algorithmPipeLine.RunFirstTask();
	}

	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_Z)
	{
		_getBoundarySkeleton(m_minMeanCurvature * m_fRegionGrowCurvatureRatio, true, true);
	}

	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_G)
	{
		m_algorithmPipeLine.RunAllTask();
	}

	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_R)
	{
		_resetAlgorithmState();
	}

	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_Y)
	{
		_reFindBoundary(true);
		_regionGrowUtilOneRegion(true);
	}

	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_T)
	{
		float* harmonic = new float[m_mesh.vertex_number];
		auto currentLoopSet = _generateBoundaryVertexSet(m_boundaryVertexVec);
		
		m_harmonicCalculator->DoParallPatch(m_originalFeatureGroups, std::vector<unsigned>(currentLoopSet.begin(), currentLoopSet.end()), harmonic);

		static auto writeUINTtoFstream = [](std::ofstream& outfstream, unsigned value)
		{
			outfstream.write(reinterpret_cast<char*>(&value), sizeof(unsigned));
		};

		// export file
		std::ofstream outFile("info2.bin", std::ios::out | std::ios::binary);
		if (outFile.is_open())
		{
			writeUINTtoFstream(outFile, currentLoopSet.size());

			for (auto vertexIndex : currentLoopSet)
			{
				writeUINTtoFstream(outFile, vertexIndex);
			}

			writeUINTtoFstream(outFile, m_originalFeatureGroups.size());
			for (auto& group : m_originalFeatureGroups)
			{
				writeUINTtoFstream(outFile, group.size());
				for (auto vertexIndex : group)
				{
					writeUINTtoFstream(outFile, vertexIndex);
				}
			}

			outFile.close();
		}
		
		for (unsigned i = 0; i < m_mesh.vertex_number; i++)
		{
			m_currentMeanCurvatureVec[i].second = (harmonic[i] > 0.5f + FLT_EPSILON) ? m_minMeanCurvature : 0.0f;
		}

		delete[] harmonic;

		m_pScene->ShowCurvature();
		m_pScene->UpdateSamplePointGeode(std::vector<unsigned>(currentLoopSet.begin(), currentLoopSet.end()));
	}

	return true;
}

bool ToothSegmentationModule::CalClipPlaneNormal(bool bShowSampleVertex /*= true*/)
{
	// m_nSampleVertexCount = (m_mesh.vertex_number / 500.0f) * m_fMeshDensity;

	m_nSampleVertexCount = 10.0f * m_fMeshDensity;
	float fSampleVertexMinDistance = 3.0f;

	static bool bNormalInited = false;
	if (bNormalInited)
	{
		return true;
	}

	_sortMeanCurvatureVec(false);

	static auto _calCovariance = [](float* valueA, float avgA, float* valueB, float avgB, unsigned uCount)
	{
		float fTemp = 0.0f;

		for (unsigned i = 0; i < uCount; i++)
		{
			fTemp += (valueA[i] - avgA) * (valueB[i] - avgB);
		}

		return fTemp / uCount;
	};

	float* Xcoords = new float[m_nSampleVertexCount];
	float* Ycoords = new float[m_nSampleVertexCount];
	float* Zcoords = new float[m_nSampleVertexCount];

	if (!Xcoords || !Ycoords || !Zcoords)
	{
		std::cout << "Malloc heap memory failed." << std::endl;
		return false;
	}

	float fAvgX = 0.0f;
	float fAvgY = 0.0f;
	float fAvgZ = 0.0f;

	unsigned uVertexCount = 0;
	for (unsigned uLoopCount = 0; uVertexCount < m_nSampleVertexCount; uLoopCount++)
	{
		if (uLoopCount >= m_mesh.vertex_number)
		{
			break;
		}

		unsigned vertexIndex = m_currentMeanCurvatureVec[uLoopCount].first;
		
		if (m_currentMeanCurvatureVec[uLoopCount].second >= 0.0f)
		{
			break;
		}

		bool bTooNear = false;
		for (auto _vertexIndex : m_sampleVertexIndexVec)
		{
			if (_distanceOfPointToPoint(m_mesh, _vertexIndex, vertexIndex) < fSampleVertexMinDistance)
			{
				bTooNear = true;
				break;
			}
		}

		if (bTooNear)
		{
			continue;
		}

		float* vertexPosition = m_mesh.vertex_position + 3 * vertexIndex;
		Xcoords[uVertexCount] = vertexPosition[0];
		Ycoords[uVertexCount] = vertexPosition[1];
		Zcoords[uVertexCount] = vertexPosition[2];

		fAvgX += Xcoords[uVertexCount];
		fAvgY += Ycoords[uVertexCount];
		fAvgZ += Zcoords[uVertexCount];

		uVertexCount++;
		m_sampleVertexIndexVec.push_back(vertexIndex);
	}

	fAvgX /= uVertexCount;
	fAvgY /= uVertexCount;
	fAvgZ /= uVertexCount;

	float fMcovariance[3][3];
	fMcovariance[0][0] = _calCovariance(Xcoords, fAvgX, Xcoords, fAvgX, uVertexCount);
	fMcovariance[0][1] = _calCovariance(Xcoords, fAvgX, Ycoords, fAvgY, uVertexCount);
	fMcovariance[0][2] = _calCovariance(Xcoords, fAvgX, Zcoords, fAvgZ, uVertexCount);
	fMcovariance[1][0] = _calCovariance(Ycoords, fAvgY, Xcoords, fAvgX, uVertexCount);
	fMcovariance[1][1] = _calCovariance(Ycoords, fAvgY, Ycoords, fAvgY, uVertexCount);
	fMcovariance[1][2] = _calCovariance(Ycoords, fAvgY, Zcoords, fAvgZ, uVertexCount);
	fMcovariance[2][0] = _calCovariance(Zcoords, fAvgZ, Xcoords, fAvgX, uVertexCount);
	fMcovariance[2][1] = _calCovariance(Zcoords, fAvgZ, Ycoords, fAvgY, uVertexCount);
	fMcovariance[2][2] = _calCovariance(Zcoords, fAvgZ, Zcoords, fAvgZ, uVertexCount);

	delete[] Xcoords;
	delete[] Ycoords;
	delete[] Zcoords;

	Xcoords = nullptr;
	Ycoords = nullptr;
	Zcoords = nullptr;

	Eigen::Matrix3f Mcovariance;
	Mcovariance(0, 0) = fMcovariance[0][0];
	Mcovariance(0, 1) = fMcovariance[0][1];
	Mcovariance(0, 2) = fMcovariance[0][2];
	Mcovariance(1, 0) = fMcovariance[1][0];
	Mcovariance(1, 1) = fMcovariance[1][1];
	Mcovariance(1, 2) = fMcovariance[1][2];
	Mcovariance(2, 0) = fMcovariance[2][0];
	Mcovariance(2, 1) = fMcovariance[2][1];
	Mcovariance(2, 2) = fMcovariance[2][2];

	Eigen::EigenSolver<Eigen::Matrix3f> eigenSolver(Mcovariance, true);
	auto eigenVector = eigenSolver.eigenvectors();
	auto eigenValue = eigenSolver.eigenvalues().real();

	Eigen::MatrixXf::Index evalsMin;
	eigenValue.rowwise().sum().minCoeff(&evalsMin);

	m_clipPlaneNormal[0] = eigenVector(0, evalsMin).real();
	m_clipPlaneNormal[1] = eigenVector(1, evalsMin).real();
	m_clipPlaneNormal[2] = eigenVector(2, evalsMin).real();

	float minCoord[3];
	float maxCoord[3];
	bool bInitMinAndMax = false;

	for (unsigned i = 0; i < m_mesh.triangle_number; i++)
	{
		unsigned* vertexIndices = m_mesh.triangle_index + 3 * i;
		float vertexCoord[3][3];

		memcpy(vertexCoord[0], m_mesh.vertex_position + 3 * vertexIndices[0], 3 * sizeof(float));
		memcpy(vertexCoord[1], m_mesh.vertex_position + 3 * vertexIndices[1], 3 * sizeof(float));
		memcpy(vertexCoord[2], m_mesh.vertex_position + 3 * vertexIndices[2], 3 * sizeof(float));

		if (!bInitMinAndMax)
		{
			minCoord[0] = vertexCoord[0][0];
			minCoord[1] = vertexCoord[0][1];
			minCoord[2] = vertexCoord[0][2];

			maxCoord[0] = vertexCoord[0][0];
			maxCoord[1] = vertexCoord[0][1];
			maxCoord[2] = vertexCoord[0][2];

			bInitMinAndMax = true;
		}

		// Check Degenerate-triangle
		if (_isDegenerateTriangle(reinterpret_cast<float**>(vertexCoord)))
		{
			continue;
		}

		// BoundingBox
		for (unsigned j = 0; j < 3; j++)
		{
			for (unsigned k = 0; k < 3; k++)
			{
				if (vertexCoord[j][k] < minCoord[k])
				{
					minCoord[k] = vertexCoord[j][k];
				}

				if (vertexCoord[j][k] > maxCoord[k])
				{
					maxCoord[k] = vertexCoord[j][k];
				}
			}
		}
	}

	float fTempVecDotSum = 0.0f;
	unsigned uSampleCount = 10;
	for (unsigned i = 0; i < uSampleCount; i++)
	{
		unsigned* vertexIndices = m_mesh.triangle_index + 3 * m_currentMeanCurvatureVec[i].first;
		float vertexCoord[3][3];
		float normal[3];

		memcpy(vertexCoord[0], m_mesh.vertex_position + 3 * vertexIndices[0], 3 * sizeof(float));
		memcpy(vertexCoord[1], m_mesh.vertex_position + 3 * vertexIndices[1], 3 * sizeof(float));
		memcpy(vertexCoord[2], m_mesh.vertex_position + 3 * vertexIndices[2], 3 * sizeof(float));

		// Check Degenerate-triangle
		if (_isDegenerateTriangle(reinterpret_cast<float**>(vertexCoord)))
		{
			continue;
		}

		CubeEnergy::CalTriangleNormal(reinterpret_cast<float*>(vertexCoord), normal);
		fTempVecDotSum += _vecDot(m_clipPlaneNormal, normal);
	}

	fTempVecDotSum /= uSampleCount;
	if (fTempVecDotSum > 0.0f)
	{
		m_clipPlaneNormal[0] = -m_clipPlaneNormal[0];
		m_clipPlaneNormal[1] = -m_clipPlaneNormal[1];
		m_clipPlaneNormal[2] = -m_clipPlaneNormal[2];
	}

	// Calculate boundingSphere
	float boundingCenter[3] =
	{
		0.5f * (minCoord[0] + maxCoord[0]),
		0.5f * (minCoord[1] + maxCoord[1]),
		0.5f * (minCoord[2] + maxCoord[2]),
	};

	float boundingRadius = 0.5f * (sqrtf(
		(maxCoord[0] - minCoord[0]) * (maxCoord[0] - minCoord[0]) +
		(maxCoord[1] - minCoord[1]) * (maxCoord[1] - minCoord[1]) +
		(maxCoord[2] - minCoord[2]) * (maxCoord[2] - minCoord[2])
	));

	m_baseCenter[0] = boundingCenter[0] - 1.0f * m_clipPlaneNormal[0] * boundingRadius;
	m_baseCenter[1] = boundingCenter[1] - 1.0f * m_clipPlaneNormal[1] * boundingRadius;
	m_baseCenter[2] = boundingCenter[2] - 1.0f * m_clipPlaneNormal[2] * boundingRadius;

	bNormalInited = true;
	memcpy(m_boundingCenter, boundingCenter, 3 * sizeof(float));
	m_fboundingRadius = boundingRadius;

	// Show sample vertex geode
	if (bShowSampleVertex)
	{
		m_pScene->UpdateSamplePointGeode(m_sampleVertexIndexVec);
	}

	return true;
}

void ToothSegmentationModule::_getBoundaryPath(float fCurvatureValue)
{
	// clear
	m_pScene->UpdateSamplePointGeode(std::vector<unsigned>());

	std::set<unsigned> meshAreaVertexSet;
	std::set<unsigned> boundaryVertexSet;
	_splitVertexIntoTwoGroups(boundaryVertexSet, meshAreaVertexSet);

	std::set<unsigned> largestAreaGroup;
	std::set<unsigned> largestPathGroup;

	_getHeadAreaVertexGroupByConnection(meshAreaVertexSet, largestAreaGroup);
	_getMaxSizeVertexGroupByConnection(boundaryVertexSet, largestPathGroup);

	auto& maxSizeGroup = largestAreaGroup;
	std::set<unsigned> boundarySet;
	for (auto& vertexIndex : maxSizeGroup)
	{
		const auto& adjVertexSet = m_vertexAdjInfoVec[vertexIndex];
		for (auto adjVertex : adjVertexSet)
		{
			if (m_currentMeanCurvatureVec[adjVertex].second < fCurvatureValue)
			{
				boundarySet.insert(adjVertex);
			}
		}
	}

	std::set<unsigned> featureVertexSet;
	auto _isFeatureVertex = [this](unsigned vertexIndex, const std::set<unsigned>& pathVertexSet)
	{
		const auto& adjVertexSet = m_vertexAdjInfoVec[vertexIndex];

		unsigned uCount = 0;
		for (auto vertexIndex : adjVertexSet)
		{
			if (pathVertexSet.count(vertexIndex) > 0)
			{
				uCount++;
			}
		}

		return (uCount > 2);
	};

	for (auto vertexIndex : boundarySet)
	{
		if (_isFeatureVertex(vertexIndex, boundaryVertexSet))
		{
			featureVertexSet.insert(vertexIndex);
		}
	}

	// filter adjVertex feature
	{
		std::set<unsigned> deleteVertexSet;
		for (auto featureVertexIndex : featureVertexSet)
		{
			if (deleteVertexSet.count(featureVertexIndex) > 0)
			{
				continue;
			}

			const auto& adjVertexSet = m_vertexAdjInfoVec[featureVertexIndex];
			for (auto adjVertexIndex : adjVertexSet)
			{
				if (featureVertexSet.count(adjVertexIndex) > 0)
				{
					deleteVertexSet.insert(adjVertexIndex);
				}
			}
		}

		for (auto deleteVertexIndex : deleteVertexSet)
		{
			featureVertexSet.erase(deleteVertexIndex);
		}
	}

	auto _distanceOfTwoVertex = [this](const std::set<unsigned>& pathVertexSet, unsigned vertex0, unsigned vertex1, std::list<unsigned>& path)
	{
		std::set<unsigned> disableVertexSet;
		std::set<unsigned> currentLoop;
		unsigned pathLength = 0;

		currentLoop.insert(vertex0);

		std::vector<std::set<unsigned>> pathStackVec;

		bool bNotFinded = true;
		while (bNotFinded)
		{
			pathLength++;

			// Arrive?
			for (auto vertexIndex : currentLoop)
			{
				if (vertexIndex == vertex1)
				{
					bNotFinded = false;
					break;
				}
			}
			if (!bNotFinded)
			{
				break;
			}
			disableVertexSet.insert(currentLoop.begin(), currentLoop.end());

			std::set<unsigned> nextLoopVertexSet;
			for (auto vertexIndex : currentLoop)
			{
				auto& adjVertexSet = m_vertexAdjInfoVec[vertexIndex];
				for (auto adjVertexIndex : adjVertexSet)
				{
					if (pathVertexSet.count(adjVertexIndex) == 0)
					{
						continue;
					}

					if (disableVertexSet.count(adjVertexIndex) > 0)
					{
						continue;
					}

					nextLoopVertexSet.insert(adjVertexIndex);
				}
			}
			pathStackVec.push_back(currentLoop);
			currentLoop = nextLoopVertexSet;

			if (currentLoop.size() == 0)
			{
				pathLength = UINT_MAX;
				break;
			}
		}

		// Get path
		std::list<unsigned> _path;
		if (pathLength != UINT_MAX)
		{
			_path.push_back(vertex1);
			unsigned loopIndex = pathStackVec.size() - 1;
			while (true)
			{
				auto& currentLoopSet = pathStackVec[loopIndex];
				auto vertexIndex = _path.back();
				const auto& adjVertexSet = m_vertexAdjInfoVec[vertexIndex];

				bool bFinded = false;
				for (auto vertexIndex : adjVertexSet)
				{
					if (currentLoopSet.count(vertexIndex) > 0)
					{
						_path.push_back(vertexIndex);
						bFinded = true;
						break;
					}
				}
				if (!bFinded)
				{
					throw "FATAL ERROR";
				}
				loopIndex--;
				if (loopIndex == -1)
				{
					break;
				}
			}

			_path.reverse();
		}
		path = _path;
		return pathLength;
	};

	std::set<std::pair<unsigned, unsigned>> vertexGroup;
	std::vector<std::vector<unsigned>> generatePathVec;

	// Find vertexPair
	{
		for (auto featureVertexIndex : featureVertexSet)
		{
			std::vector<std::pair<unsigned, float>> canConnectVertexVec;
			for (auto _vertexIndex : featureVertexSet)
			{
				if (_vertexIndex == featureVertexIndex)
				{
					continue;
				}

				float distance = _distanceOfPointToPoint(m_mesh, featureVertexIndex, _vertexIndex);
				if (distance > m_fNearFeatureDistance)
				{
					continue;
				}

				std::list<unsigned> path0;
				std::list<unsigned> path1;

				auto length0 = 0.0f;
				auto length1 = 0.0f;

				auto pathVertexCount0 = _distanceOfTwoVertex(boundarySet, featureVertexIndex, _vertexIndex, path0);
				auto pathVertexCount1 = _distanceOfTwoVertex(largestPathGroup, featureVertexIndex, _vertexIndex, path1);

				if (pathVertexCount0 == UINT_MAX)
				{
					// Unreachable vertex pair
					continue;
				}

				// Filter too-near path
				/*if (pathVertexCount0 < boundarySet.size() * m_fNearFeatureGroupRatio)
				{
					continue;
				}*/

				// Compare total absDistance
				for (auto it = path0.begin(); it != path0.end();)
				{
					unsigned beginIndex = *(it);
					unsigned endIndex = *(++it);
					if (it == path0.end())
					{
						break;
					}

					length0 += _distanceOfPointToPoint(m_mesh, beginIndex, endIndex);
				}

				for (auto it = path1.begin(); it != path1.end();)
				{
					unsigned beginIndex = *(it);
					unsigned endIndex = *(++it);
					if (it == path1.end())
					{
						break;
					}

					length1 += _distanceOfPointToPoint(m_mesh, beginIndex, endIndex);
				}

				if (length0 > (m_fConnectFeatureDistanceRatio * length1))
				{
					canConnectVertexVec.push_back(std::make_pair(_vertexIndex, distance));
				}
			}

			if (canConnectVertexVec.size() == 0)
			{
				continue;
			}

			std::sort(canConnectVertexVec.begin(), canConnectVertexVec.end(), [](const std::pair<unsigned, float>& lhs, const std::pair<unsigned, float>& rhs)
			{
				return lhs.second < rhs.second;
			});

			// Correct vertex pair
			std::pair<unsigned, unsigned> correctPair = std::make_pair(featureVertexIndex, canConnectVertexVec[0].first);
			if (correctPair.first > correctPair.second)
			{
				auto temp = correctPair.first;
				correctPair.first = correctPair.second;
				correctPair.second = temp;
			}

			vertexGroup.insert(correctPair);
			
			//// For test
// 			for (const auto& vertexPair : canConnectVertexVec)
// 			{
// 				std::pair<unsigned, unsigned> correctPair = std::make_pair(featureVertexIndex, vertexPair.first);
// 				vertexGroup.insert(correctPair);
// 				std::list<unsigned> _path;
// 				_distanceOfTwoVertex(largestPathGroup, correctPair.first, correctPair.second, _path);
// 				generatePathVec.push_back(std::vector<unsigned>(_path.begin(), _path.end()));
// 			}
		}
	}
	std::vector<std::set<unsigned>> tempVecGroup;
	for (auto& connectVertexPair : vertexGroup)
	{
		bool bFinded = false;
		std::set<unsigned>* pFirstMatchSet = nullptr;
		for (auto& connectVertexSet : tempVecGroup)
		{
			if (connectVertexSet.count(connectVertexPair.first) > 0 ||
				connectVertexSet.count(connectVertexPair.second) > 0)
			{
				if (!bFinded)
				{
					connectVertexSet.insert(connectVertexPair.first);
					connectVertexSet.insert(connectVertexPair.second);
					pFirstMatchSet = &connectVertexSet;
					bFinded = true;
				}
				else
				{
					pFirstMatchSet->insert(connectVertexSet.begin(), connectVertexSet.end());
					connectVertexSet.clear();
				}
			}
		}

		if (!bFinded)
		{
			std::set<unsigned> newSet;
			newSet.insert(connectVertexPair.first);
			newSet.insert(connectVertexPair.second);
			tempVecGroup.push_back(newSet);
		}
	}

	std::set<std::pair<unsigned, unsigned>> remainPair;
	for (const auto& connectVertexSet : tempVecGroup)
	{
		// Find min distance vertex pair
		std::vector<std::pair<unsigned, unsigned>> tempPair;
		for (auto& pair : vertexGroup)
		{
			if (connectVertexSet.count(pair.first) > 0 &&
				connectVertexSet.count(pair.second) > 0)
			{
				tempPair.push_back(pair);
			}
		}

		unsigned uMinDistance = UINT_MAX;
		std::pair<unsigned, unsigned> minPathVertexPair;
		std::list<unsigned> minDistancePath;
		for (auto& pair : tempPair)
		{
			std::list<unsigned> _path;
			auto _uMinDistance = _distanceOfTwoVertex(largestPathGroup, pair.first, pair.second, _path);
		
			if (_uMinDistance < uMinDistance)
			{
				uMinDistance = _uMinDistance;
				minDistancePath = _path;
				minPathVertexPair = pair;
			}
		}

		generatePathVec.push_back(std::vector<unsigned>(minDistancePath.begin(), minDistancePath.end()));
		remainPair.insert(minPathVertexPair);
	}

	vertexGroup = remainPair;

// 	for (auto& connectPair : vertexGroup)
// 	{
// 		std::list<unsigned> _path;
// 		_distanceOfTwoVertex(largestPathGroup, connectPair.first, connectPair.second, _path);
// 		generatePathVec.push_back(std::vector<unsigned>(_path.begin(), _path.end()));
// 	}

	auto _getAllSubCircles = [this](const std::set<unsigned>& boundarySet, auto& _isFeatureVertex)
	{
		std::set<unsigned> NonBoundaryVertexIndexSet;

		float fMaxHeight = 0.0f;
		int nMaxHeightIndex = -1;

		std::set<unsigned> largestBoundarySet;
		_getMaxSizeVertexGroupByConnection(boundarySet, largestBoundarySet);

		for (unsigned i = 0; i < m_mesh.vertex_number; i++)
		{
			if (m_vertexToPlaneDistance[i] > fMaxHeight)
			{
				fMaxHeight = m_vertexToPlaneDistance[i];
				nMaxHeightIndex = i;
			}

			if (boundarySet.count(i) > 0)
			{
				continue;
			}

			NonBoundaryVertexIndexSet.insert(i);
		}

		if (nMaxHeightIndex == -1)
		{
			throw "FATAL ERROR";
		}

		std::vector<std::pair<std::set<unsigned>, std::set<unsigned>>> boundaryGroups;
		std::vector<std::set<unsigned>> groups;
		_groupVertexSetByConnection(NonBoundaryVertexIndexSet, groups);
		for (auto& group : groups)
		{
			if (group.count(nMaxHeightIndex) > 0)
			{
				continue;
			}

			std::set<unsigned> boundaryLoopVertexSet;

			for (auto vertexIndex : group)
			{
				const auto& adjVertexSet = m_vertexAdjInfoVec[vertexIndex];
				for (auto adjVertexIndex : adjVertexSet)
				{
					if (largestBoundarySet.count(adjVertexIndex) > 0)
					{
						boundaryLoopVertexSet.insert(vertexIndex);
					}
				}
			}

			boundaryGroups.push_back(std::make_pair(boundaryLoopVertexSet, group));

// 			std::set<unsigned> largestBoundaryVertexSet;
// 			_getMaxSizeVertexGroupByConnection(boundaryLoopVertexSet, largestBoundaryVertexSet);
// 			boundaryGroups.push_back(std::make_pair(largestBoundaryVertexSet, group));
		}

		return boundaryGroups;
	};

	featureVertexSet.clear();
	for (auto& group : vertexGroup)
	{
		featureVertexSet.insert(group.first);
		featureVertexSet.insert(group.second);
	}

	std::set<unsigned> generatePathVertexSet;
	for (auto& path : generatePathVec)
	{
		for (auto vertexIndex : path)
		{
			generatePathVertexSet.insert(vertexIndex);
		}
	}

	// Reset curvature
	for (auto& curvaturePair : m_currentMeanCurvatureVec)
	{
		curvaturePair.second = 0.0f;
	}

	std::set<unsigned> featurePath;

	featurePath.insert(boundarySet.begin(), boundarySet.end());
	featurePath.insert(generatePathVertexSet.begin(), generatePathVertexSet.end());

	std::set<unsigned> allBoundaryVertexSet;
	allBoundaryVertexSet.insert(featurePath.begin(), featurePath.end());
	allBoundaryVertexSet.insert(boundarySet.begin(), boundarySet.end());

	//m_pScene->UpdateSamplePointGeode(allBoundaryVertexSet, osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f), false);
	auto loopGroups = _getAllSubCircles(allBoundaryVertexSet, _isFeatureVertex);

	for (auto& group : loopGroups)
	{
		//m_pScene->UpdateSamplePointGeode(group.first, osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f), false);
	}

	// Check loops
	std::set<unsigned> extraBoundaryVertexSet;
	std::set<unsigned> HeadVertexSet;
	std::vector<std::vector<float>> errorRayPosition;

	for (auto& loop : loopGroups)
	{
		if (_isOneConnectedRegion(loop.first))
		{
			continue;
		}

		// Error loop
		std::cout << "Error Loop exist." <<  std::endl;

		// Init scanLoop
		std::set<unsigned> scanLoopVertex;
		for (auto _vertexIndex : loop.first)
		{
			const auto& adjVertexSet = m_vertexAdjInfoVec[_vertexIndex];
			for (auto adjVertexIndex : adjVertexSet)
			{
				if (loop.second.count(adjVertexIndex) > 0)
				{
					scanLoopVertex.insert(adjVertexIndex);
				}
			}
		}

		// Init area triangle set
		std::set<unsigned> areaTriangleSet;
		for (unsigned i = 0; i < m_mesh.triangle_number; i++)
		{
			unsigned* vertexIndex = m_mesh.triangle_index + 3 * i;
			bool bNeedAdd = false;
			for (unsigned j = 0; j < 3; j++)
			{
				if (loop.second.count(vertexIndex[j]) > 0)
				{
					bNeedAdd = true;
				}
			}

			if (bNeedAdd)
			{
				areaTriangleSet.insert(i);
			}
		}

		std::vector<std::set<unsigned>> _loops;
		_groupVertexSetByConnection(scanLoopVertex, _loops);
		// add a coarse head vertex to every loop
		for (auto& singleLoop : _loops)
		{
			float averageVertexCoord[3] = { 0.0f, 0.0f, 0.0f };

			float tempAABB[2][3]; // minCoord & maxCoord
			bool bInitTempAABB = false;

			for (auto _vertexIndex : singleLoop)
			{
				float* vertexCoord = m_mesh.vertex_position + 3 * _vertexIndex;

				for (unsigned j = 0; j < 3; j++)
				{
					if (vertexCoord[j] < tempAABB[0][j])
					{
						tempAABB[0][j] = vertexCoord[j];
					}

					if (vertexCoord[j] > tempAABB[1][j])
					{
						tempAABB[1][j] = vertexCoord[j];
					}
				}

				if (!bInitTempAABB)
				{
					tempAABB[0][0] = vertexCoord[0];
					tempAABB[0][1] = vertexCoord[1];
					tempAABB[0][2] = vertexCoord[2];
					tempAABB[1][0] = vertexCoord[0];
					tempAABB[1][1] = vertexCoord[1];
					tempAABB[1][2] = vertexCoord[2];
					
					bInitTempAABB = true;
				}
			}

			averageVertexCoord[0] = 0.5f * (tempAABB[0][0] + tempAABB[1][0]);
			averageVertexCoord[1] = 0.5f * (tempAABB[0][1] + tempAABB[1][1]);
			averageVertexCoord[2] = 0.5f * (tempAABB[0][2] + tempAABB[1][2]);

			std::cout << "averageVertexCoord : " << averageVertexCoord[0] << " " << averageVertexCoord[1] << " " << averageVertexCoord[2] << std::endl;

			// do ray cast
			float rayPositiveDirection[3] =
			{
				-m_clipPlaneNormal[0],
				-m_clipPlaneNormal[1],
				-m_clipPlaneNormal[2],
			};

			float rayNegativeDirection[3] =
			{
				m_clipPlaneNormal[0],
				m_clipPlaneNormal[1],
				m_clipPlaneNormal[2],
			};

			std::set<unsigned> intersectVertexSet;

			for (auto triangleIndex : areaTriangleSet)
			{
				unsigned* index = m_mesh.triangle_index + 3 * triangleIndex;
				float* vertexCoord[3] =
				{
					m_mesh.vertex_position + 3 * index[0],
					m_mesh.vertex_position + 3 * index[1],
					m_mesh.vertex_position + 3 * index[2],
				};

				float outUVW[3];
				float intersectCoord[3];
				bool bIntersect = ColliderAlgrithm::RayCollideTriangle(averageVertexCoord, rayPositiveDirection, vertexCoord[0], vertexCoord[1], vertexCoord[2], outUVW, intersectCoord);
				if (bIntersect)
				{
					intersectVertexSet.insert(index[0]);
					intersectVertexSet.insert(index[1]);
					intersectVertexSet.insert(index[2]);
				}

				bIntersect = ColliderAlgrithm::RayCollideTriangle(averageVertexCoord, rayNegativeDirection, vertexCoord[0], vertexCoord[1], vertexCoord[2], outUVW, intersectCoord);
				if (bIntersect)
				{
					intersectVertexSet.insert(index[0]);
					intersectVertexSet.insert(index[1]);
					intersectVertexSet.insert(index[2]);
				}
			}

			float fMinHeight = FLT_MAX;
			int nMinHeightVertexIndex = -1;

			// Select a minHeightVertex as headVertex
			for (auto vertexIndex : intersectVertexSet)
			{
				if (m_vertexToPlaneDistance[vertexIndex] < fMinHeight)
				{
					fMinHeight = m_vertexToPlaneDistance[vertexIndex];
					nMinHeightVertexIndex = vertexIndex;
				}
			}

			if (nMinHeightVertexIndex != -1)
			{
				singleLoop.insert(nMinHeightVertexIndex);
				HeadVertexSet.insert(nMinHeightVertexIndex);
				std::cout << "HeadVertex insert: " << nMinHeightVertexIndex << std::endl;
				continue;
			}

			std::vector<float> errorCoord;
			errorCoord.push_back(averageVertexCoord[0]);
			errorCoord.push_back(averageVertexCoord[1]);
			errorCoord.push_back(averageVertexCoord[2]);
			errorRayPosition.push_back(errorCoord);
			throw "FATAL ERROR";
		}

		std::set<unsigned> scanedVertexSet;
		while (scanedVertexSet.size() != loop.second.size())
		{
			for (unsigned i = 0; i < _loops.size(); i++)
			{
				// Add loops to scanedLoop
				auto& singleLoop = _loops[i];
				scanedVertexSet.insert(singleLoop.begin(), singleLoop.end());

				std::set<unsigned> nextSingleLoopSet;
				for (auto loopVertex : singleLoop)
				{
					const auto& adjVertexSet = m_vertexAdjInfoVec[loopVertex];

					for (auto adjVertexIndex : adjVertexSet)
					{
						bool bCollapse = false;
						for (unsigned j = 0; j < _loops.size(); j++)
						{
							auto& otherLoop = _loops[j];
							if (j == i)
							{
								continue;
							}

							if (otherLoop.count(adjVertexIndex) > 0)
							{
								extraBoundaryVertexSet.insert(loopVertex);
								extraBoundaryVertexSet.insert(adjVertexIndex);
								bCollapse = true;
								break;
							}
						}

						if (bCollapse)
						{
							continue;
						}

						if (scanedVertexSet.count(adjVertexIndex) == 0 && loop.second.count(adjVertexIndex) > 0)
						{
							nextSingleLoopSet.insert(adjVertexIndex);
						}
					}
				}

				singleLoop = nextSingleLoopSet;
			}
		}
	}

	for (auto index : featurePath)
	{
		m_currentMeanCurvatureVec[index].second = m_minMeanCurvature;
	}

	for (auto index : extraBoundaryVertexSet)
	{
		m_currentMeanCurvatureVec[index].second = m_minMeanCurvature;
	}

	// Export curvature data to dll.
	_exportCurvatureDataToDLL();
	m_boundaryVertexVec = std::vector<unsigned>(boundarySet.begin(), boundarySet.end());

	m_pScene->ShowCurvature();
	m_pScene->UpdateSamplePointGeode(std::vector<unsigned>(boundarySet.begin(), boundarySet.end()), osg::Vec4f(1.0f, 1.0f, 0.0f, 1.0f), false);
	m_pScene->UpdateSamplePointGeode(std::vector<unsigned>(generatePathVertexSet.begin(), generatePathVertexSet.end()), osg::Vec4f(0.0f, 0.0f, 1.0f, 1.0f), false);
	m_pScene->UpdateSamplePointGeode(std::vector<unsigned>(HeadVertexSet.begin(), HeadVertexSet.end()), osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f), false);
	m_pScene->UpdateSamplePointGeode(std::vector<unsigned>(extraBoundaryVertexSet.begin(), extraBoundaryVertexSet.end()), osg::Vec4f(0.0f, 0.0f, 0.0f, 1.0f), false);
	m_pScene->UpdateSamplePointGeode(std::vector<unsigned>(featureVertexSet.begin(), featureVertexSet.end()), osg::Vec4f(0.0f, 0.0f, 0.0f, 1.0f), false);
}

bool ToothSegmentationModule::_setupMeshInfo(Mesh& mesh)
{
	_setupMeshAdjInfo(mesh);

	float* curvatureArray = new float[mesh.vertex_number];
	if (!curvatureArray)
	{
		std::cout << "new operator failed." << std::endl;
		return false;
	}

	MeshCurvature meshCurvature(mesh);

	m_shortestPathProxy.getCurvatureData(reinterpret_cast<char*>(curvatureArray));

	m_currentMeanCurvatureVec.clear();
	m_currentMeanCurvatureVec.resize(mesh.vertex_number);

	m_minMeanCurvature = curvatureArray[0];
	m_maxMeanCurvature = curvatureArray[0];

	for (unsigned uVertexCount = 0; uVertexCount < mesh.vertex_number; uVertexCount++)
	{
		m_currentMeanCurvatureVec[uVertexCount].first = uVertexCount;
		m_currentMeanCurvatureVec[uVertexCount].second = curvatureArray[uVertexCount];

		if (curvatureArray[uVertexCount] < m_minMeanCurvature)
		{
			m_minMeanCurvature = curvatureArray[uVertexCount];
		}

		if (curvatureArray[uVertexCount] > m_maxMeanCurvature)
		{
			m_maxMeanCurvature = curvatureArray[uVertexCount];
		}
	}

	// MinCurvature
	if (!meshCurvature.CalCurvature(MinCurvature, curvatureArray))
	{
		std::cout << "CalCurvature failed." << std::endl;
		return false;
	}

	m_minCurvatureVec.clear();
	m_minCurvatureVec.resize(mesh.vertex_number);

	for (unsigned uVertexCount = 0; uVertexCount < mesh.vertex_number; uVertexCount++)
	{
		m_minCurvatureVec[uVertexCount].first = uVertexCount;
		m_minCurvatureVec[uVertexCount].second = curvatureArray[uVertexCount];
	}

	if (curvatureArray)
	{
		delete[] curvatureArray;
		curvatureArray = nullptr;
	}

	m_actualMeanCurvatureVec = m_currentMeanCurvatureVec;

	return true;
}

void ToothSegmentationModule::_sortMinCurvatureVec(bool bSortByIndex)
{
	static int bCurrentSortType = -1;
	if (bCurrentSortType != -1 && bCurrentSortType == bSortByIndex)
	{
		return;
	}

	if (bSortByIndex)
	{
		std::sort(m_minCurvatureVec.begin(), m_minCurvatureVec.end(), [](const std::pair< unsigned, float>& lhs, const std::pair< unsigned, float>& rhs)
		{
			return lhs.first < rhs.first;
		});
	}
	else
	{
		// Min curvature
		std::sort(m_minCurvatureVec.begin(), m_minCurvatureVec.end(), [](const std::pair< unsigned, float>& lhs, const std::pair< unsigned, float>& rhs)
		{
			return lhs.second < rhs.second;
		});
	}

	bCurrentSortType = bSortByIndex;
}

void ToothSegmentationModule::_sortMeanCurvatureVec(bool bSortByIndex)
{
	static int bCurrentSortType = -1;
	if (bCurrentSortType != -1 && bCurrentSortType == bSortByIndex)
	{
		return;
	}

	if (bSortByIndex)
	{
		std::sort(m_currentMeanCurvatureVec.begin(), m_currentMeanCurvatureVec.end(), [](const std::pair< unsigned, float>& lhs, const std::pair< unsigned, float>& rhs)
		{
			return lhs.first < rhs.first;
		});
	}
	else
	{
		// Min curvature
		std::sort(m_currentMeanCurvatureVec.begin(), m_currentMeanCurvatureVec.end(), [](const std::pair< unsigned, float>& lhs, const std::pair< unsigned, float>& rhs)
		{
			return lhs.second < rhs.second;
		});
	}

	bCurrentSortType = bSortByIndex;
}

void ToothSegmentationModule::_splitVertexIntoTwoGroups(std::set<unsigned>& boundarySet, std::set<unsigned>& meshAreaSet)
{
	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		if (m_currentMeanCurvatureVec[i].second < m_fRegionGrowCurvatureRatio * m_minMeanCurvature)
		{
			boundarySet.insert(i);
		}
		else
		{
			meshAreaSet.insert(i);
		}
	}
}

void ToothSegmentationModule::_fillSmallMeshArea(bool bShow)
{
	std::set<unsigned> boundarySet;
	std::set<unsigned> meshAreaSet;
	_splitVertexIntoTwoGroups(boundarySet, meshAreaSet);

	std::vector<std::set<unsigned>> meshAreaGroups;
	_groupVertexSetByConnection(meshAreaSet, meshAreaGroups);

	for (auto& group : meshAreaGroups)
	{
		if (group.size() < 10)
		{
			for (auto _vertexIndex : group)
			{
				m_currentMeanCurvatureVec[_vertexIndex].second = m_minMeanCurvature;
			}
		}
	}

	if (bShow)
	{
		m_pScene->ShowCurvature();
	}
}

void ToothSegmentationModule::_addToothPartLowCurvatureInfo(bool bShow)
{
	std::set<unsigned> boundarySet;
	std::set<unsigned> meshAreaSet;
	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		if (m_currentMeanCurvatureVec[i].second < m_fRegionGrowCurvatureRatio * m_minMeanCurvature)
		{
			boundarySet.insert(i);
		}
		else
		{
			meshAreaSet.insert(i);
		}
	}

	std::set<unsigned> largestAreaGroup;
	_getHeadAreaVertexGroupByConnection(meshAreaSet, largestAreaGroup);

	for (auto vertexIndex : meshAreaSet)
	{
		if (largestAreaGroup.count(vertexIndex) > 0)
		{
			continue;
		}

		if (m_actualMeanCurvatureVec[vertexIndex].second < m_minMeanCurvature * m_fRegionGrowCurvatureRatio)
		{
			m_currentMeanCurvatureVec[vertexIndex].second = m_minMeanCurvature;
		}
	}

	if (bShow)
	{
		m_pScene->ShowCurvature();
	}
}

void ToothSegmentationModule::_exportEnergyVertexInfo(const std::vector<unsigned>& planeVertexVec, const std::vector<std::vector<unsigned>>& zeroVertexVec, const std::vector<std::vector<unsigned>>& oneVertexVec)
{
	auto currentLoopSet = _generateBoundaryVertexSet(planeVertexVec);

	static auto _writeUnsignedToFile = [](std::ofstream& outFile, unsigned value)
	{
		outFile.write(reinterpret_cast<char*>(&value), sizeof(value));
	};

	std::ofstream outFile("info.bin", std::ios::out | std::ios::binary);
	_writeUnsignedToFile(outFile, currentLoopSet.size());
	for (auto vertexIndex : currentLoopSet)
	{
		_writeUnsignedToFile(outFile, vertexIndex);
	}

	std::set<unsigned> zeroVertexSet;
	for (const auto& zeroGroup : zeroVertexVec)
	{
		for (auto vertexIndex : zeroGroup)
		{
			zeroVertexSet.insert(vertexIndex);
		}
	}

	std::set<unsigned> oneVertexSet;
	for (const auto& oneGroup : oneVertexVec)
	{
		for (auto vertexIndex : oneGroup)
		{
			oneVertexSet.insert(vertexIndex);
		}
	}

	_writeUnsignedToFile(outFile, zeroVertexSet.size());
	for (auto vertexIndex : zeroVertexSet)
	{
		_writeUnsignedToFile(outFile, vertexIndex);
	}

	_writeUnsignedToFile(outFile, oneVertexSet.size());
	for (auto vertexIndex : oneVertexSet)
	{
		_writeUnsignedToFile(outFile, vertexIndex);
	}

	outFile.close();
}

void ToothSegmentationModule::_exportAllGroupsInfo(const std::vector<std::vector<unsigned>>& groups, const std::set<unsigned>& bounadarySet)
{
	if (m_boundaryVertexVec.size() == 0)
	{
		throw "FATAL ERROR";
	}

	auto currentLoopSet = _generateBoundaryVertexSet(m_boundaryVertexVec);
	if (currentLoopSet.size() == 0)
	{
		throw "FATAL ERROR";
	}

	std::ofstream outFile("groupsInfo.bin", std::ios::binary | std::ios::out);
	int splitInt = -1;
	
	// Export loopSet
	for (auto vertexIndex : currentLoopSet)
	{
		outFile.write(reinterpret_cast<char*>(&vertexIndex), sizeof(unsigned));
	}

	outFile.write(reinterpret_cast<char*>(&splitInt), sizeof(int));

	// Export vertex groups
	for (auto group : groups)
	{
		for (auto vertexIndex : group)
		{
			outFile.write(reinterpret_cast<char*>(&vertexIndex), sizeof(unsigned));
		}

		outFile.write(reinterpret_cast<char*>(&splitInt), sizeof(int));
	}

	outFile.close();
}

void ToothSegmentationModule::_exportPlaneInfoAndFeatureInfo()
{
	std::ofstream outPutInfo("info.txt");
	if (!outPutInfo.is_open())
	{
		throw "open file failed.";
	}

	outPutInfo << "Plane normal:" << m_clipPlaneNormal[0] << " " << m_clipPlaneNormal[1] << " " << m_clipPlaneNormal[2] << "\n";
	outPutInfo << "Plane center:" << m_clipPlaneCenter[0] << " " << m_clipPlaneCenter[1] << " " << m_clipPlaneCenter[2] << "\n";

	outPutInfo << "Feature Groups:\n";
	outPutInfo.close();
}

void ToothSegmentationModule::_setupMeshAdjInfo(Mesh &mesh)
{
	m_vertexAdjInfoVec.resize(mesh.vertex_number);
	for (unsigned i = 0; i < mesh.triangle_number; i++)
	{
		unsigned* pTriangleIndex = mesh.triangle_index + 3 * i;

		for (unsigned j = 0; j < 3; j++)
		{
			unsigned indexs[2];
			indexs[0] = pTriangleIndex[(j + 1) % 3];
			indexs[1] = pTriangleIndex[(j + 2) % 3];

			m_vertexAdjInfoVec[pTriangleIndex[j]].insert(indexs[0]);
			m_vertexAdjInfoVec[pTriangleIndex[j]].insert(indexs[1]);
		}
	};

	_setupEdgeInfo(mesh);
}

void ToothSegmentationModule::_setupEdgeInfo(Mesh &mesh)
{
	for (unsigned i = 0; i < mesh.triangle_number; i++)
	{
		unsigned* pTriangleIndex = mesh.triangle_index + 3 * i;
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

float ToothSegmentationModule::CalPlaneEnergy(const Mesh& mesh, float* clipPlaneNormal, float* clipPlaneCenter, unsigned& outLoopCount, bool OneLoopCheckOpen, bool bShow)
{
	_sortMinCurvatureVec(true);

	static float fAvgMinCurvature = 0.0f;
	static bool bAvgMinCurvatureInited = false;

	if (!bAvgMinCurvatureInited)
	{
		if (m_minCurvatureVec.size() == 0)
		{
			throw "FATAL ERROR";
		}

		fAvgMinCurvature = 0.0f;
		for (auto minCurvature : m_minCurvatureVec)
		{
			fAvgMinCurvature += minCurvature.second;
		}

		fAvgMinCurvature /= m_minCurvatureVec.size();

		bAvgMinCurvatureInited = true;
	}

	if (m_triangleToPlaneDistances.size() == 0)
	{
		// Init static vector
		m_triangleToPlaneDistances.resize(mesh.triangle_number);
		m_vertexToPlaneDistance.resize(mesh.vertex_number);

		for (unsigned i = 0; i < mesh.triangle_number; i++)
		{
			unsigned* vertexIndices = mesh.triangle_index + 3 * i;

			double distances[3];
			distances[0] = _distanceOfPointToPlane(mesh, vertexIndices[0], clipPlaneNormal, m_baseCenter);
			distances[1] = _distanceOfPointToPlane(mesh, vertexIndices[1], clipPlaneNormal, m_baseCenter);
			distances[2] = _distanceOfPointToPlane(mesh, vertexIndices[2], clipPlaneNormal, m_baseCenter);

			m_triangleToPlaneDistances[i].first = min(min(distances[0], distances[1]), distances[2]);
			m_triangleToPlaneDistances[i].second = max(max(distances[0], distances[1]), distances[2]);

			m_vertexToPlaneDistance[vertexIndices[0]] = distances[0];
			m_vertexToPlaneDistance[vertexIndices[1]] = distances[1];
			m_vertexToPlaneDistance[vertexIndices[2]] = distances[2];
		}
	}

	static auto _getIntersectTriangleIndices = [this](std::vector<unsigned>& outIntersectTriangleVec)
	{
		for (unsigned i = 0; i < m_triangleToPlaneDistances.size(); i++)
		{
			auto& distanceInfo = m_triangleToPlaneDistances[i];
			if (distanceInfo.first < m_clipPlaneOffset && distanceInfo.second > m_clipPlaneOffset)
			{
				outIntersectTriangleVec.push_back(i);
			}
		}
	};

	static auto _getTriangleIndicesOnLoop = [this](std::vector<unsigned>& outIntersectTriangleVec)
	{
		for (unsigned i = 0; i < m_triangleToPlaneDistances.size(); i++)
		{
			auto& distanceInfo = m_triangleToPlaneDistances[i];
			if (distanceInfo.second < m_clipPlaneOffset)
			{
				outIntersectTriangleVec.push_back(i);
			}
		}
	};

	static auto _getTriangleCurvature = [this](unsigned uTriangleIndex, float* planeNormal, float* planeCenter, float* clipPlaneNormal, float* clipPlaneCenter)
	{
		unsigned* vertexIndices = m_mesh.triangle_index + 3 * uTriangleIndex;

		auto avgCurvature =
			m_minCurvatureVec[vertexIndices[0]].second +
			m_minCurvatureVec[vertexIndices[1]].second +
			m_minCurvatureVec[vertexIndices[2]].second;

		return avgCurvature / 3.0f;
	};

	std::vector<unsigned> intersectTriangleIndexVec;
	_getIntersectTriangleIndices(intersectTriangleIndexVec);

	std::vector<unsigned> triangleIndexOnLoop;
	_getTriangleIndicesOnLoop(triangleIndexOnLoop);

	if (intersectTriangleIndexVec.size() < 2)
	{
		return FLT_MAX;
	}

	static auto _getLoopCount = [this](const Mesh& mesh, std::vector<unsigned>& triangleIndexVec, bool bShowIntersectVertex)
	{
		unsigned nLoopCount = 0;

		if (bShowIntersectVertex)
		{
			m_pScene->UpdateSampleTriangleGeode(triangleIndexVec);
		}

		std::set<unsigned> triangleSet(triangleIndexVec.begin(), triangleIndexVec.end());
		while (triangleSet.size() > 0)
		{
			nLoopCount++;
			std::set<unsigned> stack;
			stack.insert(*triangleSet.begin());

			while (stack.size() > 0)
			{
				auto firstElement = *stack.begin();
				triangleSet.erase(firstElement);
				stack.erase(stack.begin());

				unsigned* vertexIndices = mesh.triangle_index + 3 * firstElement;

				// Add AdjTriangle to stack
				for (unsigned j = 0; j < 3; j++)
				{
					std::pair<unsigned, unsigned> edge = std::make_pair(vertexIndices[j], vertexIndices[(j + 1) % 3]);
					if (edge.first > edge.second)
					{
						auto temp = edge.first;
						edge.first = edge.second;
						edge.second = temp;
					}

					auto& edgeAdjTriangles = m_edge[edge];
					for (auto triangleIndex : edgeAdjTriangles)
					{
						if (triangleIndex != firstElement && triangleSet.count(triangleIndex) > 0)
						{
							stack.insert(triangleIndex);
						}
					}
				}
			}
		}

		return nLoopCount;
	};

	outLoopCount = _getLoopCount(mesh, intersectTriangleIndexVec, bShow);

	if (OneLoopCheckOpen && outLoopCount != 1)
	{
		return FLT_MAX;
	}

	float square_deviation = 0.0f;
	for (auto index : triangleIndexOnLoop)
	{
		float temp = _getTriangleCurvature(index, clipPlaneNormal, clipPlaneCenter, clipPlaneNormal, clipPlaneCenter) - fAvgMinCurvature;
		temp = temp * temp;
		square_deviation += temp;
	}

	square_deviation /= (triangleIndexOnLoop.size() - 1);

	return square_deviation;
}

void ToothSegmentationModule::_constructAlgPipeLine()
{
	m_algorithmPipeLine.DiscardAllTask();

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		m_pScene->UpdateSamplePointGeode(std::vector<unsigned>());
		_curvatureThreshold(m_minMeanCurvature * m_fRegionGrowCurvatureRatio);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_regionGrow(m_fPreRegionGrowRatio * m_fMeshDensity + m_fPreRegionGrowLoopCount, true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_filterSmallLowCurvatureRegion(m_minMeanCurvature * m_fRegionGrowCurvatureRatio);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_reLocatePlane(true);
	}));
	
	m_algorithmPipeLine.AddTask(Task([&]() 
	{
		unsigned uLoopCount = static_cast<unsigned>((m_fMeshDensity * m_fGrowLoopRatio));
		_regionGrow(uLoopCount, true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		m_featureVertexVec = _findFeatureVertexIndex(false, true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_reFindBoundary(true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_regionGrowUtilOneRegion(false);
		_fillSmallMeshArea(true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_getBoundarySkeleton(m_minMeanCurvature * m_fRegionGrowCurvatureRatio, true, false);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		m_featureVertexVec = _findFeatureVertexIndex(false, true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_reFindBoundary(true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_regionGrowUtilOneRegion(false);
		_fillSmallMeshArea(true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_getBoundarySkeleton(m_minMeanCurvature * m_fRegionGrowCurvatureRatio, true, false);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		m_featureVertexVec = _findFeatureVertexIndex(false, true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_reFindBoundary(true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_regionGrowUtilOneRegion(false);
		_fillSmallMeshArea(true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_getBoundarySkeleton(m_minMeanCurvature * m_fRegionGrowCurvatureRatio, true, false);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_addToothPartLowCurvatureInfo(true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		unsigned uLoopCount = static_cast<unsigned>((m_fMeshDensity * m_fGrowLoopRatio));
		_regionGrow(uLoopCount, true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_fillSmallMeshArea(true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_getBoundarySkeleton(m_minMeanCurvature * m_fRegionGrowCurvatureRatio, true, false);
	}));
	
// 	m_algorithmPipeLine.AddTask(Task([&]()
// 	{
// 		_leftLargestLowCurvatureRegion(true);
// 	}));
	
	m_algorithmPipeLine.AddTask(Task([&]()
	{
		_getBoundaryPath(m_minMeanCurvature * m_fRegionGrowCurvatureRatio);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		m_featureVertexVec = _findFeatureVertexIndex(false, true);
	}));

	m_algorithmPipeLine.AddTask(Task([&]()
	{
		m_originalFeatureGroups = _getFeatureGroups(m_featureVertexVec, m_fGroupFeatureMeshDistanceRatio, m_fGroupAbsDistance);
		m_zeroGroups = m_originalFeatureGroups;
		m_pScene->UpdateGroupVertexResult(m_originalFeatureGroups, m_zeroGroups, m_oneGroups);
		m_zeroGroups.clear();
	}));

	/*m_algorithmPipeLine.AddTask(Task([&]()
	{
		auto groups = _groupFeatureVertexIntoTwoGroup(m_originalFeatureGroups, m_zeroGroups, m_oneGroups);
		m_pScene->_updateGroupVertexResult(m_originalFeatureGroups, m_zeroGroups, m_oneGroups);
		_exportAllGroupsInfo(groups, m_boundaryVertexSet);
	}));*/
}

void ToothSegmentationModule::_regionGrow(unsigned uGrowLoopCount, bool bShow)
{
	_sortMeanCurvatureVec(true);

	std::vector<float> tempCurvatureMap(m_mesh.vertex_number, FLT_MAX);

	for (unsigned i = 0; i < m_currentMeanCurvatureVec.size(); i++)
	{
		tempCurvatureMap[i] = m_currentMeanCurvatureVec[i].second;
	}

	static auto _regionGrow = [this](std::vector<float>& tempCurvatureMap, unsigned uGrowLoopCount, float _fMinCurvatureValue)
	{
		std::set<unsigned> lastGrowRegionVertexSet;
		bool bInitLastGrowregionVertexSet = false;

		for (unsigned loop = 0; loop < uGrowLoopCount; loop++)
		{
			// Find growRegionSet
			if (!bInitLastGrowregionVertexSet)
			{
				for (unsigned i = 0; i < tempCurvatureMap.size(); i++)
				{
					if (tempCurvatureMap[i] < _fMinCurvatureValue)
					{
						lastGrowRegionVertexSet.insert(i);
					}
				}

				if (lastGrowRegionVertexSet.size() == 0)
				{
					throw "lastGrowRegionVertexSet size == 0";
				}

				bInitLastGrowregionVertexSet = true;
			}

			float fCurvatureMin = m_minMeanCurvature;
			std::set<unsigned> curvatureModifiedSet;

			for (auto _vertexIndex : lastGrowRegionVertexSet)
			{
				const auto& adjVertexVec = m_vertexAdjInfoVec[_vertexIndex];
						
				for (auto adjVertexIndex : adjVertexVec)
				{
					if (m_vertexToPlaneDistance[adjVertexIndex] > m_clipPlaneOffset)
					{
						continue;
					}

					if (tempCurvatureMap[adjVertexIndex] > fCurvatureMin)
					{
						curvatureModifiedSet.insert(adjVertexIndex);
					}
				}
			}

			for (auto _vertexIndex : curvatureModifiedSet)
			{
				lastGrowRegionVertexSet.insert(_vertexIndex);
				tempCurvatureMap[_vertexIndex] = fCurvatureMin;
			}
		}
	};

	_regionGrow(tempCurvatureMap, uGrowLoopCount, m_fRegionGrowCurvatureRatio * m_minMeanCurvature);

	for (unsigned i = 0; i < m_currentMeanCurvatureVec.size(); i++)
	{
		m_currentMeanCurvatureVec[i].second = tempCurvatureMap[i];
	}

	if (bShow)
	{
		m_pScene->ShowCurvature();
	}
}

void ToothSegmentationModule::_regionGrowUtilOneRegion(bool bShow)
{
	while (true)
	{
		std::set<unsigned> boundaryVertexSet;
		for (unsigned i = 0; i < m_mesh.vertex_number; i++)
		{
			if (m_currentMeanCurvatureVec[i].second < m_fRegionGrowCurvatureRatio * m_minMeanCurvature)
			{
				boundaryVertexSet.insert(i);
			}
		}

		if (_isOneConnectedRegion(boundaryVertexSet))
		{
			break;
		}

		_regionGrow(5, false);
	}

	// Additional regionGrow
	_regionGrow(5, false);

	if (bShow)
	{
		m_pScene->ShowCurvature();
	}
}

void ToothSegmentationModule::_reFindBoundary(bool bShow)
{
	std::set<unsigned> boundaryVertexSet;
	std::set<unsigned> meshAreaVertexSet;
	_splitVertexIntoTwoGroups(boundaryVertexSet, meshAreaVertexSet);

	std::set<unsigned> largestGroup;
	_getHeadAreaVertexGroupByConnection(meshAreaVertexSet, largestGroup);

	std::set<unsigned> exportGingivalVertexSet;
	for (auto vertexIndex : boundaryVertexSet)
	{
		const auto& adjVertexSet = m_vertexAdjInfoVec[vertexIndex];

		bool bIsGingival = false;
		for (auto adjVertexIndex : adjVertexSet)
		{
			if (largestGroup.count(adjVertexIndex) > 0)
			{
				bIsGingival = true;
				break;
			}
		}

		if (bIsGingival)
		{
			exportGingivalVertexSet.insert(vertexIndex);
		}
	}

	auto finalBoundaryVertexSet = _generateBoundaryVertexSet(std::vector<unsigned>(exportGingivalVertexSet.begin(), exportGingivalVertexSet.end()));
	_updateHarmonicField(std::vector<unsigned>(), std::vector<unsigned>(finalBoundaryVertexSet.begin(), finalBoundaryVertexSet.end()), std::vector<unsigned>(m_featureVertexVec.begin(), m_featureVertexVec.end()), bShow);

	if (bShow)
	{
		m_pScene->UpdateSamplePointGeode(std::vector<unsigned>(exportGingivalVertexSet.begin(), exportGingivalVertexSet.end()), osg::Vec4f(0.0f, 0.0f, 1.0f, 1.0f), false);
		m_pScene->UpdateSamplePointGeode(std::vector<unsigned>(finalBoundaryVertexSet.begin(), finalBoundaryVertexSet.end()), osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f), false);
	}
}

void ToothSegmentationModule::_updateHarmonicField(const std::vector<unsigned>& base, const std::vector<unsigned>& neg, const std::vector<unsigned>& pos, bool bShow)
{
	float* harmonic = new float[m_mesh.vertex_number];
	m_harmonicCalculator->Do(base, neg, pos, harmonic);

	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		m_currentMeanCurvatureVec[i].second = (0.45f < harmonic[i] && harmonic[i] < 0.50f) ? m_minMeanCurvature : 0.0f;
	}

	delete[] harmonic;
	
	if (bShow)
	{
		m_pScene->ShowCurvature();
	}
}

void ToothSegmentationModule::_updateAlgorithmParameters(std::string configFile)
{
	std::ifstream configFileStream(configFile.c_str(), std::ios::in);
	
	if (configFileStream.is_open())
	{
		std::vector<float> attributeList;

		char tempString[256];
		while ( configFileStream.getline(tempString, 256))
		{
			attributeList.push_back(std::atof(tempString));
			std::cout << "Read attribute parameter:" << tempString << std::endl;

			memset(tempString, 0, sizeof(tempString));
		}

		configFileStream.close();

		m_fPreRegionGrowRatio			 = attributeList[0];
		m_nRegionGrowFilterMinSize		 = attributeList[1];
		m_fFilterRegionRatio			 = attributeList[2];
		m_fInitEnergyMoveStepRatio		 = attributeList[3];
		m_fRegionGrowCurvatureRatio		 = attributeList[4];
		m_fNearFeatureDistance			 = attributeList[5];
		m_fConnectFeatureDistanceRatio	 = attributeList[6];
		m_fGrowLoopRatio				 = attributeList[7];
		m_fGroupFeatureMeshDistanceRatio = attributeList[8];
		m_fGroupAbsDistance				 = attributeList[9];
		m_fNearFeatureGroupRatio		 = attributeList[10];
	}
	else
	{
		std::cout << "[ERROR] OpenFile failed." << std::endl;

		m_fPreRegionGrowRatio = 4;
		m_nRegionGrowFilterMinSize = -1;
		m_fFilterRegionRatio = 0.03f;
		m_fInitEnergyMoveStepRatio = 500.0f;
		m_fRegionGrowCurvatureRatio = 0.50f;
		m_fNearFeatureDistance = 8.0f;
		m_fConnectFeatureDistanceRatio = 1.0f;
		m_fGrowLoopRatio = 1.5f;
		m_fGroupFeatureMeshDistanceRatio = 1.5f;
		m_fGroupAbsDistance = 12.0f;
	}
}

void ToothSegmentationModule::_resetAlgorithmState()
{
	_updateAlgorithmParameters("parameter.cfg");

	// clear algorithm middle data
	m_boundaryVertexVec.clear();
	m_sampleVertexIndexVec.clear();
	m_featureVertexVec.clear();
	m_originalFeatureGroups.clear();
	m_zeroGroups.clear();
	m_oneGroups.clear();
	
	m_currentMeanCurvatureVec = m_actualMeanCurvatureVec;
	m_pScene->ShowCurvature();
	_constructAlgPipeLine();
}

void ToothSegmentationModule::_reLocatePlane(bool bShow)
{
	_sortMeanCurvatureVec(true);

	float maxHeight = m_clipPlaneOffset;
	float newPlaneHeight = m_clipPlaneOffset;

	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		float fCurrentHeight = m_vertexToPlaneDistance[i];
		if (fCurrentHeight > maxHeight)
		{
			maxHeight = fCurrentHeight;
		}

		if (m_currentMeanCurvatureVec[i].second < -0.01f)
		{
			if (fCurrentHeight > newPlaneHeight)
			{
				newPlaneHeight = fCurrentHeight;
			}
		}
	}

	if (newPlaneHeight > m_clipPlaneOffset)
	{
		m_clipPlaneOffset = newPlaneHeight + 0.5f * (maxHeight - newPlaneHeight);
		UpdateClipPlaneCenter();
	}

	if (bShow)
	{
		m_pScene->SetPlaneGeode(false, false);
	}
}

void ToothSegmentationModule::_leftLargestLowCurvatureRegion(bool bShow)
{
	std::set<unsigned> lowCurvatureVertexSet;
	for (unsigned i = 0; i < m_currentMeanCurvatureVec.size(); i++)
	{
		if (m_currentMeanCurvatureVec[i].second < m_minMeanCurvature * m_fRegionGrowCurvatureRatio)
		{
			lowCurvatureVertexSet.insert(i);
		}
	}

	std::vector<std::set<unsigned>> regionSetVec;
	_groupVertexSetByConnection(lowCurvatureVertexSet, regionSetVec);

	if (regionSetVec.size() > 1)
	{
		for (const auto& region : regionSetVec)
		{
			if (region.size() < lowCurvatureVertexSet.size() * 0.5f)
			{
				for (auto _vertexIndex : region)
				{
					m_currentMeanCurvatureVec[_vertexIndex].second = 0.0f;
				}
			}
		}
	}

	if (bShow)
	{
		m_pScene->ShowCurvature();
	}
}

void ToothSegmentationModule::_initMeshDensity()
{
	// Init mesh density.
	float totalEdgeLength = 0.0f;

	for (unsigned i = 0; i < m_mesh.triangle_number; i++)
	{
		unsigned* vertexIndices = m_mesh.triangle_index + 3 * i;
		float edgeLength[3];

		edgeLength[0] = _distanceOfPointToPoint(m_mesh, vertexIndices[0], vertexIndices[1]);
		edgeLength[1] = _distanceOfPointToPoint(m_mesh, vertexIndices[1], vertexIndices[2]);
		edgeLength[2] = _distanceOfPointToPoint(m_mesh, vertexIndices[2], vertexIndices[0]);

		totalEdgeLength += edgeLength[0];
		totalEdgeLength += edgeLength[1];
		totalEdgeLength += edgeLength[2];
	}

	totalEdgeLength /= (m_mesh.triangle_number * 3.0);
	m_fMeshDensity = 1.0f / totalEdgeLength;

	std::cout << "Mesh Density: " << m_fMeshDensity << std::endl;
}

void ToothSegmentationModule::_exportCurvatureData()
{
	// Export to file
	std::fstream outfile("curvatureData.txt", std::ios::out);
	if (!outfile.is_open())
	{
		throw "FATAL ERROR.";
	}

	for (unsigned i = 0; i < m_currentMeanCurvatureVec.size(); i++)
	{
		outfile << m_currentMeanCurvatureVec[i].second << "\n";
	}
	outfile.close();
}

void ToothSegmentationModule::_getBoundarySkeleton(float fCurvatureValue, bool bShow, bool bOnlyDoOneStep)
{
	_sortMeanCurvatureVec(true);

	static auto _groupVertexs = [this](const std::set<unsigned>& needGroupVertexSet, const std::set<unsigned>& regionSet, const std::vector<int>& vertexToAreaIndexVec, std::set<unsigned>& centerVertexSet, std::set<unsigned>& complexVertexSet, std::map<unsigned, std::set<unsigned>>& diskVertexMap)
	{
		centerVertexSet.clear();
		complexVertexSet.clear();
		diskVertexMap.clear();

		for (auto vertexIndex : needGroupVertexSet)
		{
			const auto& adjVertexVec = m_vertexAdjInfoVec[vertexIndex];

			bool bIsCenterVertex = true;
			for (auto& adjVertex : adjVertexVec)
			{
				if (regionSet.count(adjVertex) == 0)
				//& if (vertexToAreaIndexVec[adjVertex] != -1)
				{
					bIsCenterVertex = false;
					break;
				}
			}

			if (bIsCenterVertex)
			{
				centerVertexSet.insert(vertexIndex);
			}
		}

		for (auto vertexIndex : needGroupVertexSet)
		{
			if (centerVertexSet.count(vertexIndex) > 0)
			{
				continue;
			}

			auto& adjVertexVec = m_vertexAdjInfoVec[vertexIndex];
			std::set<unsigned> connectedAreaIndexSet;
			std::set<unsigned> adjBoundarySet;

			for (auto adjVertexIndex : adjVertexVec)
			{
				if (regionSet.count(adjVertexIndex) == 0)
				//& if (vertexToAreaIndexVec[adjVertexIndex] != -1)
				{
					adjBoundarySet.insert(adjVertexIndex);
				}
				else
				{
					continue;
				}

				connectedAreaIndexSet.insert(vertexToAreaIndexVec[adjVertexIndex]);
			}

			if (connectedAreaIndexSet.size() > 1)
			{
				complexVertexSet.insert(vertexIndex);
				continue;
			}

			if (!_isOneConnectedRegion(adjBoundarySet))
			{
				complexVertexSet.insert(vertexIndex);
				continue;
			}

			unsigned uGroupIndex = *connectedAreaIndexSet.begin();
			diskVertexMap[uGroupIndex].insert(vertexIndex);
		}
	};

	// Fast vertexIndex to vertexType mapping
	// Type:
	//		complex = -2
	//		center	= -1
	//		disk	= GroupIndex ( > 0)
	static std::vector<int> vertexToDifferentTypeMap(m_mesh.vertex_number, INT_MAX);
	for (auto& typeValue : vertexToDifferentTypeMap)
	{
		typeValue = INT_MAX;
	}

	std::set<unsigned> minCurvatureVertexSet;
	std::set<unsigned> otherVertexSet;
	_splitVertexIntoTwoGroups(minCurvatureVertexSet, otherVertexSet);

	std::set<unsigned> largestBoundaryVertexSet;
	_getMaxSizeVertexGroupByConnection(minCurvatureVertexSet, largestBoundaryVertexSet);

	for (auto _vertexIndex : minCurvatureVertexSet)
	{
		if (largestBoundaryVertexSet.count(_vertexIndex) > 0)
		{
			continue;
		}

		m_currentMeanCurvatureVec[_vertexIndex].second = 0.0f;
		otherVertexSet.insert(_vertexIndex);
	}

	// Divide mesh into different groups
	std::vector<int> vertexToAreaIndexVec(m_mesh.vertex_number, -1);
	unsigned uMeshAreaGroupSize = 0;

	{
		std::vector<std::set<unsigned>> meshAreaGroups;
		_groupVertexSetByConnection(otherVertexSet, meshAreaGroups);
		uMeshAreaGroupSize = meshAreaGroups.size();
		std::cout << "meshAreaGroups.size = " << meshAreaGroups.size() << ", meshArea total vertex size:" << otherVertexSet.size() << std::endl;

		for (unsigned i = 0; i < meshAreaGroups.size(); i++)
		{
			auto& areaGroup = meshAreaGroups[i];
			for (auto _vertexIndex : areaGroup)
			{
				vertexToAreaIndexVec[_vertexIndex] = i;
			}
		}
	}

	std::set<unsigned> totalInternalComplexVertexSet;
	bool bBeginDeleteInternalComplexVertex = false;

	std::set<unsigned> centerVertexSet;
	std::set<unsigned> complexVertexSet;
	std::map<unsigned, std::set<unsigned>> diskVertexMap;
	_groupVertexs(largestBoundaryVertexSet, largestBoundaryVertexSet, vertexToAreaIndexVec, centerVertexSet, complexVertexSet, diskVertexMap);

	for (auto _vertexIndex : centerVertexSet)
	{
		vertexToDifferentTypeMap[_vertexIndex] = -1;
	}

	for (auto _vertexIndex : complexVertexSet)
	{
		vertexToDifferentTypeMap[_vertexIndex] = -2;
	}

	for (auto& _diskPair : diskVertexMap)
	{
		for (auto _vertexIndex : _diskPair.second)
		{ 
			vertexToDifferentTypeMap[_vertexIndex] = _diskPair.first;
		}
	}

	while (true)
	{
		std::map<unsigned, std::set<unsigned>> allDeleteVertexSet;
		
		for (unsigned i = 0; i < uMeshAreaGroupSize; i++)
		{
			if (diskVertexMap.find(i) == diskVertexMap.end())
			{
				continue;
			}

			auto& growVertexSet = diskVertexMap[i];
			if (growVertexSet.size() == 0)
			{
				continue;
			}

			// Delete some internal complex vertex
			/*
					Area0
				\		    /
				 D--- A ---E
				 C--- B ---F
				/		    \
					Area0

				A, B is internal complex vertex, can not delete both of them in one time.
			*/

			std::set<unsigned> internalComplexVertexSet;

			for (auto vertexIndex : growVertexSet)
			{
				const auto& adjVertexSet = m_vertexAdjInfoVec[vertexIndex];
				std::set<unsigned> sameRegionAdjVertexSet;

				for (auto adjVertexIndex : adjVertexSet)
				{
  					if (vertexToDifferentTypeMap[adjVertexIndex] == -2)
					{
						sameRegionAdjVertexSet.insert(adjVertexIndex);
						continue;
					}

  					if (vertexToDifferentTypeMap[adjVertexIndex] == i)
					{
						sameRegionAdjVertexSet.insert(adjVertexIndex);
						continue;
					}
				}

				if (sameRegionAdjVertexSet.size() <= 2)
				{
					continue;
				}

				std::set<unsigned> tempSet;
				for (auto _vertexIndex : sameRegionAdjVertexSet)
				{
					const auto& _adjVertexSet = m_vertexAdjInfoVec[_vertexIndex];
					for (auto _adjVertexIndex : _adjVertexSet)
					{
						if (vertexToAreaIndexVec[_adjVertexIndex] != i)
						{
							continue;
						}

						tempSet.insert(_adjVertexIndex);
					}
				}

				for (unsigned loopCount = 0; loopCount < 1; loopCount++)
				{
					std::set<unsigned> addVertexSet;
					for (auto _vertexIndex : tempSet)
					{
						const auto& _adjVertexSet = m_vertexAdjInfoVec[_vertexIndex];
						for (auto _adjVertexIndex : _adjVertexSet)
						{
							if (vertexToAreaIndexVec[_adjVertexIndex] != i)
							{
								continue;
							}

							addVertexSet.insert(_adjVertexIndex);
						}
					}

					tempSet.insert(addVertexSet.begin(), addVertexSet.end());
				}

				if (_isOneConnectedRegion(tempSet))
				{
					continue;
				}

				internalComplexVertexSet.insert(vertexIndex);
			}

			std::set<unsigned> reserveVertexSet;

			std::vector<std::set<unsigned>> internalGroups;
			_groupVertexSetByConnection(internalComplexVertexSet, internalGroups);
			for (auto& group : internalGroups)
			{
				std::set<unsigned> usedVertexSet;
				std::set<unsigned> currentLoopSet;
				
				static auto _isTheSameSideInternalVertexPair = [this](unsigned vertexIndex0, unsigned vertexIndex1, const std::set<unsigned>& meshAreaGroup)
				{
					std::set<unsigned> adjvertexSet0;
					std::set<unsigned> adjvertexSet1;

					adjvertexSet0.insert(vertexIndex0);
					adjvertexSet1.insert(vertexIndex1);

					std::set<unsigned>* adjvertexSet[2] = {&adjvertexSet0, &adjvertexSet1};

					for (unsigned i = 0; i < 2; i++)
					{
						for (unsigned j = 0; j < 2; j++)
						{
							for (auto _vertexIndex : *adjvertexSet[j])
							{
								if (meshAreaGroup.count(_vertexIndex) == 0)
								{
									continue;
								}

								const auto& _adjVertexSet = m_vertexAdjInfoVec[_vertexIndex];
								for (auto _adjVertexIndex : _adjVertexSet)
								{
									if (meshAreaGroup.count(_adjVertexIndex) == 0)
									{
										continue;
									}

									adjvertexSet[j]->insert(_adjVertexIndex);
								}
							}
						}
					}

					for (auto _vertex : adjvertexSet0)
					{
						if (adjvertexSet1.count(_vertex) > 0)
						{
							return true;
						}
					}

					return false;
				};

				// For test
				reserveVertexSet.insert(group.begin(), group.end());
				continue;
			}

			// update other diskVertex to avoid connecting different region
			for (auto vertexIndex : growVertexSet)
			{
				const auto& adjVertexSet = m_vertexAdjInfoVec[vertexIndex];
				for (auto adjVertexIndex : adjVertexSet)
				{
					unsigned adjDiskGroupCount = 0;
					for (auto& otherDiskGroup : diskVertexMap)
					{
						if (otherDiskGroup.first == i)
						{
							continue;
						}

						if (vertexToDifferentTypeMap[adjVertexIndex] == otherDiskGroup.first)
						{
							otherDiskGroup.second.erase(adjVertexIndex);
							if (vertexToDifferentTypeMap[adjVertexIndex] != otherDiskGroup.first)
							{
								throw "Exception";
							}

							vertexToDifferentTypeMap[adjVertexIndex] = INT_MAX;

							break;
						}
					}
				}
			}

			for (auto it = growVertexSet.begin(); it != growVertexSet.end();)
			{
				auto vertexIndex = *it;
				if (reserveVertexSet.count(vertexIndex) > 0)
				{
					it++;
					continue;
				}

				allDeleteVertexSet[i].insert(vertexIndex);
				vertexToAreaIndexVec[vertexIndex] = i;
				largestBoundaryVertexSet.erase(vertexIndex);
				it = growVertexSet.erase(it);

				if (vertexToDifferentTypeMap[vertexIndex] != i)
				{
					throw "Exception";
				}

				vertexToDifferentTypeMap[vertexIndex] = INT_MAX;
			}
			
			totalInternalComplexVertexSet.insert(reserveVertexSet.begin(), reserveVertexSet.end());
		}

		if (allDeleteVertexSet.size() == 0)
		{
			bBeginDeleteInternalComplexVertex = true;
			break;
		}

		// update center & complex & disk vertex set
		std::set<unsigned> adjDeleteVertexSet;
		for (auto deleteVertexGroup : allDeleteVertexSet)
		{
			for (auto _vertexIndex : deleteVertexGroup.second)
			{
				const auto& _adjVertexSet = m_vertexAdjInfoVec[_vertexIndex];
				for (auto _adjVertexIndex : _adjVertexSet)
				{
					if (largestBoundaryVertexSet.count(_adjVertexIndex) == 0)
					{
						continue;
					}

					adjDeleteVertexSet.insert(_adjVertexIndex);
				}
			}
		}

		// clear adjVertex type info
		for (auto it = centerVertexSet.begin(); it != centerVertexSet.end();)
		{
			unsigned _vertexIndex = *it;

			if (adjDeleteVertexSet.count(_vertexIndex) == 0)
			{
				it++;
				continue;
			}

			it = centerVertexSet.erase(it);
			if (vertexToDifferentTypeMap[_vertexIndex] != -1)
			{
				throw "Exception";
			}

			vertexToDifferentTypeMap[_vertexIndex] = INT_MAX;
		}
		for (auto it = complexVertexSet.begin(); it != complexVertexSet.end();)
		{
			unsigned _vertexIndex = *it;
			if (adjDeleteVertexSet.count(_vertexIndex) == 0)
			{
				it++;
				continue;
			}

			it = complexVertexSet.erase(it);
			if (vertexToDifferentTypeMap[_vertexIndex] != -2)
			{
				throw "Exception";
			}

			vertexToDifferentTypeMap[_vertexIndex] = INT_MAX;
		}

		for (auto& _diskVertexPair : diskVertexMap)
		{
			for (auto it = _diskVertexPair.second.begin(); it != _diskVertexPair.second.end();)
			{
				unsigned _vertexIndex = *it;
				if (adjDeleteVertexSet.count(_vertexIndex) == 0)
				{
					it++;
					continue;
				}

				it = _diskVertexPair.second.erase(it);

				if (vertexToDifferentTypeMap[_vertexIndex] != _diskVertexPair.first)
				{
					throw "Exception";
				}

				vertexToDifferentTypeMap[_vertexIndex] = INT_MAX;
			}
		}

		std::set<unsigned> _centerVertexSet;
		std::set<unsigned> _complexVertexSet;
		std::map<unsigned, std::set<unsigned>> _diskVertexMap;
		_groupVertexs(adjDeleteVertexSet, largestBoundaryVertexSet, vertexToAreaIndexVec, _centerVertexSet, _complexVertexSet, _diskVertexMap);

		for (auto _vertexIndex : _centerVertexSet)
		{
			centerVertexSet.insert(_vertexIndex);
			vertexToDifferentTypeMap[_vertexIndex] = -1;
		}

		for (auto _vertexIndex : _complexVertexSet)
		{
			complexVertexSet.insert(_vertexIndex);
			vertexToDifferentTypeMap[_vertexIndex] = -2;
		}
		
		for (auto& _diskVertexPair : _diskVertexMap)
		{
			for (auto _vertexIndex : _diskVertexPair.second)
			{
				diskVertexMap[_diskVertexPair.first].insert(_vertexIndex);
				vertexToDifferentTypeMap[_vertexIndex] = _diskVertexPair.first;
			}
		}
		
		for (auto deleteVertexGroup : allDeleteVertexSet)
		{
			for (auto deleteVertexIndex : deleteVertexGroup.second)
			{
				m_currentMeanCurvatureVec[deleteVertexIndex].second = 0.0f;
			}
		}

		// Debug log
		static unsigned uLoopCount = 0;
		std::cout << "loopCount = " << uLoopCount++ << ", allDeleteVertexSet.size = " << allDeleteVertexSet.size() << std::endl;

		if (bOnlyDoOneStep)
		{
			break;
		}
	}

	if (bShow)
	{
		m_pScene->ShowCurvature();
		if (bOnlyDoOneStep)
		{
			m_pScene->UpdateSamplePointGeode(std::vector<unsigned>(totalInternalComplexVertexSet.begin(), totalInternalComplexVertexSet.end()));
		}
	}
}

std::vector<unsigned> ToothSegmentationModule::_generateBoundaryVertexSet(const std::vector<unsigned>& boundaryVertexSet)
{
	unsigned loopCount = m_fMeshDensity * 4.0;

	std::set<unsigned> meshAreaSet;
	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		if (m_currentMeanCurvatureVec[i].second > -FLT_EPSILON)
		{
			meshAreaSet.insert(i);
		}
	}

	std::set<unsigned> largestGroup;
	_getHeadAreaVertexGroupByConnection(meshAreaSet, largestGroup);

	std::set<unsigned> usedVertexSet;
	usedVertexSet.insert(boundaryVertexSet.begin(), boundaryVertexSet.end());
	auto currentLoopSet = boundaryVertexSet;

	std::set<unsigned> maxHeightVertexSet;

	for (unsigned i = 0; i < loopCount; i++)
	{
		std::set<unsigned> nextLoopVertexSet;
		usedVertexSet.insert(currentLoopSet.begin(), currentLoopSet.end());

		for (auto vertexIndex : currentLoopSet)
		{
			const auto& adjVertexSet = m_vertexAdjInfoVec[vertexIndex];
			for (auto adjVertexIndex : adjVertexSet)
			{
				if (usedVertexSet.count(adjVertexIndex) > 0)
				{
					continue;
				}

				if (largestGroup.count(adjVertexIndex) == 0)
				{
					continue;
				}

				if (m_vertexToPlaneDistance[adjVertexIndex] > m_clipPlaneOffset)
				{
					maxHeightVertexSet.insert(vertexIndex);
					continue;
				}

				nextLoopVertexSet.insert(adjVertexIndex);
			}
		}

		usedVertexSet.insert(nextLoopVertexSet.begin(), nextLoopVertexSet.end());
		currentLoopSet = std::vector<unsigned>(nextLoopVertexSet.begin(), nextLoopVertexSet.end());
	}

	for (auto _vertexIndex : maxHeightVertexSet)
	{
		currentLoopSet.push_back(_vertexIndex);
	}

	return currentLoopSet;
}

const std::vector<std::pair<unsigned, float>>& ToothSegmentationModule::GetCurrentMeanCurvatureVec()
{
	return m_currentMeanCurvatureVec;
}

void ToothSegmentationModule::_exportCurvatureDataToDLL()
{
	// Export curvature data to dll.
	float* fCurvatureData = new float[m_currentMeanCurvatureVec.size()];
	for (unsigned i = 0; i < m_currentMeanCurvatureVec.size(); i++)
	{
		fCurvatureData[i] = m_currentMeanCurvatureVec[i].second;
	}

	m_shortestPathProxy.setCurvatureData(reinterpret_cast<char*>(fCurvatureData));
	delete[] fCurvatureData;
}

void ToothSegmentationModule::_exportToothAndGingivalVertexInfo()
{
	std::set<unsigned> boundaryVertexSet;
	std::set<unsigned> meshAreaVertexSet;

	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		if (m_currentMeanCurvatureVec[i].second < m_minMeanCurvature * m_fRegionGrowCurvatureRatio)
		{
			boundaryVertexSet.insert(i);
		}
		else
		{
			meshAreaVertexSet.insert(i);
		}
	}

	std::set<unsigned> largestGroup;
	_getMaxSizeVertexGroupByConnection(boundaryVertexSet, largestGroup);

	std::set<unsigned> exportGingivalVertexSet;
	for (auto vertexIndex : boundaryVertexSet)
	{
		const auto& adjVertexSet = m_vertexAdjInfoVec[vertexIndex];

		bool bIsGingival = false;
		for (auto adjVertexIndex : adjVertexSet)
		{
			if (largestGroup.count(vertexIndex) > 0)
			{
				bIsGingival = true;
				break;
			}
		}

		if (bIsGingival)
		{
			exportGingivalVertexSet.insert(vertexIndex);
		}
	}

	std::ofstream outFile("ToothAndGingivalInfo.bin", std::ios::out | std::ios::binary);
	if (!outFile.is_open())
	{
		throw "FATAL ERROR";
	}

	int nSplitValue = -1;

	// Export tooth feature vertex set
	for (auto vertexIndex : m_featureVertexVec)
	{
		outFile.write(reinterpret_cast<char*>(&vertexIndex), sizeof(unsigned));
	}

	outFile.write(reinterpret_cast<char*>(&nSplitValue), sizeof(unsigned));

	for (auto vertexIndex : exportGingivalVertexSet)
	{
		outFile.write(reinterpret_cast<char*>(&vertexIndex), sizeof(unsigned));
	}

	outFile.write(reinterpret_cast<char*>(&nSplitValue), sizeof(unsigned));

	outFile.close();
}

void ToothSegmentationModule::_curvatureThreshold(float fCurvatureValue)
{
	unsigned uReserveVertexCount = 0;

	for (auto& curvaturePair : m_currentMeanCurvatureVec)
	{
		bool bReserveCurvature = (curvaturePair.second < fCurvatureValue);
		curvaturePair.second = bReserveCurvature ? m_minMeanCurvature : 0.0f;
		
		if (bReserveCurvature)
		{
			uReserveVertexCount++;
		}
	}

	m_pScene->ShowCurvature();
}

void ToothSegmentationModule::_filterSmallLowCurvatureRegion(float fCurvatureValue)
{
	std::set<unsigned> minCurvatureVertexSet;
	for (unsigned i = 0; i < m_currentMeanCurvatureVec.size(); i++)
	{
		if (m_currentMeanCurvatureVec[i].second < fCurvatureValue)
		{
			if (minCurvatureVertexSet.count(i) == 0)
			{
				minCurvatureVertexSet.insert(i);
			}
		}
	}

	std::vector<std::set<unsigned>> lowCurvatureRegionSet;
	_groupVertexSetByConnection(minCurvatureVertexSet, lowCurvatureRegionSet);
	for (const auto& region : lowCurvatureRegionSet)
	{
		/*if (region.size() > m_nRegionGrowFilterMinSize)*/
		if (region.size() > m_fFilterRegionRatio * minCurvatureVertexSet.size())
		{
			continue;
		}

		for (auto vertexIndex : region)
		{
			m_currentMeanCurvatureVec[vertexIndex].second = 0.0f;
		}
	}

	m_pScene->ShowCurvature();
}

void ToothSegmentationModule::UpdateClipPlaneCenter()
{
	m_clipPlaneCenter[0] = m_baseCenter[0] + m_clipPlaneOffset * m_clipPlaneNormal[0];
	m_clipPlaneCenter[1] = m_baseCenter[1] + m_clipPlaneOffset * m_clipPlaneNormal[1];
	m_clipPlaneCenter[2] = m_baseCenter[2] + m_clipPlaneOffset * m_clipPlaneNormal[2];
}

bool ToothSegmentationModule::InitEnergy(bool bPlaneEnergy, float* inData)
{
	if (!bPlaneEnergy)
	{
		// Use mean curvature
		_sortMeanCurvatureVec(true);

		for (unsigned i = 0; i < m_currentMeanCurvatureVec.size(); i++)
		{
			auto curvature = m_currentMeanCurvatureVec[i].second;

			// Map [-1, 1] --> [0, 1]
			inData[i] = 0.5f + curvature * 0.5f;
		}

		return true;
	}

	if (!CalClipPlaneNormal())
	{
		return false;
	}

	unsigned loopCount = 0U;
	float fMoveSpeed = m_fboundingRadius /m_fInitEnergyMoveStepRatio;
	std::map<unsigned, std::pair<unsigned, float>> energyResult;

	float fMinEnergy = FLT_MAX;
	float fMaxEnergy = FLT_MIN;

	unsigned maxEnergyOffset = 0U;
	float fLastMaxEnergy = 0.0f;

	while (true)
	{
		m_clipPlaneOffset += fMoveSpeed;
		UpdateClipPlaneCenter();

		unsigned uIntersectLoopCount = 0;
		float fEnergyValue = CalPlaneEnergy(m_mesh, m_clipPlaneNormal, m_clipPlaneCenter, uIntersectLoopCount, false);

		energyResult[loopCount] = std::make_pair(uIntersectLoopCount, fEnergyValue);
		loopCount++;

		if (fabs(m_clipPlaneOffset) > 2 * m_fboundingRadius)
		{
			break;
		}
	}

	float filterKPer = 0.1f;

	float fEnergyUpper = 0.0f;
	float fEnergyLower = 0.0f;

	std::vector<float> fTempEnergy;
	for (auto& pair : energyResult)
	{
		if (pair.second.second != FLT_MAX)
		{
			fTempEnergy.push_back(pair.second.second);
		}
	}

	std::sort(fTempEnergy.begin(), fTempEnergy.end());
	if (fTempEnergy.size() < 10)
	{
		throw "Energy value too less";
	}

	fEnergyLower = fTempEnergy[fTempEnergy.size() * filterKPer];
	fEnergyUpper = fTempEnergy[fTempEnergy.size() * (1 - filterKPer)];

	for (auto& pair : energyResult)
	{
		if (pair.second.second >= fEnergyUpper || pair.second.second <= fEnergyLower)
		{
			pair.second.second = FLT_MAX;
		}
	}


	for (auto& energyPair : energyResult)
	{
		if (energyPair.second.second != FLT_MAX && fLastMaxEnergy < energyPair.second.second)
		{
			fLastMaxEnergy = energyPair.second.second;
			maxEnergyOffset = energyPair.first;
		}
	}

	m_maxEnergyPlaneOffset = maxEnergyOffset * fMoveSpeed;

	// Filter some energy which offset less than maxEnergyOffset
	std::cout << "Max Energy Value: " << fLastMaxEnergy << std::endl;
	for (unsigned i = 0; i < maxEnergyOffset; i++)
	{
		energyResult[i].second = FLT_MAX;
	}

	for (auto& energyPair : energyResult)
	{
		if (energyPair.second.first != 1)
		{
			energyPair.second.second = FLT_MAX;
		}
	}

	// Find MaxEnergy & MinEnergy
	for (auto energyPair : energyResult)
	{
		auto energy = energyPair.second;
		if (energy.second == FLT_MAX)
		{
			continue;
		}

		if (energy.second < fMinEnergy)
		{
			fMinEnergy = energy.second;
		}

		if (energy.second > fMaxEnergy)
		{
			fMaxEnergy = energy.second;
		}
	}

	// Map Energy to [0.0, 1.0]
	bool bZeroLength = false;
	float fLength = fMaxEnergy - fMinEnergy;
	if (fabs(fLength) < FLT_EPSILON)
	{
		bZeroLength = true;
		std::cout << "fLength equals zero" << std::endl;
	}

	for (auto& value : energyResult)
	{
		if (value.second.second == FLT_MAX)
		{
			value.second.second = 5.0f;
		}
		else
		{
			if (bZeroLength)
			{
				value.second.second = 0.5f;
			}
			else
			{
				value.second.second = (value.second.second - fMinEnergy) / fLength;
			}
		}
	}

	// Reset plane to base position
	m_clipPlaneOffset = 0;
	UpdateClipPlaneCenter();

	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		auto distance = _distanceOfPointToPlane(m_mesh, i, m_clipPlaneNormal, m_clipPlaneCenter);

		unsigned index = distance / fMoveSpeed;
		//float k = distance / fMoveSpeed - index;

		auto value0 = energyResult[index];
		// auto value1 = energyResult[index + 1];

		// inData[i] = (1 - k) * value0 + k * value1;
		inData[i] = value0.second;
	}

	// Find suitable plane
	m_clipPlaneOffset = 0.0f;
	UpdateClipPlaneCenter();

	loopCount = 0;
	float lastEnergy = 1.0f;

	while (true)
	{
		m_clipPlaneOffset += fMoveSpeed;
		if (fabs(m_clipPlaneOffset) > 2 * m_fboundingRadius)
		{
			break;
		}

		if (energyResult[loopCount].second > 0.5f)
		{
			if (energyResult[loopCount].second < 1.1f)
			{
				lastEnergy = energyResult[loopCount].second;
			}
			else if (lastEnergy < 0.35f)
			{
				break;
			}

			loopCount++;
			continue;
		}

		if (energyResult[loopCount].second < 0.2f)
		{
			// Find
			break;
		}

		if (energyResult[loopCount].second < lastEnergy)
		{
			lastEnergy = energyResult[loopCount].second;
			loopCount++;
			continue;
		}
		else
		{
			// Find 
			break;
		}
	}

	UpdateClipPlaneCenter();

	return true;
}

std::vector<unsigned> ToothSegmentationModule::_findFeatureVertexIndex(bool bOnlyHeightFeature, bool bShow)
{
	auto static _isHeightFeaturePointLambda = [this](unsigned i, float fSearchRadius)
	{
		float vertexHeight = m_vertexToPlaneDistance[i];
		if (vertexHeight > m_clipPlaneOffset)
		{
			return false;
		}

		std::set<unsigned> adjVertexSet;
		std::set<unsigned> stack;
		for (auto adjVertexIndex : m_vertexAdjInfoVec[i])
		{
			if (m_vertexToPlaneDistance[adjVertexIndex] < vertexHeight)
			{
				return false;
			}
		}

		stack.insert(i);
		while (stack.size() > 0)
		{
			unsigned index = *(stack.begin());
			
			stack.erase(stack.begin());
			adjVertexSet.insert(index);

			for (auto _index : m_vertexAdjInfoVec[index])
			{
				if (adjVertexSet.count(_index) == 0 && _distanceOfPointToPoint(m_mesh, _index, i) < fSearchRadius)
				{
					if (m_vertexToPlaneDistance[_index] < vertexHeight)
					{
						return false;
					}

					stack.insert(_index);
				}
			}
		}

		return true;
	};

	//float fSearchRadius = 2.0f;
	//float fSearchRadius = 1.5f;
	float fSearchRadius = 1.2f;
	float fFilterRatio = 3.0f;

	// Init vertexToPlaneDistance
	if (m_vertexToPlaneDistance.size() == 0)
	{
		throw "vertexToPlaneDistance need init.";
	}

	// Find all feature vertex
	std::set<unsigned> result;
	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		if (_isHeightFeaturePointLambda(i, fSearchRadius))
		{
			result.insert(i);
		}
	}

	// Filter some vertex
	std::vector<std::pair<unsigned, float>> vertexHeight;
	for (auto& index : result)
	{
		vertexHeight.push_back(std::make_pair(index, m_vertexToPlaneDistance[index]));
	}

	std::sort(vertexHeight.begin(), vertexHeight.end(), [](std::pair<unsigned, float>& lhs, std::pair<unsigned, float>& rhs)
	{
		return lhs.second < rhs.second;
	});

	float fMinHeight = vertexHeight[0].second;
	float fMiddleHeight = vertexHeight[vertexHeight.size() / 2].second;

	float fMaxHeight = fMiddleHeight + fFilterRatio * (fMiddleHeight - fMinHeight);

	for (auto it = result.begin(); it != result.end();)
	{
		if (m_vertexToPlaneDistance[*it] > fMaxHeight)
		{
			it = result.erase(it);
			continue;
		}

		it++;
	}

	std::set<unsigned> meshAreaVertexSet;
	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		if (m_currentMeanCurvatureVec[i].second > m_fRegionGrowCurvatureRatio * m_minMeanCurvature)
		{
			meshAreaVertexSet.insert(i);
		}
	}
	
	std::set<unsigned> largestGroup;
	_getHeadAreaVertexGroupByConnection(meshAreaVertexSet, largestGroup);

	for (auto it = result.begin(); it != result.end();)
	{
		auto vertexIndex = *it;

		if (largestGroup.count(vertexIndex) > 0)
		{
			it = result.erase(it);
		}
		else
		{
			it++;
		}
	}

	if (bOnlyHeightFeature)
	{
		std::vector<unsigned> resultVec(result.begin(), result.end());
		if (bShow)
		{
			m_pScene->UpdateSamplePointGeode(resultVec, osg::Vec4f(0.0f, 1.0f, 0.5f, 1.0f));
		}

		return resultVec;
	}

	// Add curvature feature vertex set
	std::set<unsigned> toothVertexSet;
	for (auto _vertexIndex: meshAreaVertexSet)
	{
		if (largestGroup.count(_vertexIndex) > 0)
		{
			continue;
		}

		toothVertexSet.insert(_vertexIndex);
	}

	std::set<unsigned> highCurvatureVertexSet;
	for (auto vertexIndex : toothVertexSet)
	{
		if (m_actualMeanCurvatureVec[vertexIndex].second > m_maxMeanCurvature * 0.6f)
		{
			highCurvatureVertexSet.insert(vertexIndex);
		}
	}

	if (highCurvatureVertexSet.size() < toothVertexSet.size() * 0.01f)
	{
		std::vector<unsigned> resultVec(result.begin(), result.end());
		if (bShow)
		{
			m_pScene->UpdateSamplePointGeode(resultVec, osg::Vec4f(0.0f, 1.0f, 0.5f, 1.0f));
		}

		return resultVec;
	}

	std::vector<std::set<unsigned>> highCurvatureGroups;
	_groupVertexSetByConnection(highCurvatureVertexSet, highCurvatureGroups);
	//unsigned uReserveRegionVertexCount = toothVertexSet.size() * 0.05f / result.size();
	unsigned uReserveRegionVertexCount = highCurvatureVertexSet.size() * 0.3f / result.size();

	for (auto highVertexSet : highCurvatureGroups)
	{
		if (highVertexSet.size() < uReserveRegionVertexCount)
		{
			continue;
		}

		// Get minHeight vertex as feature vertex
// 		float fMinHeight = FLT_MAX;
// 		int minVertexIndex = -1;
// 		for (auto vertexIndex : highVertexSet)
// 		{
// 			if (m_vertexToPlaneDistance[vertexIndex] < fMinHeight)
// 			{
// 				fMinHeight = m_vertexToPlaneDistance[vertexIndex];
// 				minVertexIndex = vertexIndex;
// 			}
// 		}
// 
// 		if (minVertexIndex == -1)
// 		{
// 			throw "FATAL ERROR";
// 		}
// 
// 		result.insert(minVertexIndex);

		// Random select several vertex as feature vertex
		std::vector<unsigned> tempVec = std::vector<unsigned>(highVertexSet.begin(), highVertexSet.end());

		unsigned uSelectVertexCount = tempVec.size() * 0.01f + 1;
		unsigned uSelectCount = 0U;
		unsigned uCurrentIndex = 0U;
		while (uSelectCount < uSelectVertexCount)
		{
			result.insert(tempVec[uCurrentIndex]);
			uCurrentIndex += 100;
			uCurrentIndex = uCurrentIndex % tempVec.size();
			uSelectCount++;
		}
	}

	std::vector<unsigned> resultVec(result.begin(), result.end());
	if (bShow)
	{
		m_pScene->UpdateSamplePointGeode(resultVec, osg::Vec4f(0.0f, 1.0f, 0.5f, 1.0f));
	}

	return resultVec;
}

std::vector<std::vector<unsigned>> ToothSegmentationModule::_getFeatureGroups(const std::vector<unsigned>& featureVertexs, float fMeshDistanceRatio, float fGroupAbsDistance)
{
	// Grouping these vertex
	// vertex in one group must fit condition :
	//		1. abs distance less than constant
	//		2. each pair abs distance fit shortest mesh distance
	std::vector<std::vector<unsigned>> vertexGroups;

	static auto _findGroupIndex = [this](std::vector<std::vector<unsigned>>& groups, unsigned index, float fMaxAbsDistance)
	{
		for (unsigned i = 0; i < groups.size(); i++)
		{
			auto& group = groups[i];
			for (auto _index : group)
			{
				auto absDistance = _distanceOfPointToPoint(m_mesh, _index, index);
				if (absDistance < fMaxAbsDistance)
				{
					// Get mesh path distance
					if (!m_shortestPathProxy.calShortestPath(_index, index))
					{
						continue;
					}

					unsigned pathLength = m_shortestPathProxy.getShortestPathLength();
					unsigned* pathData = new unsigned[pathLength];
					if (!m_shortestPathProxy.getShortestPathData(pathData))
					{
						throw "getShortestPathData failed.";
					}

					float meshLength = 0.0f;
					for (unsigned j = 0; j < pathLength - 1; j++)
					{
						meshLength += _distanceOfPointToPoint(m_mesh, pathData[j], pathData[j + 1]);
					}

					delete[] pathData;

					// Compare with abs distance
					// if (meshLength < fMeshDistanceRatio * absDistance)
					// if (meshLength < fGroupAbsDistance)
					{
						// Pass
						return static_cast<int>(i);
					}
				}
			}
		}

		return -1;
	};

	for (auto index : featureVertexs)
	{
		auto groupIndex = _findGroupIndex(vertexGroups, index, fGroupAbsDistance);
		if (groupIndex == -1)
		{
			std::vector<unsigned> tempVec;
			tempVec.push_back(index);
			vertexGroups.push_back(tempVec);
		}
		else
		{
			vertexGroups[groupIndex].push_back(index);
		}
	}

	return vertexGroups;
}

std::vector<std::vector<unsigned>> ToothSegmentationModule::_groupFeatureVertexIntoTwoGroup(std::vector<std::vector<unsigned>>& vertexGroups, std::vector<std::vector<unsigned>>& zeroGroups, std::vector<std::vector<unsigned>>& oneGroups)
{
	static auto _calGroupCenterCoord = [&vertexGroups, this](unsigned groupIndex, float* outCoord)
	{
		float sampleVertexCoord[3] = { 0.0f, 0.0f, 0.0f };
		for (auto vertexIndex : vertexGroups[groupIndex])
		{
			float* position = m_mesh.vertex_position + 3 * vertexIndex;
			sampleVertexCoord[0] += position[0];
			sampleVertexCoord[1] += position[1];
			sampleVertexCoord[2] += position[2];
		}

		sampleVertexCoord[0] /= vertexGroups[groupIndex].size();
		sampleVertexCoord[1] /= vertexGroups[groupIndex].size();
		sampleVertexCoord[2] /= vertexGroups[groupIndex].size();

		memcpy(outCoord, sampleVertexCoord, sizeof(float) * 3);
	};

	// Index group (0, 1)
	static auto _getTwoNearGroupIndex = [&vertexGroups, this](unsigned groupIndex, bool bAngleCheck)
	{
		std::set<unsigned> result;
		float sampleVertexCoord[3] = {0.0f, 0.0f, 0.0f};
		_calGroupCenterCoord(groupIndex, sampleVertexCoord);

		for (unsigned j = 0; j < 2; j++)
		{
			int nNearestGroupIndex = -1;
			float fMinDistance = FLT_MAX;

			float lastVertexCoord[3];

			if (result.size() == 1)
			{
				_calGroupCenterCoord(*result.begin(), lastVertexCoord);
			}

			for (unsigned i = 0; i < vertexGroups.size(); i++)
			{
				if (i == groupIndex)
				{
					continue;
				}

				if (result.count(i) > 0)
				{
					continue;
				}

				auto& currentGroup = vertexGroups[i];
				float currentGroupVertexCenter[3];
				_calGroupCenterCoord(i, currentGroupVertexCenter);

				if (bAngleCheck && result.size() == 1)
				{
					// Check angle of two group
					float edge0[3] =
					{
						sampleVertexCoord[0] - currentGroupVertexCenter[0],
						sampleVertexCoord[1] - currentGroupVertexCenter[1],
						sampleVertexCoord[2] - currentGroupVertexCenter[2],
					};

					float edge1[3] =
					{
						sampleVertexCoord[0] - lastVertexCoord[0],
						sampleVertexCoord[1] - lastVertexCoord[1],
						sampleVertexCoord[2] - lastVertexCoord[2],
					};

					if (_vecDot(edge0, edge1) > 0.0f)
					{
						continue;
					}
				}

				float fCurrentGroupMinDistance = _distance(sampleVertexCoord, currentGroupVertexCenter);

				if (fCurrentGroupMinDistance < fMinDistance)
				{
					fMinDistance = fCurrentGroupMinDistance;
					nNearestGroupIndex = i;
				}
			}

			if (nNearestGroupIndex != -1)
			{
				result.insert(nNearestGroupIndex);
			}
		}

		return result;
	};

	std::set<unsigned> zeroValueGroupSet;
	std::set<unsigned> oneValueGroupSet;

	if (vertexGroups.size() <= 2)
	{
		zeroValueGroupSet.insert(0);
		oneValueGroupSet.insert(1);
	}

	else
	{
		int nBeginGroupIndex = -1;
		int nBeginNearestAdjGroupIndex = -1;
		for (unsigned i = 0; i < vertexGroups.size(); i++)
		{
			auto adjGroupIndex = _getTwoNearGroupIndex(i, false);
			std::vector<unsigned> adjGroupIndexVec(adjGroupIndex.begin(), adjGroupIndex.end());
			unsigned adjGroupVertexIndex[2];

			adjGroupVertexIndex[0] = vertexGroups[adjGroupIndexVec[0]].front();
			adjGroupVertexIndex[1] = vertexGroups[adjGroupIndexVec[1]].front();

			unsigned tempVertexIndex = vertexGroups[i].front();

			float currentGroupVertex[3];
			float adjGroupVertex[2][3];
			
			_calGroupCenterCoord(i, currentGroupVertex);
			_calGroupCenterCoord(adjGroupIndexVec[0], adjGroupVertex[0]);
			_calGroupCenterCoord(adjGroupIndexVec[1], adjGroupVertex[1]);

			float vector[2][3];
			vector[0][0] = currentGroupVertex[0] - adjGroupVertex[0][0];
			vector[0][1] = currentGroupVertex[1] - adjGroupVertex[0][1];
			vector[0][2] = currentGroupVertex[2] - adjGroupVertex[0][2];

			vector[1][0] = currentGroupVertex[0] - adjGroupVertex[1][0];
			vector[1][1] = currentGroupVertex[1] - adjGroupVertex[1][1];
			vector[1][2] = currentGroupVertex[2] - adjGroupVertex[1][2];

			if (_vecDot(vector[0], vector[1]) > 0)
			{
				float distance0 = _distance(currentGroupVertex, adjGroupVertex[0]);
				float distance1 = _distance(currentGroupVertex, adjGroupVertex[1]);

				nBeginGroupIndex = i;
				nBeginNearestAdjGroupIndex = (distance0 > distance1) ? adjGroupIndexVec[1] : adjGroupIndexVec[0];
			}
		}

		if (nBeginGroupIndex == -1)
		{
			throw "FATAL ERROR";
		}

		std::set<unsigned> usedGroupIndex;
		std::vector<unsigned> stack;
		stack.push_back(nBeginNearestAdjGroupIndex);

		usedGroupIndex.insert(nBeginGroupIndex);
		zeroValueGroupSet.insert(nBeginGroupIndex);

		bool bIsZeroGroup = false;
		while (stack.size() > 0)
		{
			unsigned currentGroupIndex = stack.front();
			stack.erase(stack.begin());
			usedGroupIndex.insert(currentGroupIndex);

			if (bIsZeroGroup)
			{
				zeroValueGroupSet.insert(currentGroupIndex);
			}
			else
			{
				oneValueGroupSet.insert(currentGroupIndex);
			}
			bIsZeroGroup = !bIsZeroGroup;

			auto adjGroupIndex = _getTwoNearGroupIndex(currentGroupIndex, true);

			unsigned uAddCount = 0;
			for (auto groupIndex : adjGroupIndex)
			{
				if (usedGroupIndex.count(groupIndex) == 0)
				{
					std::cout << "Add groupIndex" << groupIndex << std::endl;
					stack.push_back(groupIndex);
					uAddCount++;
				}
			}

			if (adjGroupIndex.size() != 1 && uAddCount != 1 && usedGroupIndex.size() != vertexGroups.size())
			{
				throw "FATAL ERROR";
			}
		}
	}

	zeroGroups.clear();
	oneGroups.clear();

	for (auto groupIndex : zeroValueGroupSet)
	{
		zeroGroups.push_back(vertexGroups[groupIndex]);
	}

	for (auto groupIndex : oneValueGroupSet)
	{
		oneGroups.push_back(vertexGroups[groupIndex]);
	}

	return vertexGroups;
}