#pragma once
#include <set>
#include <map>
#include <vector>
#include <osgGA/EventQueue>
#include <osgGA/GUIActionAdapter>
#include "AlgorithmPipeLine.h"
#include "ShortestPath.h"
#include "Mesh.h"
#include "ToothSegmentScene.h"
#include "harmonic_caculator.h"

using namespace LauncaGeometry;
class ToothSegmentScene;

class ToothSegmentationModule
{
public:
	ToothSegmentationModule(Mesh& mesh, ToothSegmentScene* pScene);

	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool InitEnergy(bool bPlaneEnergy, float* inData);
	bool CalClipPlaneNormal(bool bShowSampleVertex = true);
	void UpdateClipPlaneCenter();
		
	float CalPlaneEnergy(const Mesh& mesh, float* m_clipPlaneNormal, float* m_clipPlaneCenter, unsigned& outLoopCount, bool OneLoopCheckOpen, bool bShowIntersectVertex = false);
	const std::vector<std::pair<unsigned, float>>& GetCurrentMeanCurvatureVec();

	float m_clipPlaneOffset;
	float m_baseCenter[3];
	float m_clipPlaneNormal[3];
	float m_clipPlaneCenter[3];
private:
	void _reFindBoundary(bool bShow);
	void _updateHarmonicField(const std::vector<unsigned>& base, const std::vector<unsigned>& neg, const std::vector<unsigned>& pos, bool bShow);
	void _updateAlgorithmParameters(std::string configFile);
	void _resetAlgorithmState();
	void _reLocatePlane(bool bShow);
	void _leftLargestLowCurvatureRegion(bool bShow);

	void _initMeshDensity();
	void _constructAlgPipeLine();

	void _setupEdgeInfo(Mesh &mesh);
	void _setupMeshAdjInfo(Mesh &mesh);
	bool _setupMeshInfo(Mesh& mesh);

	void _sortMinCurvatureVec(bool bSortByIndex);
	void _sortMeanCurvatureVec(bool bSortByIndex);

	void _splitVertexIntoTwoGroups(std::set<unsigned>& boundarySet, std::set<unsigned>& meshAreaSet);
	void _fillSmallMeshArea(bool bShow);
	void _addToothPartLowCurvatureInfo(bool bShow);
	void _curvatureThreshold(float fCurvatureValue);
	void _filterSmallLowCurvatureRegion(float fCurvatureValue);
	void _regionGrow(unsigned uGrowLoopCount, bool bShowCurvature);
	void _regionGrowUtilOneRegion(bool bShowCurvature);
	void _getBoundaryPath(float fCurvatureValue);
	void _getBoundarySkeleton(float fCurvatureValue, bool bShowCurvature, bool bDebugMode);
	std::vector<unsigned> _generateBoundaryVertexSet(const std::vector<unsigned>& boundaryVertexSet);

	void _exportCurvatureDataToDLL();
	void _exportToothAndGingivalVertexInfo();
	void _exportCurvatureData();
	void _exportPlaneInfoAndFeatureInfo();
	void _exportEnergyVertexInfo(const std::vector<unsigned>& planeVertexVec, const std::vector<std::vector<unsigned>>& zeroVertexVec, const std::vector<std::vector<unsigned>>& oneVertexVec);
	void _exportAllGroupsInfo(const std::vector<std::vector<unsigned>>& groups, const std::set<unsigned>& bounadarySet);

	std::vector<std::set<unsigned>> _groupVertexSetByConnection(std::set<unsigned> vertexSet);
	bool _isOneConnectedRegion(std::set<unsigned> vertexSet);
	bool _groupVertexSetByConnection(std::set<unsigned> vertexSet, std::vector<std::set<unsigned>>& outResult);
	bool _getMaxSizeVertexGroupByConnection(std::set<unsigned> vertexSet, std::set<unsigned>& outResult);
	bool _getHeadAreaVertexGroupByConnection(std::set<unsigned> vertexSet, std::set<unsigned>& outResult);

	std::vector<unsigned> _findFeatureVertexIndex(bool bOnlyHeightFeature, bool bShow);
	
	std::vector<std::vector<unsigned>> _getFeatureGroups(const std::vector<unsigned>& featureVertexs, float fMeshDistanceRatio, float fGroupAbsDistance);
	std::vector<std::vector<unsigned>> _groupFeatureVertexIntoTwoGroup(std::vector<std::vector<unsigned>>& vertexGroups, std::vector<std::vector<unsigned>>& zeroGroups, std::vector<std::vector<unsigned>>& oneGroups);
	
	// Mesh attribute
	float m_boundingCenter[3];
	float m_fboundingRadius;
	
	unsigned m_nSampleVertexCount;
	float m_maxEnergyPlaneOffset;
	float m_minMeanCurvature;
	float m_maxMeanCurvature;
	float m_fMeshDensity;

	Mesh& m_mesh;
	ShortestPath m_shortestPathProxy;
	AlgorithmPipeLine m_algorithmPipeLine;
	ToothSegmentScene* m_pScene;
	std::auto_ptr<HarmonicCaculator> m_harmonicCalculator;

	// Const data
	std::map<std::pair<unsigned, unsigned>, std::vector<unsigned>> m_edge;
	std::vector<std::pair<double, double>>	m_triangleToPlaneDistances;
	std::vector<float> m_vertexToPlaneDistance;
	std::vector<std::pair<unsigned, float>> m_minCurvatureVec;
	std::vector<std::pair<unsigned, float>> m_actualMeanCurvatureVec;
	std::vector<std::set<unsigned>> m_vertexAdjInfoVec;

	// Algorithm middle data
	std::vector<unsigned> m_boundaryVertexVec;
	std::vector<unsigned> m_sampleVertexIndexVec;
	std::vector<unsigned> m_featureVertexVec;
	std::vector<std::vector<unsigned>> m_originalFeatureGroups;
	std::vector<std::vector<unsigned>> m_zeroGroups;
	std::vector<std::vector<unsigned>> m_oneGroups;
	std::vector<std::pair<unsigned, float>> m_currentMeanCurvatureVec;

	float m_fPreRegionGrowLoopCount = 2.0f;
	// Algorithm parameters
	float m_fPreRegionGrowRatio = 0.4f;
	int m_nRegionGrowFilterMinSize = -1;
	float m_fFilterRegionRatio = 0.03f;
	float m_fInitEnergyMoveStepRatio = 500.0f;
	float m_fRegionGrowCurvatureRatio = 0.50f;
	float m_fNearFeatureDistance = 8.0f;
	float m_fConnectFeatureDistanceRatio = 1.0f;
	float m_fGrowLoopRatio = 1.5f;
	float m_fGroupFeatureMeshDistanceRatio = 1.5f;
	float m_fGroupAbsDistance = 12.0f;
	float m_fNearFeatureGroupRatio = 0.01f;
};