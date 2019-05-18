#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AttributeUtilNode.h>
#include "Mesh.h"
#include "PathModule.h"
#include "osg\PolygonMode"
#include "AlgorithmPipeLine.h"
#include "ToothSegmentationModule.h"

using namespace LauncaGeometry;
using namespace OSGWrapper;

class ToothSegmentationModule;

class ToothSegmentScene : public RenderScene
{
public:
	ToothSegmentScene(Mesh& mesh);
	~ToothSegmentScene();

	void ShowCurvature();
	void UpdateGroupVertexResult(std::vector<std::vector<unsigned>>& groupVertexs, std::vector<std::vector<unsigned>>& zeroVertexs, std::vector<std::vector<unsigned>>& oneVertexs);
	void UpdateSampleTriangleGeode(const std::vector<unsigned>& triIndexVec);
	void UpdateSamplePointGeode(const std::vector<unsigned>& vertexIndexVec, osg::Vec4f color = osg::Vec4f(0.0f, 1.0f, 1.0f, 0.5f), bool bClearOlderPoint = true);
	void UpdateSamplePointGeode(const std::set<unsigned>& vertexIndexVec, osg::Vec4f color = osg::Vec4f(0.0f, 1.0f, 1.0f, 0.5f), bool bClearOlderPoint = true);
	void UpdateSamplePointGeode(float* fVertexCoord, unsigned uVertexSize, osg::Vec4f color = osg::Vec4f(0.0f, 1.0f, 1.0f, 0.5f), bool bClearOlderPoint = true);
	bool SetPlaneGeode(bool bNeedAdjust, bool bUpdateSampleVertex);
protected:
	// Scene config
	void ResetCamera();
	void Setup();

	// Event Callback
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

private:
	bool _adjustPlanePosition();
	osg::Geode* _createMeshGeode(Mesh& mesh, bool bShowPlaneEnergy);
	
	// OSG data
	osg::ref_ptr<osg::PolygonMode>		m_polygon_mode;
	osg::ref_ptr<ManipulableNode>		m_manipulableNode;
	osg::ref_ptr<Manipulator>			m_manipulator;
	osg::ref_ptr<AttributeUtilNode>		m_renderNode;
	osg::ref_ptr<AttributeUtilNode>		m_clipPlaneNode;
	osg::ref_ptr<AttributeUtilNode>		m_samplePointNode;
	osg::ref_ptr<AttributeUtilNode>		m_maxEnergyPointNode;
	osg::ref_ptr<AttributeUtilNode>		m_energyCubeNode;

	osg::ref_ptr<osg::Geode>			m_meshGeode;
	osg::ref_ptr<osg::Geode>			m_clipPlaneGeode;
	osg::ref_ptr<osg::Geode>			m_energyCubeGeode;
	osg::ref_ptr<osg::Geode>			m_samplePointGeode;
	osg::ref_ptr<osg::Geode>			m_maxEnergyPointGeode;

	Mesh& m_mesh;
	ToothSegmentationModule* m_pToothSegmentationModule;
};