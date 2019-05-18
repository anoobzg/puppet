#include <osg\PolygonMode>
#include <osg\Point>
#include <osg\BlendFunc>
#include <osg\Math>
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeodeCreator.h>
#include <osg\Depth>
#include <Eigenvalues>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "MeshCurvature.h"
#include "Base.h"
#include "CubeEnergy.h"
#include "OSGCube.h"
#include "ShortestPath.h"
#include "CollideAlgrithm.h"
#include "ToothSegmentScene.h"

void _normalize(float* vector3)
{
	float length = sqrtf(vector3[0] * vector3[0] + vector3[1] * vector3[1] + vector3[2] * vector3[2]);
	if (length > 1e-5f)
	{
		vector3[0] /= length;
		vector3[1] /= length;
		vector3[2] /= length;
	}
}

ToothSegmentScene::ToothSegmentScene(Mesh & mesh)
	:m_mesh(mesh)
{
	m_pToothSegmentationModule = new ToothSegmentationModule(m_mesh, this);

	m_manipulableNode = new ManipulableNode();
	m_manipulableNode->UseModelUniform();
	m_manipulator = new Manipulator(*m_manipulableNode);
	addChild(m_manipulableNode);

	m_renderNode = new AttributeUtilNode();
	m_renderNode->SetRenderProgram("energycolor430");
	m_renderNode->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL));
	m_manipulableNode->AddChild(m_renderNode);

	m_polygon_mode = new osg::PolygonMode();
	m_renderNode->SetAttribute(m_polygon_mode);

	m_clipPlaneNode = new AttributeUtilNode();
	m_clipPlaneNode->SetRenderProgram("purecolor430");
	m_clipPlaneNode->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 1.0f, 0.0f, 0.5f)));
	m_clipPlaneNode->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL));
	m_manipulableNode->AddChild(m_clipPlaneNode);

	m_samplePointNode = new AttributeUtilNode();
	m_samplePointNode->SetRenderProgram("purecolor430");
	m_samplePointNode->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL));
	m_samplePointNode->SetAttribute(new osg::Depth(osg::Depth::ALWAYS));
	m_samplePointNode->SetAttribute(new osg::Point(5));
	m_manipulableNode->AddChild(m_samplePointNode);

	m_maxEnergyPointNode = new AttributeUtilNode();
	m_maxEnergyPointNode->SetRenderProgram("purecolor430");
	m_maxEnergyPointNode->AddUniform(new osg::Uniform("color", osg::Vec4f(0.0f, 0.0f, 1.0f, 0.5f)));
	m_maxEnergyPointNode->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL));
	m_maxEnergyPointNode->SetAttribute(new osg::Point(10));
	m_manipulableNode->AddChild(m_maxEnergyPointNode);

	m_energyCubeNode = new AttributeUtilNode();
	m_energyCubeNode->SetRenderProgram("purecolor430");
	m_energyCubeNode->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL));
	m_energyCubeNode->SetAttribute(new osg::Point(10));
	m_manipulableNode->AddChild(m_energyCubeNode);

	m_clipPlaneGeode = new osg::Geode();
	m_clipPlaneGeode->setCullingActive(false);
	m_clipPlaneNode->AddChild(m_clipPlaneGeode);

	m_samplePointGeode = new osg::Geode();
	m_samplePointGeode->setCullingActive(false);
	m_samplePointNode->AddChild(m_samplePointGeode);

	m_maxEnergyPointGeode = new osg::Geode();
	m_maxEnergyPointGeode->setCullingActive(false);
	m_maxEnergyPointNode->AddChild(m_maxEnergyPointGeode); 
	
	m_energyCubeGeode = new osg::Geode();
	m_energyCubeGeode->setCullingActive(false);
	m_energyCubeNode->AddChild(m_energyCubeGeode);

	Setup();
	ResetCamera();
	SetPlaneGeode(false, false);
}

ToothSegmentScene::~ToothSegmentScene()
{
}

void ToothSegmentScene::ResetCamera()
{
	const osg::BoundingSphere& sphere = getBound();
	osg::Vec3f center = sphere.center();
	float radius = 0.6f * sphere.radius();

	float fovy = GetFovy();
	float len = radius / sin(fovy * 3.1415926f / 180.0f / 2.0f);
	osg::Vec3f eye = center + osg::Vec3f(0.0f, 0.0f, -1.0f) * len;
	osg::Vec3f up = osg::Vec3f(0.0f, -1.0f, 0.0f);
	osg::Matrixf view_matrix = osg::Matrixf::lookAt(eye, center, up);

	SetViewMatrix(view_matrix);
	//SetNearFar(len - 3.1f * radius, len + 3.1f * radius);
	SetNearFar(0.01f, 10000.0f);
}

void ToothSegmentScene::Setup()
{
	if (!m_meshGeode.valid())
	{
		m_meshGeode = _createMeshGeode(m_mesh, true);
	}

	m_renderNode->RemoveAll();
	m_renderNode->AddChild(m_meshGeode);
}

bool ToothSegmentScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool ToothSegmentScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_P)
	{
		static bool line_mode = false;
		line_mode = !line_mode;

		m_polygon_mode->setMode(osg::PolygonMode::FRONT_AND_BACK, line_mode ? osg::PolygonMode::LINE : osg::PolygonMode::FILL);
		return true;
	}

	return m_pToothSegmentationModule->OnKey(ea, aa);
}

bool ToothSegmentScene::SetPlaneGeode(bool bNeedAdjust, bool bUpdateSampleVertex)
{
	static bool bNormalInited = false;
	if (!bNormalInited && !m_pToothSegmentationModule->CalClipPlaneNormal())
	{
		return false;
	}
	bNormalInited = true;

	if (bUpdateSampleVertex)
	{
		// Add sample point to geode
		std::vector<unsigned> m_sampleVertexIndices;
		for (auto& curvature : m_pToothSegmentationModule->GetCurrentMeanCurvatureVec())
		{
			m_sampleVertexIndices.push_back(curvature.first);
		};
		UpdateSamplePointGeode(m_sampleVertexIndices);
	}

	if (bNeedAdjust && !_adjustPlanePosition())
	{
		return false;
	}

	m_pToothSegmentationModule->UpdateClipPlaneCenter();

	auto planeLength = 2 * m_meshGeode->getBoundingBox().radius();

	float planePoint[4][3];
	planePoint[0][0] = 0.0f;
	planePoint[0][1] = 1.0f;
	planePoint[0][2] = -m_pToothSegmentationModule->m_clipPlaneNormal[1] / m_pToothSegmentationModule->m_clipPlaneNormal[2];

	_normalize(planePoint[0]);
	MeshCurvature::_vecCross(m_pToothSegmentationModule->m_clipPlaneNormal, planePoint[0], planePoint[1]);
	MeshCurvature::_vecCross(m_pToothSegmentationModule->m_clipPlaneNormal, planePoint[1], planePoint[2]);
	MeshCurvature::_vecCross(m_pToothSegmentationModule->m_clipPlaneNormal, planePoint[2], planePoint[3]);

	for (int i = 0; i < 4; i++)
	{
		planePoint[i][0] *= planeLength;
		planePoint[i][1] *= planeLength;
		planePoint[i][2] *= planeLength;
	}

	float planeVertex[12] =
	{
		m_pToothSegmentationModule->m_clipPlaneCenter[0] + planePoint[0][0], m_pToothSegmentationModule->m_clipPlaneCenter[1] + planePoint[0][1], m_pToothSegmentationModule->m_clipPlaneCenter[2] + planePoint[0][2],
		m_pToothSegmentationModule->m_clipPlaneCenter[0] + planePoint[1][0], m_pToothSegmentationModule->m_clipPlaneCenter[1] + planePoint[1][1], m_pToothSegmentationModule->m_clipPlaneCenter[2] + planePoint[1][2],
		m_pToothSegmentationModule->m_clipPlaneCenter[0] + planePoint[2][0], m_pToothSegmentationModule->m_clipPlaneCenter[1] + planePoint[2][1], m_pToothSegmentationModule->m_clipPlaneCenter[2] + planePoint[2][2],
		m_pToothSegmentationModule->m_clipPlaneCenter[0] + planePoint[3][0], m_pToothSegmentationModule->m_clipPlaneCenter[1] + planePoint[3][1], m_pToothSegmentationModule->m_clipPlaneCenter[2] + planePoint[3][2],
	};

	osg::Array* coordArray = OSGWrapper::ArrayCreator::CreateVec3Array(4, planeVertex);
	osg::DrawArrays* primitiveSet = new osg::DrawArrays(GL_QUADS, 0, 4);
	
	osg::ref_ptr<osg::Geometry> planeGeode = new osg::Geometry();
	planeGeode->setVertexAttribArray(0, coordArray, osg::Array::BIND_PER_VERTEX);
	planeGeode->addPrimitiveSet(primitiveSet);
	planeGeode->setUseVertexBufferObjects(true);
	planeGeode->setUseDisplayList(false);
	planeGeode->setCullingActive(false);

	m_clipPlaneGeode->removeChildren(0, m_clipPlaneGeode->getNumChildren());
	m_clipPlaneGeode->addDrawable(planeGeode);

	return true;
}

void ToothSegmentScene::UpdateSamplePointGeode(const std::vector<unsigned>& vertexIndexVec, osg::Vec4f color, bool bClearOlderPoint)
{
	float* pointVertex = new float[vertexIndexVec.size() * 3];

	for (unsigned i = 0; i < vertexIndexVec.size(); i++)
	{
		float* vertexPosition = m_mesh.vertex_position + 3 * vertexIndexVec[i];
		pointVertex[3 * i] = vertexPosition[0];
		pointVertex[3 * i + 1] = vertexPosition[1];
		pointVertex[3 * i + 2] = vertexPosition[2];
	}

	UpdateSamplePointGeode(pointVertex, vertexIndexVec.size(), color, bClearOlderPoint);
	
	delete[] pointVertex;
}

void ToothSegmentScene::UpdateSamplePointGeode(float* fVertexCoord, unsigned uVertexSize, osg::Vec4f color /*= osg::Vec4f(0.0f, 0.0f, 1.0f, 0.5f)*/, bool bClearOlderPoint /*= true*/)
{
	osg::Array* coordArray = OSGWrapper::ArrayCreator::CreateVec3Array(uVertexSize, fVertexCoord);
	osg::DrawArrays* primitiveSet = new osg::DrawArrays(GL_POINTS, 0, uVertexSize);

	osg::ref_ptr<osg::Geometry> sampleVertexGeode = new osg::Geometry();
	sampleVertexGeode->setVertexAttribArray(0, coordArray, osg::Array::BIND_PER_VERTEX);
	sampleVertexGeode->addPrimitiveSet(primitiveSet);
	sampleVertexGeode->setUseVertexBufferObjects(true);
	sampleVertexGeode->setUseDisplayList(false);
	sampleVertexGeode->setCullingActive(false);
	sampleVertexGeode->getOrCreateStateSet()->addUniform(new osg::Uniform("color", color));

	if (bClearOlderPoint)
	{
		m_samplePointGeode->removeChildren(0, m_samplePointGeode->getNumChildren());
	}

	m_samplePointGeode->addDrawable(sampleVertexGeode);
}

void ToothSegmentScene::UpdateSamplePointGeode(const std::set<unsigned>& vertexIndexSet, osg::Vec4f color /*= osg::Vec4f(0.0f, 1.0f, 1.0f, 0.5f)*/, bool bClearOlderPoint /*= true*/)
{
	return UpdateSamplePointGeode(std::vector<unsigned>(vertexIndexSet.begin(), vertexIndexSet.end()), color, bClearOlderPoint);
}

void ToothSegmentScene::UpdateSampleTriangleGeode(const std::vector<unsigned>& triIndexVec)
{
	float* pointVertex = new float[triIndexVec.size() * 3 * 3];

	for (unsigned i = 0; i < triIndexVec.size(); i++)
	{
		unsigned* index = m_mesh.triangle_index + triIndexVec[i] * 3;

		for (unsigned j = 0; j < 3; j++)
		{
			float* vertexPosition = m_mesh.vertex_position + 3 * index[j];
			pointVertex[9 * i + 3 * j] = vertexPosition[0];
			pointVertex[9 * i + 3 * j + 1] = vertexPosition[1];
			pointVertex[9 * i + 3 * j + 2] = vertexPosition[2];
		}
	}

	osg::Array* coordArray = OSGWrapper::ArrayCreator::CreateVec3Array(3 * triIndexVec.size(), pointVertex);
	osg::DrawArrays* primitiveSet = new osg::DrawArrays(GL_TRIANGLES, 0, 3 * triIndexVec.size());

	osg::ref_ptr<osg::Geometry> sampleVertexGeode = new osg::Geometry();
	sampleVertexGeode->setVertexAttribArray(0, coordArray, osg::Array::BIND_PER_VERTEX);
	sampleVertexGeode->addPrimitiveSet(primitiveSet);
	sampleVertexGeode->setUseVertexBufferObjects(true);
	sampleVertexGeode->setUseDisplayList(false);
	sampleVertexGeode->setCullingActive(false);

	m_samplePointGeode->removeChildren(0, m_samplePointGeode->getNumChildren());
	m_samplePointGeode->addDrawable(sampleVertexGeode);

	delete[] pointVertex;
}

void ToothSegmentScene::UpdateGroupVertexResult(std::vector<std::vector<unsigned>>& groupVertexs, std::vector<std::vector<unsigned>>& zeroVertexs, std::vector<std::vector<unsigned>>& oneVertexs)
{
	// Clear sampleNode data
	m_samplePointGeode->removeChildren(0, m_samplePointGeode->getNumChildren());

	// Create lines per group
	for (auto& group : zeroVertexs)
	{
		float* pointVertex = new float[group.size() * 3];

		for (unsigned i = 0; i < group.size(); i++)
		{
			float* vertexPosition = m_mesh.vertex_position + 3 * group[i];
			pointVertex[3 * i] = vertexPosition[0];
			pointVertex[3 * i + 1] = vertexPosition[1];
			pointVertex[3 * i + 2] = vertexPosition[2];
		}

		osg::Array* coordArray = OSGWrapper::ArrayCreator::CreateVec3Array(group.size(), pointVertex);
		osg::DrawArrays* primitiveSet = new osg::DrawArrays((group.size() == 1) ? GL_POINTS : GL_LINE_STRIP, 0, group.size());

		osg::ref_ptr<osg::Geometry> sampleVertexGeode = new osg::Geometry();
		sampleVertexGeode->setVertexAttribArray(0, coordArray, osg::Array::BIND_PER_VERTEX);
		sampleVertexGeode->addPrimitiveSet(primitiveSet);
		sampleVertexGeode->setUseVertexBufferObjects(true);
		sampleVertexGeode->setUseDisplayList(false);
		sampleVertexGeode->setCullingActive(false);
		sampleVertexGeode->getOrCreateStateSet()->addUniform(new osg::Uniform("color", osg::Vec4f(0.0f, 0.0f, 1.0f, 1.0f)));

		m_samplePointGeode->addDrawable(sampleVertexGeode);

		delete[] pointVertex;
	}

	for (auto& group : oneVertexs)
	{
		float* pointVertex = new float[group.size() * 3];

		for (unsigned i = 0; i < group.size(); i++)
		{
			float* vertexPosition = m_mesh.vertex_position + 3 * group[i];
			pointVertex[3 * i] = vertexPosition[0];
			pointVertex[3 * i + 1] = vertexPosition[1];
			pointVertex[3 * i + 2] = vertexPosition[2];
		}

		osg::Array* coordArray = OSGWrapper::ArrayCreator::CreateVec3Array(group.size(), pointVertex);
		osg::DrawArrays* primitiveSet = new osg::DrawArrays((group.size() == 1) ? GL_POINTS : GL_LINE_STRIP, 0, group.size());

		osg::ref_ptr<osg::Geometry> sampleVertexGeode = new osg::Geometry();
		sampleVertexGeode->setVertexAttribArray(0, coordArray, osg::Array::BIND_PER_VERTEX);
		sampleVertexGeode->addPrimitiveSet(primitiveSet);
		sampleVertexGeode->setUseVertexBufferObjects(true);
		sampleVertexGeode->setUseDisplayList(false);
		sampleVertexGeode->setCullingActive(false);
		sampleVertexGeode->getOrCreateStateSet()->addUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f)));

		m_samplePointGeode->addDrawable(sampleVertexGeode);

		delete[] pointVertex;
	}
}

bool ToothSegmentScene::_adjustPlanePosition()
{
	bool bInitDirection = false;
	float fLastEnergyValue = FLT_MAX;
	float fMoveSpeed = 0.5f;
	float fMinEnergy = FLT_MAX;
	float fOffset = 0.0f;

	while (true)
	{
		if (!bInitDirection)
		{
			bInitDirection = true;

			unsigned uLoopCount = 0;
			float energyValue0 = m_pToothSegmentationModule->CalPlaneEnergy(m_mesh, m_pToothSegmentationModule->m_clipPlaneNormal, m_pToothSegmentationModule->m_baseCenter, uLoopCount, true);

			float tempCenter[3];
			m_pToothSegmentationModule->m_clipPlaneOffset += fMoveSpeed;

			tempCenter[0] = m_pToothSegmentationModule->m_baseCenter[0] + m_pToothSegmentationModule->m_clipPlaneOffset * m_pToothSegmentationModule->m_clipPlaneNormal[0];
			tempCenter[1] = m_pToothSegmentationModule->m_baseCenter[1] + m_pToothSegmentationModule->m_clipPlaneOffset * m_pToothSegmentationModule->m_clipPlaneNormal[1];
			tempCenter[2] = m_pToothSegmentationModule->m_baseCenter[2] + m_pToothSegmentationModule->m_clipPlaneOffset * m_pToothSegmentationModule->m_clipPlaneNormal[2];

			float energyValue1 = m_pToothSegmentationModule->CalPlaneEnergy(m_mesh, m_pToothSegmentationModule->m_clipPlaneNormal, tempCenter, uLoopCount, true);

			if (energyValue0 < energyValue1)
			{
				fMoveSpeed = -fMoveSpeed;
			}
		}

		m_pToothSegmentationModule->m_clipPlaneOffset += fMoveSpeed;
		m_pToothSegmentationModule->UpdateClipPlaneCenter();

		unsigned uLoopCount;
		float fEnergyValue = m_pToothSegmentationModule->CalPlaneEnergy(m_mesh, m_pToothSegmentationModule->m_clipPlaneNormal, m_pToothSegmentationModule->m_clipPlaneCenter, uLoopCount, true);
		
		if (fEnergyValue < fMinEnergy)
		{
			std::cout << "update min value" << fEnergyValue << std::endl;
			fOffset = m_pToothSegmentationModule->m_clipPlaneOffset;
			fMinEnergy = fEnergyValue;
		}

		if (fabs(m_pToothSegmentationModule->m_clipPlaneOffset) > 2 * m_meshGeode->getBound().radius())
		{
			break;
		}
	}

	m_pToothSegmentationModule->m_clipPlaneOffset = fOffset;
	m_pToothSegmentationModule->UpdateClipPlaneCenter();

	return true;
}

void ToothSegmentScene::ShowCurvature()
{
	m_meshGeode = _createMeshGeode(m_mesh, false);
	m_renderNode->RemoveAll();
	m_renderNode->AddChild(m_meshGeode);
}

osg::Geode* ToothSegmentScene::_createMeshGeode(Mesh& mesh, bool bShowPlaneEnergy)
{
	osg::Array* coord_array = OSGWrapper::ArrayCreator::CreateVec3Array(mesh.vertex_number, mesh.vertex_position);
	osg::Array* normal_array = OSGWrapper::ArrayCreator::CreateVec3Array(mesh.vertex_number, mesh.vertex_normal);

	float* energy = new float[mesh.vertex_number];
	if (!m_pToothSegmentationModule->InitEnergy(bShowPlaneEnergy, energy))
	{
		return nullptr;
	}

	osg::Array* energy_array = OSGWrapper::ArrayCreator::CreateFloatArray(mesh.vertex_number, energy);

	delete[] energy;

	osg::PrimitiveSet* primitive_set = OSGWrapper::ArrayCreator::CreatePrimitiveSet(osg::PrimitiveSet::TRIANGLES, 3 * mesh.triangle_number, mesh.triangle_index);
	return OSGWrapper::GeodeCreator::CreateIndexAttributeGeode(primitive_set, coord_array, energy_array, normal_array);
}