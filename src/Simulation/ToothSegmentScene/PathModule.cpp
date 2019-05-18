#include "PathModule.h"
#include <fstream>
#include <osg/LineWidth>
#include <osg/Depth>
#include <iostream>

void _calBezierControlPointInfo(
	Path::Point3D& vertex0, 
	Path::Point3D& vertex1, 
	Path::Point3D& vertex2, 
	float outControlPoint[2][3]
)
{
	float middlePoint[2][3];
	middlePoint[0][0] = (vertex1.x + vertex0.x) / 2.0f;
	middlePoint[1][0] = (vertex1.x + vertex2.x) / 2.0f;
	middlePoint[0][1] = (vertex1.y + vertex0.y) / 2.0f;
	middlePoint[1][1] = (vertex1.y + vertex2.y) / 2.0f;
	middlePoint[0][2] = (vertex1.z + vertex0.z) / 2.0f;
	middlePoint[1][2] = (vertex1.z + vertex2.z) / 2.0f;

	float fLength[2];
	fLength[0] =
		(vertex1.x - vertex0.x) * (vertex1.x - vertex0.x) +
		(vertex1.y - vertex0.y) * (vertex1.y - vertex0.y) +
		(vertex1.z - vertex0.z) * (vertex1.z - vertex0.z);

	fLength[1] =
		(vertex1.x - vertex2.x) * (vertex1.x - vertex2.x) +
		(vertex1.y - vertex2.y) * (vertex1.y - vertex2.y) +
		(vertex1.z - vertex2.z) * (vertex1.z - vertex2.z);

	fLength[0] = sqrtf(fLength[0]);
	fLength[1] = sqrtf(fLength[1]);

	float k;
	k = fLength[0] / (fLength[0] + fLength[1]);

	float tempPoint[3];
	tempPoint[0] = middlePoint[1][0] - middlePoint[0][0];
	tempPoint[1] = middlePoint[1][1] - middlePoint[0][1];
	tempPoint[2] = middlePoint[1][2] - middlePoint[0][2];

	outControlPoint[0][0] = vertex1.x - tempPoint[0] * k;
	outControlPoint[0][1] = vertex1.y - tempPoint[1] * k;
	outControlPoint[0][2] = vertex1.z - tempPoint[2] * k;

	outControlPoint[1][0] = vertex1.x + tempPoint[0] * (1 - k);
	outControlPoint[1][1] = vertex1.y + tempPoint[1] * (1 - k);
	outControlPoint[1][2] = vertex1.z + tempPoint[2] * (1 - k);
}

PathModule::PathModule(
	osg::ref_ptr<ManipulableNode> manipulable_node, 
	LauncaGeometry::Mesh& mesh, std::shared_ptr<Surface> surface
) :
	m_manipulable_node(manipulable_node), 
	m_surface(surface), m_hove_ball(nullptr), 
	m_selected_ball(nullptr), 
	m_bSmoothControlPath(false), 
	m_bUserClickCorrect(true),
	m_nSmoothFactorPair({ 5, 50 })
{
	SetNodeData();
	m_surface->SetSurfaceTopoCallback(this);
}

PathModule::~PathModule()
{
	m_manipulable_node->RemoveChild(m_control_node);
	m_manipulable_node->RemoveChild(m_control_path_node);
}

bool PathModule::ReLoadWeight()
{
	// Reload config file to modify weight 
	const char* configFilePath = "config.ini";
	std::ifstream configFile(configFilePath, std::ios::in);
	if (!configFile.is_open())
	{
		return false;
	}

	char temp[256];

	configFile.getline(temp, 256);
	float m = std::atof(temp);

	configFile.getline(temp, 256);
	float n = std::atof(temp);

	configFile.close();

	m_surface->SetWeightFactor(m, n);

	return true;
}

bool PathModule::ReLoadSmoothFactor()
{
	// Reload config file to modify weight 
	const char* configFilePath = "smoothConfig.ini";
	std::ifstream configFile(configFilePath, std::ios::in);
	if (!configFile.is_open())
	{
		return false;
	}

	float m, n;
	char temp[256];
	configFile.getline(temp, 256);
	m = std::atoi(temp);

	configFile.getline(temp, 256);
	n = std::atoi(temp);

	configFile.close();

	m_nSmoothFactorPair = { m, n };

	return true;
}

bool PathModule::ReCreatePath()
{
	for (auto& path : m_control_paths)
	{
		auto it = m_control_paths.find(path.first);
		if (it != m_control_paths.end())
		{
			m_control_points_path_geode->removeChild((*it).second);
		}
		m_surface->DeleteControlPointPath(path.first);
	}
	m_control_paths.clear();

	return m_surface->ReCreatePath();
}

bool PathModule::IsCirclePath()
{
	return m_surface->IsCirclePath();
}

void PathModule::HoverControlPoint(unsigned handle)
{
	std::map<unsigned, osg::ref_ptr<ControlBall>>::iterator it = m_control_balls.find(handle);
	if (m_hove_ball) m_hove_ball->Unhover();
	m_hove_ball = 0;

	if (it != m_control_balls.end())
	{
		m_hove_ball = (*it).second;
		if (m_hove_ball) m_hove_ball->Hover();
	}
}

void PathModule::_generateBezierCurve(
	std::vector<Path::Point3D>& inputPath, 
	std::list<Path::Point3D>& outputPath, 
	Path* pPrevPath, 
	Path* pNextPath
)
{
	// Help : http://www.antigrain.com/research/bezier_interpolation/index.html#PAGE_BEZIER_INTERPOLATION
	if (inputPath.size() < 1)
	{
		return;
	}

	// Input data pre-process
	int nTempCount = 0;
	int nLoopCount = 0;
	int nInputPathDataSize = inputPath.size();
	
	for (auto it = inputPath.begin(); it != inputPath.end();)
	{
		nLoopCount++;

		if (nLoopCount == nInputPathDataSize || nLoopCount == 1)
		{
			it++;
			continue;
		}

		if (nTempCount < m_nSmoothFactorPair.first)
		{
			nTempCount++;
			it = inputPath.erase(it);
			continue;
		}
		
		nTempCount = 0;
		it++;
	}

	std::map<int, std::pair<Path::Point3D, Path::Point3D>> controlPointMap;

	if (inputPath.size() == 0)
	{
		return;
	}

	bool bHeadControlPointExist = false;
	bool bTailControlPointExist = false;

	// Modify head and tail part
	do
	{
		if (pPrevPath)
		{
			while (pPrevPath != nullptr && pPrevPath->vertexPathList.size() < 1)
			{
				pPrevPath = pPrevPath->m_pPrevPath;
			}

			if (!pPrevPath)
			{
				break;
			}

			auto it = pPrevPath->vertexPathList.end();
			it--;

			if (pPrevPath->vertexPathList.size() > m_nSmoothFactorPair.first)
			{
				for (int i = 0; i < m_nSmoothFactorPair.first; i++)
				{
					it--;
				}
			}
			else
			{
				it = pPrevPath->vertexPathList.begin();
			}

			Path::Point3D& prevAdjNode = (*it);

			float controlPoint[2][3];
			
			_calBezierControlPointInfo(prevAdjNode, inputPath[0], inputPath[1], controlPoint);
			controlPointMap[0].first.x = controlPoint[0][0];
			controlPointMap[0].first.y = controlPoint[0][1];
			controlPointMap[0].first.z = controlPoint[0][2];

			controlPointMap[0].second.x = controlPoint[1][0];
			controlPointMap[0].second.y = controlPoint[1][1];
			controlPointMap[0].second.z = controlPoint[1][2];

			bHeadControlPointExist = true;
		}
	}
	while (false);

	do
	{
		if (pNextPath)
		{
			while (pNextPath != nullptr && pNextPath->vertexPathList.size() < 1)
			{
				pNextPath = pNextPath->m_pNextPath;
			}

			if (!pNextPath)
			{
				break;
			}

			auto it = pNextPath->vertexPathList.begin();

			if (pNextPath->vertexPathList.size() > m_nSmoothFactorPair.first)
			{
				for (int i = 0; i < m_nSmoothFactorPair.first; i++)
				{
					it++;
				}
			}
			else
			{
				it = pNextPath->vertexPathList.end();
				it--;
			}

			Path::Point3D nextAdjNode = pNextPath->vertexPathList.front();
			if (it != pNextPath->vertexPathList.end())
			{
				nextAdjNode = (*it);
			}

			float controlPoint[2][3];

			_calBezierControlPointInfo(
				inputPath[inputPath.size() - 2], 
				inputPath[inputPath.size() - 1], 
				nextAdjNode, 
				controlPoint
			);

			controlPointMap[inputPath.size() - 1].first.x = controlPoint[0][0];
			controlPointMap[inputPath.size() - 1].first.y = controlPoint[0][1];
			controlPointMap[inputPath.size() - 1].first.z = controlPoint[0][2];

			controlPointMap[inputPath.size() - 1].second.x = controlPoint[1][0];
			controlPointMap[inputPath.size() - 1].second.y = controlPoint[1][1];
			controlPointMap[inputPath.size() - 1].second.z = controlPoint[1][2];

			bTailControlPointExist = true;
		}
	} 
	while (false);

	for (int i = 1; i < inputPath.size() - 1; i ++)
	{
		float controlPoint[2][3];
		_calBezierControlPointInfo(
			inputPath[i - 1], 
			inputPath[i], 
			inputPath[i + 1], 
			controlPoint
		);

		controlPointMap[i].first.x = controlPoint[0][0];
		controlPointMap[i].first.y = controlPoint[0][1];
		controlPointMap[i].first.z = controlPoint[0][2];

		controlPointMap[i].second.x = controlPoint[1][0];
		controlPointMap[i].second.y = controlPoint[1][1];
		controlPointMap[i].second.z = controlPoint[1][2];
	}

	// Generate bezier curve
	outputPath.push_back(inputPath[0]);

	for (int i = 0; i < inputPath.size() - 1; i++)
	{
		Path::Point3D& controlPoint0 = controlPointMap[i].second;
		Path::Point3D& controlPoint1 = controlPointMap[i + 1].first;

		// 3 - Bezier curve : 
		// Pt = P0 * (1 - t)^3 + 3 * P1 * t * (1 - t)^2 + 3 * P2 * t^2 * (1 - t) + P3 * t^3
		// t = [0, 1]

		bool bSkipInterpolate = 
			((i == 0) && !bHeadControlPointExist) || 
			((i == inputPath.size() - 2) && !bTailControlPointExist);
		
		if (!bSkipInterpolate)
		{
			float nTStep = 1.0f / (m_nSmoothFactorPair.second + 1);
			float t = nTStep;

			for (int j = 0; j < m_nSmoothFactorPair.second; j++)
			{
				float OneMinusT = 1 - t;
				float OMTSquare = OneMinusT * OneMinusT;
				float OMTCube = OMTSquare * OneMinusT;

				float TSquare = t * t;
				float TCube = TSquare * t;

				float fX = inputPath[i].x * OMTCube + 3 * controlPoint0.x * OMTSquare * t + 3 * controlPoint1.x * OneMinusT * TSquare + inputPath[i + 1].x * TCube;
				float fY = inputPath[i].y * OMTCube + 3 * controlPoint0.y * OMTSquare * t + 3 * controlPoint1.y * OneMinusT * TSquare + inputPath[i + 1].y * TCube;
				float fZ = inputPath[i].z * OMTCube + 3 * controlPoint0.z * OMTSquare * t + 3 * controlPoint1.z * OneMinusT * TSquare + inputPath[i + 1].z * TCube;

				outputPath.push_back({ 0, fX, fY, fZ });
				t += nTStep;
			}
		}

		outputPath.push_back(inputPath[i + 1]);
	}
}

void PathModule::ControlPointModified(ControlPoint & control_point)
{
	unsigned handle = control_point.GetHandle();

	std::map<unsigned, osg::ref_ptr<ControlBall>>::iterator it = m_control_balls.find(handle);
	if (it != m_control_balls.end())
	{
		(*it).second->SetPosition(control_point.x, control_point.y, control_point.z);
	}
}

void PathModule::ControlPointAdded(ControlPoint & control_point)
{
	unsigned handle = control_point.GetHandle();

	ControlBall* geometry = new ControlBall(handle);
	geometry->SetPosition(control_point.x, control_point.y, control_point.z);
	m_control_balls.insert(std::pair<unsigned, osg::ref_ptr<ControlBall>>(handle, geometry));
	m_control_points_geode->addDrawable(geometry);
}

void PathModule::ControlPointDeleted(ControlPoint & control_point)
{
	unsigned handle = control_point.GetHandle();

	std::map<unsigned, osg::ref_ptr<ControlBall>>::iterator it = m_control_balls.find(handle);
	if (it != m_control_balls.end())
	{
		m_control_points_geode->removeChild((*it).second);
		m_control_balls.erase(it);

		std::cout << "Remove " << handle << std::endl;
	}
}

void PathModule::PathAdded(Path & pathInfo)
{
	std::list <Path::Point3D> outputPathList;
	std::vector <Path::Point3D> tempDataVec(
		pathInfo.vertexPathList.begin(), 
		pathInfo.vertexPathList.end()
	);

	if (m_bSmoothControlPath)
	{
		_generateBezierCurve(tempDataVec, outputPathList, pathInfo.m_pPrevPath, pathInfo.m_pNextPath);
	}
	else
	{
		outputPathList.assign(tempDataVec.begin(), tempDataVec.end());
	}

	unsigned path_size = (unsigned)outputPathList.size();
	if (path_size == 0)
	{
		return;
	}

	float* vertex = new float[3 * path_size];
	float* t = vertex;
	for (auto it = outputPathList.begin(); it != outputPathList.end(); it++)
	{
		*t = (*it).x;
		*(t + 1) = (*it).y;
		*(t + 2) = (*it).z;
		t += 3;
	}

	osg::Array* coord_array = OSGWrapper::ArrayCreator::CreateVec3Array(path_size, vertex);
	delete[] vertex;

	osg::DrawArrays* primitive_set = new osg::DrawArrays(GL_LINE_STRIP, 0, path_size);

	ControlPath* geometry = new ControlPath(
		pathInfo.GetHandle(), 
		pathInfo.m_control_point_handle, 
		primitive_set, 
		coord_array
	);

	geometry->setCullingActive(false);

	m_control_points_path_geode->addDrawable(geometry);
	m_control_paths.insert({ pathInfo.GetHandle(), geometry });
}

void PathModule::PathRemoved(Path & path)
{
	auto& controlPath = m_control_paths[path.GetHandle()];
	m_control_points_path_geode->removeChild(controlPath);
	m_control_paths.erase(path.GetHandle());
}

void PathModule::SetNodeData()
{
	m_control_node = new AttributeUtilNode();
	m_control_node->SetRenderProgram("ball430"); 

	m_manipulable_node->AddChild (m_control_node);

	m_control_points_geode = new osg::Geode();
	m_control_points_geode->setCullingActive(false);
	m_control_node->AddChild(m_control_points_geode);

	m_control_path_node = new AttributeUtilNode();
	m_control_path_node->SetRenderProgram("purecolor430");
	m_control_path_node->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f)));
	m_control_path_node->SetAttribute(new osg::LineWidth(4.0f));
	m_control_path_node->SetAttribute(new osg::Depth(osg::Depth::ALWAYS));
	m_manipulable_node->AddChild(m_control_path_node);

	m_control_points_path_geode = new osg::Geode();
	m_control_points_path_geode->setCullingActive(false);
	m_control_path_node->AddChild(m_control_points_path_geode);
}

std::list<unsigned> PathModule::GetPathPrimitives()
{
	std::list<unsigned> result;

	if (!IsCirclePath())
	{
		return result;
	}

	auto circleVertexs = m_surface->GetCirclePath();
	if (circleVertexs.size() == 0)
	{
		return result;
	}

	typedef unsigned VertexHandleType;
	std::map<VertexHandleType, unsigned> circleVertexMap;

	unsigned i = 1;
	for (auto it = circleVertexs.begin(); it != circleVertexs.end(); it++, i++)
	{
		circleVertexMap[*it] = i;
	}

	for (unsigned i = 0; i < m_surface->m_mesh.triangle_number; i++)
	{
		unsigned vertexHandles[3];

		vertexHandles[0] = *(m_surface->m_mesh.triangle_index + 3 * i);
		vertexHandles[1] = *(m_surface->m_mesh.triangle_index + 3 * i + 1);
		vertexHandles[2] = *(m_surface->m_mesh.triangle_index + 3 * i + 2);

		unsigned indexs[2];
		unsigned nCount = 0;
		if (circleVertexMap.count(vertexHandles[0]) > 0)
		{
			indexs[nCount] = circleVertexMap[vertexHandles[0]];
			nCount++;
		}

		if (circleVertexMap.count(vertexHandles[1]) > 0)
		{
			indexs[nCount] = circleVertexMap[vertexHandles[1]];
			nCount++;
		}

		if (circleVertexMap.count(vertexHandles[2]) > 0)
		{
			indexs[nCount] = circleVertexMap[vertexHandles[2]];
			nCount++;

			if (circleVertexMap.count(vertexHandles[0]) > 0)
			{
				auto temp = indexs[0];
				indexs[0] = indexs[1];
				indexs[1] = temp;
			}
		}
		
		if (nCount >= 2 && indexs[0] > indexs[1])
		{
			result.push_back(i + 1);
		}
	}

	return result;
}

std::list<unsigned> PathModule::GetPathVertexIndexs()
{
	return m_surface->GetCirclePath();
}
