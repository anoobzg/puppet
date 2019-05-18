#include "surface.h"
#include "MeshVertexTraits.h"
#include <iostream>
#include <assert.h>
#include "MeshCurvature.h"

// Debug
void _debugAdjPathInfo(Path* pStartPath)
{
	int nCount = 0;

	std::cout << "NextPtr Check: " << std::endl;
	Path* pTempPath = pStartPath;
	for (pTempPath = pStartPath; pTempPath != nullptr;)
	{
		nCount++;
		std::cout << "Path index"<< nCount << ": " << std::endl;
		for (auto& nodePosition : pTempPath->vertexPathList)
		{
			std::cout << "Position: " << nodePosition.x << " " << nodePosition.y << " " << nodePosition.z << std::endl;
		}

		if (!pTempPath->m_pNextPath)
		{
			break;
		}

		pTempPath = pTempPath->m_pNextPath;
	}

	std::cout << std::endl;
	std::cout << "PrevPtr Check: " << std::endl;

	nCount = 0;
	for (; pTempPath != nullptr;)
	{
		nCount++;
		std::cout << "Path index" << nCount << " : " << std::endl;
		for (auto& nodePosition : pTempPath->vertexPathList)
		{
			std::cout << "Position: " << nodePosition.x << " " << nodePosition.y << " " << nodePosition.z << std::endl;
		}

		pTempPath = pTempPath->m_pPrevPath;
	}
}

float _distance(float vertex1[3], float vertex2[3])
{
	return sqrtf(
		(vertex1[0] - vertex2[0]) * (vertex1[0] - vertex2[0]) +
		(vertex1[1] - vertex2[1]) * (vertex1[1] - vertex2[1]) +
		(vertex1[2] - vertex2[2]) * (vertex1[2] - vertex2[2])
	);
}

ControlPoint::ControlPoint()
{
	static unsigned control_point_id = 0;
	m_handle = control_point_id++;
}

ControlPoint::~ControlPoint()
{

}

unsigned ControlPoint::GetHandle()
{
	return m_handle;
}

Path::Path(unsigned control_point_handle[2]) :m_pPrevPath(nullptr), m_pNextPath(nullptr)
{
	static unsigned path_id = 0;
	m_handle = path_id++;

	m_control_point_handle[0] = control_point_handle[0];
	m_control_point_handle[1] = control_point_handle[1];
}

Path::~Path()
{

}

unsigned Path::GetHandle()
{
	return m_handle;
}

Surface::Surface(Mesh& mesh)
	:m_mesh(mesh), m_topo_callback(0), m_bAutoFinishCircle(true)
{
	m_alg.SetupGraph(m_mesh.vertex_number, m_mesh.vertex_position, m_mesh.triangle_number, m_mesh.triangle_index);
}

Surface::~Surface()
{

}

void Surface::SetSurfaceTopoCallback(SurfaceTopoCallback* callback)
{
	m_topo_callback = callback;
}

bool Surface::AddControlPoint(unsigned vertex_handle, float clickCoord[3])
{
	// Check duplicate control point exist
	for (auto control_point : m_control_points)
	{
		if (control_point.second->m_vertex_index == vertex_handle)
		{
			return false;
		}
	}

	// Check isCirclePath
	if (IsCirclePath())
	{
		return false;
	}

	auto pControlPoint = _createControlPoint(vertex_handle, clickCoord);
	if (pControlPoint == nullptr)
	{
		return false;
	}

	std::pair<ControlPointsIter, bool> result = m_control_points.insert({ pControlPoint->GetHandle(), pControlPoint });
	if (!result.second)
	{
		delete pControlPoint;
		return false;
	}

	m_point_order_list.push_back(pControlPoint);

	if (m_topo_callback)
	{
		m_topo_callback->ControlPointAdded(*pControlPoint);
	}

	return true;
}

bool Surface::AddControlPointPath()
{
	if (m_point_order_list.size() < 2)
	{
		return true;
	}

	unsigned end_control_point_handle = m_point_order_list.back()->GetHandle();
	unsigned start_control_point_handle = (*(--(--m_point_order_list.end())))->GetHandle();
	if (!_createControlPath(start_control_point_handle, end_control_point_handle))
	{
		return false;
	}

	// Auto-generate circle
	if (m_bAutoFinishCircle && m_point_order_list.size() > 2)
	{
		auto firstControlPoint = m_point_order_list.front();
		auto lastControlPoint = m_point_order_list.back();

		float deltaDistance[3];
		deltaDistance[0] = firstControlPoint->x - lastControlPoint->x;
		deltaDistance[1] = firstControlPoint->y - lastControlPoint->y;
		deltaDistance[2] = firstControlPoint->z - lastControlPoint->z;

		float distance = sqrtf(deltaDistance[0] * deltaDistance[0] + deltaDistance[1] * deltaDistance[1] + deltaDistance[2] * deltaDistance[2]);
		if (distance < 1.0f)
		{
			m_point_order_list.push_back(firstControlPoint);
			_createControlPath(lastControlPoint->GetHandle(), firstControlPoint->GetHandle());
		}
	}

	return true;
}

void Surface::DeleteControlPoint(unsigned control_point_handle)
{
	std::list<unsigned> needDeletePathHandleVec;
	for (auto& control_path_info : m_control_point_paths)
	{
		if (control_path_info.second->m_control_point_handle[0] == control_point_handle ||
			control_path_info.second->m_control_point_handle[1] == control_point_handle)
		{
			needDeletePathHandleVec.push_back(control_path_info.first);
		}
	}

	for (auto handle : needDeletePathHandleVec)
	{
		DeleteControlPointPath(handle);
	}

	std::list<std::pair<int, int>> needCreateControlPathVec;
	if (m_point_order_list.front()->GetHandle() != control_point_handle &&
		m_point_order_list.back()->GetHandle() != control_point_handle)
	{
		unsigned start_control_point_handle = 0;
		unsigned end_control_point_handle = 0;

		auto it = m_point_order_list.begin();
		while (it != m_point_order_list.end())
		{
			if ((*it)->GetHandle() == control_point_handle)
			{
				it++;
				end_control_point_handle = (*it)->GetHandle();
				continue;
			}

			unsigned tempIndex = (*it)->GetHandle();
			if ((++it) == m_point_order_list.end())
			{
				break;
			}

			if ((*(it))->GetHandle() == control_point_handle)
			{
				start_control_point_handle = tempIndex;
				continue;
			}
		}

		needCreateControlPathVec.push_back({ start_control_point_handle, end_control_point_handle });
	}

	for (auto& info : needCreateControlPathVec)
	{
		if (!_createControlPath(info.first, info.second))
		{
			std::cout << "_createControlPath failed." << std::endl;
			return;
		}
	}

	for (auto it = m_point_order_list.begin(); it != m_point_order_list.end();)
	{
		if ((*it)->GetHandle() == control_point_handle)
		{
			it = m_point_order_list.erase(it);
			continue;
		}

		it++;
	}

	ControlPoints::iterator it = m_control_points.find(control_point_handle);
	if (it != m_control_points.end())
	{
		if (m_topo_callback) m_topo_callback->ControlPointDeleted(*(*it).second);
		delete (*it).second;
		m_control_points.erase(it);
	}
}

void Surface::DeleteControlPointPath(unsigned control_point_path_handle)
{
	auto it = m_control_point_paths.find(control_point_path_handle);
	if (it != m_control_point_paths.end())
	{
		if ((*it).second->m_pNextPath)
		{
			assert((*it).second->m_pNextPath->m_pPrevPath == (*it).second);
			(*it).second->m_pNextPath->m_pPrevPath = nullptr;
		}

		if ((*it).second->m_pPrevPath)
		{
			assert((*it).second->m_pPrevPath->m_pNextPath == (*it).second);
			(*it).second->m_pPrevPath->m_pNextPath = nullptr;
		}

		if (m_topo_callback) m_topo_callback->PathRemoved(*(*it).second);
		delete (*it).second;
		m_control_point_paths.erase(it);
	}
}

void Surface::UpdateControlPointPath(unsigned control_point_handle, unsigned vertex_handle)
{
	ControlPoints::iterator itPoint = m_control_points.find(control_point_handle);
	if (itPoint == m_control_points.end())
	{
		std::cout << "control_point not found" << std::endl;
		return;
	}

	const unsigned modifyIndex = (*itPoint).second->m_vertex_index;

	std::vector<std::pair<int, int>> updateIndexVec;
	std::vector<int> deletePathVec;

	auto it = m_control_point_paths.begin();
	while(it != m_control_point_paths.end())
	{
		auto& pathInfo = *it;

		unsigned nStartIndex = m_control_points[pathInfo.second->m_control_point_handle[0]]->m_vertex_index;
		unsigned nEndIndex = m_control_points[pathInfo.second->m_control_point_handle[1]]->m_vertex_index;

		if (nStartIndex == modifyIndex || nEndIndex == modifyIndex)
		{
			deletePathVec.push_back(pathInfo.first);

			if (nStartIndex == modifyIndex)
			{
				nStartIndex = vertex_handle;
			}
			else
			{
				nEndIndex = vertex_handle;
			}

			updateIndexVec.push_back({ pathInfo.second->m_control_point_handle[0] , pathInfo.second->m_control_point_handle[1] });
		}

		it++;
	}

	for (auto deletedIndex : deletePathVec)
	{
		DeleteControlPointPath(deletedIndex);
	}

	for (auto& pair : updateIndexVec)
	{
		if (!_createControlPath(pair.first, pair.second))
		{
			std::cout << "FATAL ERROR, Add control point path failed." << std::endl;
			return;
		}
	}
}

void Surface::InsertControlPoint(unsigned vertex_handle, float clickCoord[3])
{
	// Find nearest control path
	const float PI = 3.1415926f;

	float position[3];
	MeshVertexTraits::GetVertexCoord(m_mesh, vertex_handle, position[0], position[1], position[2]);

	unsigned nearestControlPathHandle = 0;
	bool bHasNearestControlPath = false;
	unsigned nearest_vertex_handle[2];
	unsigned nearest_control_point_handle[2];

	auto it = m_point_order_list.begin();
	for (int i = 0; i < m_point_order_list.size() - 1; i++)
	{
		unsigned vertex_handle[2];
		unsigned control_point_handle[2];

		vertex_handle[0] = (*it)->m_vertex_index;
		control_point_handle[0] = (*it)->GetHandle();

		it++;
		vertex_handle[1] = (*it)->m_vertex_index;
		control_point_handle[1] = (*it)->GetHandle();

		float path_position[2][3];
		MeshVertexTraits::GetVertexCoord(m_mesh, vertex_handle[0], path_position[0][0], path_position[0][1], path_position[0][2]);
		MeshVertexTraits::GetVertexCoord(m_mesh, vertex_handle[1], path_position[1][0], path_position[1][1], path_position[1][2]);

		float edgeLength[3];
		edgeLength[0] = _distance(position, path_position[0]);
		edgeLength[1] = _distance(position, path_position[1]);
		edgeLength[2] = _distance(path_position[0], path_position[1]);

		float fAngle = acosf((edgeLength[0] * edgeLength[0] + edgeLength[1] * edgeLength[1] - edgeLength[2] * edgeLength[2]) / (2 * edgeLength[0] * edgeLength[1]));
		if (fAngle > (PI * 12.0f / 18.0f))
		{
			for (auto pathInfo : m_control_point_paths)
			{
				bool bTemp[2];
				unsigned path_vertex_handle[2];
				path_vertex_handle[0] = m_control_points[pathInfo.second->m_control_point_handle[0]]->m_vertex_index;
				path_vertex_handle[1] = m_control_points[pathInfo.second->m_control_point_handle[1]]->m_vertex_index;
				bTemp[0] = path_vertex_handle[0] == vertex_handle[0] || path_vertex_handle[0] == vertex_handle[1];
				bTemp[1] = path_vertex_handle[1] == vertex_handle[0] || path_vertex_handle[1] == vertex_handle[1];

				if (bTemp[0] && bTemp[1])
				{
					nearestControlPathHandle = pathInfo.first;
					break;
				}
			}

			nearest_vertex_handle[0] = vertex_handle[0];
			nearest_vertex_handle[1] = vertex_handle[1];

			nearest_control_point_handle[0] = control_point_handle[0];
			nearest_control_point_handle[1] = control_point_handle[1];

			bHasNearestControlPath = true;
			break;
		}
	}

	if (!bHasNearestControlPath)
	{
		return;
	}

	auto prevAdjControlPath = m_control_point_paths[nearestControlPathHandle]->m_pPrevPath;
	auto nextAdjControlPath = m_control_point_paths[nearestControlPathHandle]->m_pNextPath;
	DeleteControlPointPath(nearestControlPathHandle);

	// create new controlPoint
	auto pInsertControlPoint = _createControlPoint(vertex_handle, clickCoord);
	if (pInsertControlPoint == nullptr)
	{
		std::cout << "createControlPoint failed." << std::endl;
		return;
	}

	std::pair<ControlPointsIter, bool> result = m_control_points.insert({ pInsertControlPoint->GetHandle(), pInsertControlPoint });
	if (!result.second)
	{
		delete pInsertControlPoint;
		return;
	}

	if (m_topo_callback)
	{
		m_topo_callback->ControlPointAdded(*pInsertControlPoint);
	}

	bool bFindInsertPosition = false;
	for (auto it = m_point_order_list.begin(); it != m_point_order_list.end();)
	{
		auto firstIt = it++;
		auto secondIt = it;

		if ((*firstIt)->GetHandle() == nearest_control_point_handle[0] && (*secondIt)->GetHandle() == nearest_control_point_handle[1])
		{
			m_point_order_list.insert(secondIt, pInsertControlPoint);
			bFindInsertPosition = true;
			break;
		}
	}

	if (!bFindInsertPosition)
	{
		std::cout << "FATAL ERROR, can not find insert position." << std::endl;
		return;
	}

	if (!_createControlPath(nearest_control_point_handle[0], pInsertControlPoint->GetHandle()))
	{
		std::cout << "FATAL ERROR, _createControlPath failed." << std::endl;
		return;
	}

	if (!_createControlPath(pInsertControlPoint->GetHandle(), nearest_control_point_handle[1]))
	{
		std::cout << "FATAL ERROR, _createControlPath failed." << std::endl;
		return;
	}
}

bool Surface::ModifyControlPoint(unsigned control_point_handle, unsigned vertex_handle, float clickCoord[3])
{
	ControlPoints::iterator it = m_control_points.find(control_point_handle);
	if (it != m_control_points.end())
	{
		(*it).second->x = clickCoord[0]; (*it).second->y = clickCoord[1]; (*it).second->z = clickCoord[2];
		(*it).second->m_vertex_index = vertex_handle;
		if (m_topo_callback)
		{
			m_topo_callback->ControlPointModified(*(*it).second);
		}

		UpdateControlPointPath(control_point_handle, vertex_handle);

		return true;
	}
	return false;
}

bool Surface::CheckCollideControlPoint(const osg::Matrixf& matrix, const osg::Vec3f& eye, const osg::Vec3f& center, unsigned& control_point_handle)
{
	for (ControlPoints::iterator it = m_control_points.begin(); it != m_control_points.end(); ++it)
	{
		ControlPoint* cp = (*it).second;

		osg::Vec3f local_eye = eye * matrix;
		osg::Vec3f local_center = center * matrix;
		osg::Vec3f point(cp->x, cp->y, cp->z);

		osg::Vec3f eye_to_point = point - local_eye;
		osg::Vec3f eye_to_center = local_center - local_eye;
		eye_to_center.normalize();

		float dot = eye_to_center * eye_to_point;
		if (dot <= 0.0f) continue;

		float slen = eye_to_point.length2();
		if (slen < dot * dot)
			continue;

		float d = sqrtf(slen - dot * dot);
		if (d > 0.3f)
			continue;

		control_point_handle = (*it).first;
		return true;
	}
	return false;
}

ControlPoint * Surface::_createControlPoint(unsigned vertex_handle, float clickCoord[3])
{
	if (vertex_handle >= m_mesh.vertex_number)
		return nullptr;

	ControlPoint* point = new ControlPoint();
	point->x = clickCoord[0];
	point->y = clickCoord[1];
	point->z = clickCoord[2];

	point->m_vertex_index = vertex_handle;

	return point;
}

bool Surface::_createControlPath(unsigned start_control_point_handle, unsigned end_control_point_handle)
{
	std::vector<unsigned> result_vec;
	unsigned start_vertex_handle = m_control_points[start_control_point_handle]->m_vertex_index;
	unsigned end_vertex_handle = m_control_points[end_control_point_handle]->m_vertex_index;

	unsigned vertex_id = end_vertex_handle;
	bool bResult = _doPathFinding(start_vertex_handle, vertex_id, result_vec);
	if (!bResult)
	{
		return false;
	}

	unsigned vertex_indexs[2];
	vertex_indexs[0] = start_control_point_handle;
	vertex_indexs[1] = end_control_point_handle;

	Path* path = new Path(vertex_indexs);
	for (auto vertexIndex : result_vec)
	{
		float position[3];
		MeshVertexTraits::GetVertexCoord(m_mesh, vertexIndex, position[0], position[1], position[2]);
		path->vertexPathList.push_back({ vertexIndex, position[0], position[1], position[2] });
	}

	auto result = m_control_point_paths.insert({ path->GetHandle(), path });

	if (!result.second)
	{
		delete path;
		return false;
	}

	// Update adjPathInfo
	Path* prevPath = nullptr;
	for (auto& pathInfo : m_control_point_paths)
	{
		if (pathInfo.second->m_control_point_handle[1] == path->m_control_point_handle[0])
		{
			if (prevPath)
			{
				prevPath = nullptr;
				break;
			}

			prevPath = pathInfo.second;
		}
	}

	Path* nextPath = nullptr;
	for (auto& pathInfo : m_control_point_paths)
	{
		if (pathInfo.second->m_control_point_handle[0] == path->m_control_point_handle[1])
		{
			if (nextPath)
			{
				nextPath = nullptr;
				break;
			}

			nextPath = pathInfo.second;
		}
	}

	if (prevPath)
	{
		prevPath->m_pNextPath = path;
		path->m_pPrevPath = prevPath;

		// ReGeneratePath
		if (m_topo_callback)
		{
			m_topo_callback->PathRemoved(*prevPath);
			m_topo_callback->PathAdded(*prevPath);
		}
	}

	if (nextPath)
	{
		nextPath->m_pPrevPath = path;
		path->m_pNextPath = nextPath;

		// ReGeneratePath
		if (m_topo_callback)
		{
			m_topo_callback->PathRemoved(*nextPath);
			m_topo_callback->PathAdded(*nextPath);
		}
	}

	if (m_topo_callback)
	{
		m_topo_callback->PathAdded(*path);
	}

	// _debugAdjPathInfo(m_control_point_paths.begin()->second);

	return true;
}

bool Surface::_doPathFinding(int nStartVertexIndex, int nEndVertexIndex, std::vector<unsigned>& result)
{
	if (!m_alg.calShortestPath(nStartVertexIndex, nEndVertexIndex))
	{
		return false;
	}

	int nPathLength = m_alg.getShortestPathLength();
	if (nPathLength <= 0)
		return false;

	unsigned* r = new unsigned[nPathLength];
	if (!m_alg.getShortestPathData(r))
	{
		delete[] r;
		return false;
	}

	result.clear();
	for (int i = 0; i < nPathLength; i++)
	{
		result.push_back(*(r + i));
	}

	delete[] r;

	return true;
}

void Surface::SetCurvatureData(float* curvatureData)
{
	m_alg.setCurvatureData(reinterpret_cast<char*>(curvatureData));
}

void Surface::SetWeightFactor(float m, float n)
{
	m_alg.setWeightFactor(m, n);
}

void Surface::SetCurvatureType(int nUseGuassCurvature)
{
	m_alg.setCurvatureType(nUseGuassCurvature);
}

void Surface::SetupCurvatureArray(float* curve)
{
	m_alg.getCurvatureData((char*)curve);
	//MeshCurvature meshCurvature(m_mesh);
	//meshCurvature.CalCurvature(MinCurvature, curve);
}

bool Surface::ReCreatePath()
{
	// ReCreate control path
	bool bFirstVertex = true;
	unsigned lastVertexId = 0;
	for (auto pControlPoint : m_point_order_list)
	{
		if (bFirstVertex)
		{
			lastVertexId = pControlPoint->GetHandle();
			bFirstVertex = false;
			continue;
		}

		// Ignore return value
		_createControlPath(lastVertexId, pControlPoint->GetHandle());

		lastVertexId = pControlPoint->GetHandle();
	}

	return true;
}

bool Surface::IsCirclePath()
{
	return (m_point_order_list.size() < 3)? 
		false: m_point_order_list.front() == m_point_order_list.back();
}

unsigned Surface::GetProperStartIndex(unsigned index)
{
	return m_alg.getProperStartIndex(index);
}

std::list<unsigned> Surface::GetCirclePath()
{
	std::vector<ControlPoint*> orderPointVec;
	
	if (m_point_order_list.front() == m_point_order_list.back())
	{
		orderPointVec.assign(m_point_order_list.begin(), m_point_order_list.end());
	}

	std::list<unsigned> result;
	if (m_point_order_list.size() == 0)
	{
		return result;
	}

	for (int i = 0; i < orderPointVec.size() - 1; i++)
	{
		unsigned controlPoints[2];
		controlPoints[0] = orderPointVec[i]->GetHandle();
		controlPoints[1] = orderPointVec[i + 1]->GetHandle();

		for (auto& control_path : m_control_point_paths)
		{
			if (control_path.second->m_control_point_handle[0] == controlPoints[0] &&
				control_path.second->m_control_point_handle[1] == controlPoints[1])
			{
				result.push_back(control_path.second->vertexPathList.front().vertex_handle);

				for (auto& point : control_path.second->vertexPathList)
				{
					if (point.vertex_handle != control_path.second->vertexPathList.front().vertex_handle &&
						point.vertex_handle != control_path.second->vertexPathList.back().vertex_handle)
					{
						result.push_back(point.vertex_handle);
					}
				}

				break;
			}
		}
	}

	result.push_back(orderPointVec.back()->m_vertex_index);

	return result;
}

bool Surface::ClipMesh(unsigned clipCircleVertexSize, unsigned * clipCircleVertexData, unsigned userClickTriangleVertexData[3])
{
	return m_alg.clipMesh(clipCircleVertexSize, clipCircleVertexData, userClickTriangleVertexData);
}

unsigned Surface::getClipMeshPrimitiveSize()
{
	return m_alg.getClipMeshPrimitiveSize();
}

void Surface::getClipMeshPrimitiveData(unsigned * pPrimitiveData)
{
	m_alg.getClipMeshPrimitiveData(pPrimitiveData);
}
