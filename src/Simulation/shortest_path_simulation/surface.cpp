#include "surface.h"
#include "MeshLoader.h"

#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeodeCreator.h>
#include <osgWrapper\GeometryCreator.h>
#include <assert.h>
#include <iostream>

#include "MeshVertexTraits.h"

using namespace LauncaGeometry;
Surface::Surface(const char* file)
{
	m_mesh.reset(MeshLoader::LoadFromFileName(file));

	if (m_mesh.get())
		m_algrithm.SetupGraph(m_mesh->vertex_number, m_mesh->vertex_position, m_mesh->triangle_number, m_mesh->triangle_index);

	pCurvatureData = (float*)malloc(m_mesh->vertex_number * sizeof(float));
	GetCurvatureData(reinterpret_cast<char*>(pCurvatureData));

	m_path = new osg::Geode();
	m_path->setCullingActive(false);
}

Surface::~Surface()
{
	if (pCurvatureData)
	{
		free(pCurvatureData);
		pCurvatureData = nullptr;
	}
}

Mesh* Surface::GetMesh()
{
	return m_mesh.get();
}

osg::Geode* Surface::GetGeode()
{
	if (!m_geode.valid() && m_mesh.get())
	{
		osg::Array* coord_array = OSGWrapper::ArrayCreator::CreateVec3Array(m_mesh->vertex_number, m_mesh->vertex_position);
		osg::Array* normal_array = OSGWrapper::ArrayCreator::CreateVec3Array(m_mesh->vertex_number, m_mesh->vertex_normal);

		float* color = new float[4 * m_mesh->vertex_number];
		for (unsigned i = 0; i < 4 * m_mesh->vertex_number; ++i)
			*(color + i) = 1.0f;
		osg::Array* color_array = OSGWrapper::ArrayCreator::CreateVec4Array(m_mesh->vertex_number, color);
		osg::PrimitiveSet* primitive_set = OSGWrapper::ArrayCreator::CreatePrimitiveSet(osg::PrimitiveSet::TRIANGLES, 3 * m_mesh->triangle_number, m_mesh->triangle_index);
		m_geode = OSGWrapper::GeodeCreator::CreateIndexAttributeGeode(primitive_set, coord_array, normal_array, color_array);
	}
	return m_geode;
}

osg::Geode* Surface::GetPath()
{
	return m_path;
}

void Surface::GetCurvatureData(char* pCurvatureData)
{
	m_algrithm.getCurvatureData(pCurvatureData);
}

void Surface::AddControlPoint(unsigned triangle_index, bool bCorrect)
{
	if (!m_mesh.get() || triangle_index >= m_mesh->triangle_number)
		return;

	unsigned vertex_id = *(m_mesh->triangle_index + 3 * triangle_index);
	unsigned nCorrectEndIndex = vertex_id;
	unsigned nCorrectStartIndex = m_control_points.back();

	if (m_control_points.size() < 1)
	{
		// First vertex need be corrected.
		m_control_points.push_back(nCorrectEndIndex);
		return;
	}

	std::vector<unsigned> result_vec;
	unsigned last_vertex_id = m_control_points.back();
	bool bResult = _doPathFinding(last_vertex_id, vertex_id, result_vec);
	if (!bResult)
		return;

	if (bCorrect)
	{
		nCorrectEndIndex = GetCorrectEndIndex();
		nCorrectStartIndex = GetCorrectStartIndex();

		std::cout << "CorrectStartIndex: " << nCorrectStartIndex << ", OriginalStartIndex: " << last_vertex_id << std::endl;
		std::cout << "CorrectEndIndex: " << nCorrectEndIndex << ", OriginalEndIndex: " << vertex_id << std::endl;
		
		if (nCorrectEndIndex != vertex_id)
		{
			auto it = result_vec.begin();
			bool bNeedClip = false;
			while (it != result_vec.end())
			{
				if (*it == nCorrectEndIndex)
				{
					bNeedClip = true;
					break;
				}

				it++;
			}

			if (bNeedClip)
			{
				result_vec.erase(it, result_vec.end());
			}
		}

		// Correct first vertex
		if (m_control_points.size() == 1)
		{
			m_control_points.front() = nCorrectStartIndex;

			if (nCorrectStartIndex != last_vertex_id)
			{
				auto it = result_vec.begin();
				bool bNeedClip = false;
				while (it != result_vec.end())
				{
					if (*it == nCorrectStartIndex)
					{
						bNeedClip = true;
						break;
					}

					it++;
				}

				if (bNeedClip)
				{
					result_vec.erase(result_vec.begin(), it);
				}
			}
		}
	}

	m_control_points.push_back(nCorrectEndIndex);

	CreateOnePath(result_vec);
	return;
}

bool Surface::_doPathFinding(int nStartVertexIndex, int nEndVertexIndex, std::vector<unsigned>& result)
{
	if (!m_algrithm.calShortestPath(nStartVertexIndex, nEndVertexIndex))
	{
		return false;
	}

	int nPathLength = m_algrithm.getShortestPathLength();
	if (nPathLength <= 0)
		return false;

	unsigned* r = new unsigned[nPathLength];
	if (!m_algrithm.getShortestPathData(r))
	{
		delete[] r;
		return false;
	}
	
	result.clear();
	for (int i = 0; i < nPathLength; i++)
	{
		result.push_back(*(r+i));
	}
	
	delete[] r;

	return true;
}

unsigned Surface::GetCorrectEndIndex()
{
	return m_algrithm.getCorrectEndIndex();
}

unsigned Surface::GetCorrectStartIndex()
{
	return m_algrithm.getCorrectStartIndex();
}

void Surface::CreateOnePath(std::vector<unsigned>& path)
{
	unsigned path_size = (unsigned)path.size();
	if (path_size == 0)
		return;

	float* vertex = new float[3 * path_size];
	float* t = vertex;
	for (unsigned i = 0; i < path_size; ++i)
	{
		MeshVertexTraits::GetVertexCoord(*m_mesh, path[i], *t, *(t + 1), *(t + 2));
		t += 3;
	}

	osg::Array* coord_array = OSGWrapper::ArrayCreator::CreateVec3Array(path_size, vertex);
	delete[] vertex;

	osg::DrawArrays* primitive_set = new osg::DrawArrays(GL_LINE_STRIP, 0, path_size);
	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(primitive_set, coord_array);
	geometry->setCullingActive(false);

	osg::DrawArrays* point_set = new osg::DrawArrays(GL_POINTS, 0, 1);
	osg::Geometry* point = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(point_set, coord_array);
	point->setCullingActive(false);

	m_path->addDrawable(geometry);
	m_path->addDrawable(point);
}
