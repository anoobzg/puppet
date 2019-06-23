#include <functional>   // std::greater
#include <algorithm>    // std::sort
#include <numeric>
#include "projectionicpmapping.h"
#include <omp.h>
#include "get_boundingbox_corner.h"
#include "compute_boundingbox.h"

namespace trimesh
{
	ProjectionICPMapping::ProjectionICPMapping(float fx, float fy, float cx, float cy)
		:m_fx(fx), m_fy(fy), m_cx(cx), m_cy(cy), m_source(NULL)
		, m_target(NULL), m_tracer(NULL)
	{
	}

	ProjectionICPMapping::~ProjectionICPMapping()
	{

	}

	void ProjectionICPMapping::SetSource(trimesh::TriMesh* source)
	{
		m_source = source;
	}

	void ProjectionICPMapping::SetTarget(trimesh::TriMesh* target)
	{
		m_target = target;
	}

	bool ProjectionICPMapping::Valid()
	{
		return m_source && m_target;
	}

	void ProjectionICPMapping::Setup()
	{
		const trimesh::box3& source_box = m_source->bbox;
		const trimesh::box3& target_box = m_target->bbox;

		GetBoundingboxCorner(source_box, m_source_corner);
	}

	void ProjectionICPMapping::SetTracer(ProjectionICPTracer* tracer)
	{
		m_tracer = tracer;
	}

	void ProjectionICPMapping::Mapping(const trimesh::xform &source_form, std::vector<PtPair> &pairs)
	{
		pairs.clear();

		trimesh::xform nxf2 = source_form;
		nxf2[12] = 0;
		nxf2[13] = 0;
		nxf2[14] = 0;
		trimesh::xform xf12 = inv(source_form);
		trimesh::xform nxf12 = xf12;
		nxf12[12] = 0;
		nxf12[13] = 0;
		nxf12[14] = 0;

		float xmin, xmax, ymin, ymax, zmin, zmax;
		TransformBoundingbox(m_source_corner, source_form,
			xmin, xmax, ymin, ymax, zmin, zmax);

		const int sz = floor(m_source->vertices.size() / 1000);
		int nv = (int)m_target->vertices.size() / sz;
		std::vector<PtPair> 	pairs1(nv);
		std::vector<int> idx(nv, 0);
		int num_omp = omp_get_num_procs();
		//#pragma omp parallel for num_threads(num_omp)
		{
#pragma omp parallel for num_threads(num_omp)
			for (int i = 0; i < nv; i++)
			{
				int index = i * sz;
				const trimesh::point& target_point = m_target->vertices.at(index);
				if (target_point.x > xmin && target_point.x < xmax &&
					target_point.y > ymin && target_point.y < ymax &&
					target_point.z > zmin && target_point.z < zmax)
				{
					trimesh::point p = xf12 * target_point;

					// Project source vertex into the destination's image plane.
					int u = (int)roundf((m_fx * p[0] + m_cx * p[2]) / p[2]);
					int v = (int)roundf((m_fy * p[1] + m_cy * p[2]) / p[2]);

					// Check corresponding vertex
					if ((u >= 1 && u < m_source->grid_width - 1) && (v >= 1 && v < m_source->grid_height - 1)) {
						int j = u + v * m_source->grid_width;
						if (m_source->grid[j] >= 0 && m_source->grid[j] < m_source->vertices.size())
						{
							// Project both points into world coords and save
							const trimesh::vec& n1 = m_target->normals[index];
							trimesh::point p2 = source_form * m_source->vertices[m_source->grid[j]];
							trimesh::vec n2 = nxf2 * m_source->normals[m_source->grid[j]];
							if (n1.dot(n2) > 0.6)
							{
								pairs1[i] = PtPair(target_point, n1, p2, n2);
								idx[i] = 1;
							}
						}
					}
				}
			}
		}
		int num = std::accumulate(idx.begin(), idx.end(), 0);
		int k = num;
		int n = 0;

		pairs.resize(k);
		for (int i = 0; i < nv; i++)
		{
			if (idx[i] == 1)
			{
				pairs[n] = pairs1[i];
				n++;
			}
		}

	}
} // namespace test_trimesh
