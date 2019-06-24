#ifndef TEST_ICP_H
#define TEST_ICP_H

#include "projectionicpheader.h"
#include<sstream>

namespace trimesh 
{
	class ProjectionICPMapping
	{
	public:
		ProjectionICPMapping(float fx, float fy, float cx, float cy);
		~ProjectionICPMapping();

		void SetSource(trimesh::TriMesh* source);
		void SetTarget(trimesh::TriMesh* target);
		bool Valid();
		void Setup();

		void Mapping(const trimesh::xform& source_form, std::vector<PtPair> &pairs);
		void SetTracer(ProjectionICPTracer* tracer);
	private:
		const float m_fx;
		const float m_fy;
		const float m_cx;
		const float m_cy;

		trimesh::TriMesh* m_source;
		trimesh::TriMesh* m_target;

		float m_maxdist;
		std::vector<trimesh::point> m_source_corner;
		ProjectionICPTracer* m_tracer;
	};
}

#endif
