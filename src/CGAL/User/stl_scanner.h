#pragma once
#ifndef SURF_POLYHEDRON_SCAN_STL
#define SURF_POLYHEDRON_SCAN_STL
#include <CGAL/Modifier_base.h>
#include <iostream>
#include <cstddef>
#include "polyhedron_builder.h"

template < class HDS>
class STL_Scanner : public CGAL::Modifier_base<HDS> 
{
	typedef typename HDS::Vertex            Vertex;
	typedef typename HDS::Halfedge          Halfedge;
	typedef typename HDS::Face              Face;
	typedef typename HDS::Vertex_handle     Vertex_handle;
	typedef typename HDS::Halfedge_handle   Halfedge_handle;
	typedef typename HDS::Face_handle       Face_handle;
	typedef typename HDS::Face_handle       Facet_handle;
	typedef typename Vertex::Base           VBase;
	typedef typename Halfedge::Base         HBase;
	typedef typename Vertex::Point          Point;
	typedef typename HDS::size_type         size_type;

protected:
	std::string m_name;
	std::istream&    m_in;
public:

	typedef HDS Halfedge_data_structure;

	STL_Scanner(std::string name, std::istream& in)
		: m_in(in),m_name(name){}

	// Activation
	void operator()(HDS& hds);
};

template < class HDS >
void STL_Scanner<HDS>:: operator()(HDS& target)
{
	CGAL::Polyhedron_Builder<HDS> B(target, true);

	FILE *fp;
	fp = fopen(m_name.c_str(), "rb");
	if (fp == NULL)
	{
		return;
	}

	int f;
	fseek(fp, 80, SEEK_SET);
	fread(&f, sizeof(int), 1, fp);

	typedef typename HDS::Traits     Traits;

	class CmpVec
	{
	public:
		CmpVec(double _eps = DBL_MIN) : eps_(_eps) {}

		bool operator()(const Point& _v0, const Point& _v1) const
		{
			if (fabs(_v0.x() - _v1.x()) <= eps_)
			{
				if (fabs(_v0.y() - _v1.y()) <= eps_)
				{
					return (_v0.z() < _v1.z() - eps_);
				}
				else return (_v0.y() < _v1.y() - eps_);
			}
			else return (_v0.x() < _v1.x() - eps_);
		}
	private:
		double eps_;
	};
	// For each triangle read the normal, the three coords and a short set to zero
	typedef std::map<Point, unsigned, CmpVec> uniformPoint;
	typedef uniformPoint::iterator PointIter;
	typedef std::pair<PointIter, bool> unifomrRt;
	typedef std::vector<unsigned> Triangles;
	typedef Triangles::iterator triIter;
	uniformPoint _points;
	PointIter iter;
	Triangles _triangle;
	_triangle.resize(3 * f);
	B.begin_surface(3*f, f, 4 * f);

	// read in all vertices
	unsigned tmp = 0;
	for (int i = 0; i<f; ++i)
	{
		unsigned short attr;
		float fnorm[3];
		float ftri[9];
		fread(&fnorm[0], 3*sizeof(float), 1, fp);
		fread(&ftri[0], 9*sizeof(float), 1, fp);
		fread(&attr, sizeof(unsigned short), 1, fp);
		Point norm(fnorm[0], fnorm[1], fnorm[2]);
		Point tri[3];
		tri[0] = Point(ftri[0], ftri[1], ftri[2]);
		tri[1] = Point(ftri[3], ftri[4], ftri[5]);
		tri[2] = Point(ftri[6], ftri[7], ftri[8]);

		for (int k = 0; k<3; ++k)
		{
			if ((iter = _points.find(tri[k])) == _points.end())
			{
				unifomrRt it = _points.insert(std::pair<Point, unsigned>(tri[k], tmp++));
				Vertex_handle h = B.add_vertex(tri[k]);
				h->id() = tmp - 1;
				_triangle[3 * i + k] = tmp - 1;
			}
			else
				_triangle[3 * i + k] = (*iter).second;
		}
	}

	unsigned i = 0;
	if (!m_in || B.error()) {
		B.rollback();
		m_in.clear(std::ios::badbit);
		fclose(fp);
		return;
	}

	// read in all facets
	for (int i = 0; i < f; i++) {
		B.save(_triangle[3*i], _triangle[3*i+1], _triangle[3*i+2]);
		Facet_handle fh = B.begin_facet();
		fh->id() = i;
		
		for (std::size_t j = 0; j < 3; j++) {
			std::size_t index = _triangle[3 * i + j];

			B.add_vertex_to_facet(index);
		}
		//TO DO : Insert read color
		B.end_facet();

		B.store();
	}

	if (!m_in || B.error()) {
		B.rollback();
		m_in.clear(std::ios::badbit);
		fclose(fp);
		return;
	}
	if (B.check_unconnected_vertices()) {
		if (!B.remove_unconnected_vertices()) {
			std::cout << " " << std::endl;
			std::cout << "Polyhedron_scan_STL<Traits>::" << std::endl;
			std::cout << "operator()(): input error: cannot "
				"successfully remove isolated vertices."
				<< std::endl;

			B.rollback();
			m_in.clear(std::ios::badbit);
			fclose(fp);
			return;
		}
	}
	B.end_surface();

	_triangle.clear();
	fclose(fp);
}

#endif // SURF_POLYHEDRON_SCAN_STL