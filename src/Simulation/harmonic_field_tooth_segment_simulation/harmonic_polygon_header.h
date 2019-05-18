#pragma once

#include "kernel.h"
#include <CGAL/Polyhedron_3.h>
#include <CGAL/HalfedgeDS_vertex_max_base_with_id.h>
#include <CGAL/HalfedgeDS_halfedge_max_base_with_id.h>
#include <CGAL/HalfedgeDS_face_max_base_with_id.h>

template < class Refs, class P, class N, class ID>
class HalfedgeDS_vertex_harmonic : public CGAL::HalfedgeDS_vertex_max_base_with_id<Refs, P, ID>
{
public:
	typedef CGAL::HalfedgeDS_vertex_max_base_with_id< Refs, P, ID> Base;
	typedef P Point;
	typedef N Normal;
public:
	HalfedgeDS_vertex_harmonic() {}
	HalfedgeDS_vertex_harmonic(Point const& p) : Base(p){}

	Normal m_normal;
	bool is_concave;
};


template < class Refs, class ID>
class HalfedgeDS_halfedge_harmonic : public CGAL::HalfedgeDS_halfedge_max_base_with_id<Refs, ID>
{
public:
	typedef CGAL::HalfedgeDS_halfedge_max_base_with_id< Refs, ID> Base;
	typedef typename Base::Base_base Base_base;

	double m_weight;
	double m_p;
	bool m_set;
public:
	HalfedgeDS_halfedge_harmonic():m_weight(0.0), m_set(false){}

	double& Weight() { return m_weight; }
	const double& Weight() const { return m_weight; }
	bool& Set() { return m_set; }
	const bool& Set() const { return m_set; }
};

template < class Refs, class Pln, class ID>
class HalfedgeDS_face_harmonic : public CGAL::HalfedgeDS_face_max_base_with_id<Refs, Pln, ID>
{
public:

	typedef CGAL::HalfedgeDS_face_max_base_with_id< Refs, Pln, ID> Base;
	unsigned unseed_order;

public:
	HalfedgeDS_face_harmonic(){}
	HalfedgeDS_face_harmonic(Pln const& p) : Base(p){}
};

class Polyhedron_items_harmonic {
public:
	template < class Refs, class Traits>
	struct Vertex_wrapper {
		typedef typename Traits::Point_3 Point;
		typedef typename Traits::Vector_3 Normal;
		typedef HalfedgeDS_vertex_harmonic< Refs, Point, Normal, std::size_t> Vertex;
	};
	template < class Refs, class Traits>
	struct Halfedge_wrapper {
		typedef HalfedgeDS_halfedge_harmonic<Refs, std::size_t> Halfedge;
	};
	template < class Refs, class Traits>
	struct Face_wrapper {
		typedef HalfedgeDS_face_harmonic< Refs, CGAL::Tag_false, std::size_t>  Face;
	};
};

typedef typename CGAL::Polyhedron_3 < FKernel, Polyhedron_items_harmonic > Polyhedron;

typedef typename Polyhedron::Vertex_iterator Vertex_iterator;
typedef typename Polyhedron::Vertex_handle Vertex_handle;
typedef typename Polyhedron::Halfedge_handle Halfedge_handle;
typedef typename Polyhedron::Halfedge_iterator Halfedge_iterator;
typedef typename Polyhedron::Point Point;
typedef typename FKernel::Vector_3 Vector;
typedef typename Polyhedron::Facet_iterator Facet_iterator;