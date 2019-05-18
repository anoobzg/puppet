#pragma once
#ifndef _CGAL_TYPES
#define _CGAL_TYPES

#ifdef CGAL_TEST
#define DEBUG_TRACE
#endif

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Timer.h>
typedef CGAL::Simple_cartesian<double> Kernel; // fastest in experiments

typedef Kernel::FT FT;
typedef Kernel::Ray_3 Ray;
typedef Kernel::Line_3 Line;
typedef Kernel::Point_3 Point;
typedef Kernel::Plane_3 Plane;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Segment_3 Segment;
typedef Kernel::Triangle_3 Triangle;
typedef Kernel::Aff_transformation_3 Aff_transformation;

#include <CGAL/Polyhedron_3.h>

template <typename Refs, typename Tag, typename Point>
class Polyhedron_id_vertex :
	public CGAL::HalfedgeDS_vertex_base<Refs, Tag, Point>
{
private:
	typedef CGAL::HalfedgeDS_vertex_base<Refs, Tag, Point> Pdv_base;
	int _id;
public:
	Polyhedron_id_vertex() : Pdv_base(), _id(-1) {}
	Polyhedron_id_vertex(const Point& p) : Pdv_base(p), _id(-1) {}

	int& id() { return _id; }
	int  id() const { return _id; }
};

template <class Refs, class Tprev, class Tvertex, class Tface>
class Polyhedron_id_halfedge :
	public CGAL::HalfedgeDS_halfedge_base<Refs, Tprev, Tvertex, Tface>
{
	typedef CGAL::HalfedgeDS_halfedge_base<Refs, Tprev, Tvertex, Tface> Pdv_base;
	int _id;
public:
	Polyhedron_id_halfedge() : Pdv_base(), _id(-1) {}
	int& id() { return _id; }
	int  id() const { return _id; }
};

template <class Refs, class T_, class Pln_>
class Polyhedron_id_face :
	public CGAL::HalfedgeDS_face_base<Refs, T_, Pln_>
{
private:
	int _id;
	unsigned _patch;
public:
	Polyhedron_id_face() :_id(-1),_patch(0) {}

	int& id() { return _id; }
	int  id() const { return _id; }
	unsigned& patch() { return _patch; }
	unsigned patch() const { return _patch; }
};


class Polyhedron_id_items : public CGAL::Polyhedron_items_3 {
public:
	// wrap vertex
	template<class Refs, class Traits> struct Vertex_wrapper
	{
		typedef typename Traits::Point_3 Point;
		typedef Polyhedron_id_vertex<Refs,
			CGAL::Tag_true,
			Point> Vertex;
	};

	// wrap face
	template<class Refs, class Traits> struct Face_wrapper
	{
		typedef Polyhedron_id_face<Refs,
			CGAL::Tag_true,
			typename Traits::Plane_3> Face;
	};

	// wrap halfedge
	template<class Refs, class Traits> struct Halfedge_wrapper
	{
		typedef Polyhedron_id_halfedge<Refs,
			CGAL::Tag_true,
			CGAL::Tag_true,
			CGAL::Tag_true> Halfedge;
	};
};

typedef CGAL::Polyhedron_3<Kernel, Polyhedron_id_items> Polyhedron;
typedef Polyhedron::Halfedge_handle    Halfedge_handle;
typedef Polyhedron::Facet_handle       Facet_handle;
typedef Polyhedron::Vertex_handle      Vertex_handle;
typedef Polyhedron::Facet_iterator    Facet_iterator;
typedef Polyhedron::Halfedge_iterator Halfedge_iterator;
typedef Polyhedron::Vertex_iterator   Vertex_iterator;

#define PROFILE_TIMER 	CGAL::Timer timer; timer.start();
#define PROFILE_TIME(x) std::cout << x << timer.time() << std::endl;  timer.reset();
#endif // _CGAL_TYPES