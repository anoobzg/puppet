#pragma once
#ifndef _POLYGON_ID_INIT
#define _POLYGON_ID_INIT

#include "cgaltype.h"
#include "CGAL\boost\graph\graph_traits_Polyhedron_3.h"

class Polygon_ID_init
{
	typedef Polyhedron::Facet_iterator Facet_iterator;
	typedef Polyhedron::Halfedge_around_facet_circulator HF_circulator;
public:
	static void init(Polyhedron& p)
	{
		typedef boost::graph_traits<Polyhedron>::halfedge_descriptor halfedge_descriptor;
		typedef boost::graph_traits<Polyhedron>::face_descriptor face_descriptor;
		typedef boost::graph_traits< Polyhedron >::face_iterator face_iterator;
		typedef boost::graph_traits< Polyhedron >::vertex_iterator vertex_iterator;
		typedef boost::graph_traits< Polyhedron >::halfedge_iterator halfedge_iterator;
		CGAL::Iterator_range<face_iterator> range = faces(p);

		unsigned id = 0;
		BOOST_FOREACH(Facet_handle h, range)
		{
			h->id() = id;
			++id;
		}

		CGAL::Iterator_range<vertex_iterator> v_range = vertices(p);

		id = 0;
		BOOST_FOREACH(Vertex_handle h, v_range)
		{
			h->id() = id;
			++id;
		}

		CGAL::Iterator_range<halfedge_iterator> h_range = halfedges(p);

		id = 0;
		BOOST_FOREACH(Halfedge_handle h, h_range)
		{
			h->id() = id;
			++id;
		}
	}
};

#endif // _CGAL_TYPES