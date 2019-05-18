#pragma once
#ifndef MESH_CHECKER_H
#define MESH_CHECKER_H

#include "cgaltype.h"
#include <string>
#include <CGAL\boost\graph\graph_traits_Polyhedron_3.h>
#include <set>

class Mesh_Checker
{
	typedef boost::graph_traits< Polyhedron >::face_iterator face_iterator;
public:
	Mesh_Checker(Polyhedron& P):_p(P) {}
	~Mesh_Checker() {}

	bool check() {
		return check_degenerate_triangle() && error_message.empty();
	}
	const std::string& error() { return error_message; }
private:
	bool check_degenerate_triangle()
	{
		CGAL::Iterator_range<face_iterator> range = CGAL::faces(_p);
		std::cout.precision(10);
		BOOST_FOREACH(Facet_handle h, range)
		{
			assert(h->is_triangle());
			Halfedge_handle hh = h->halfedge();
			Halfedge_handle h1 = hh;
			Halfedge_handle h2 = h1->next();
			Halfedge_handle h3 = h2->next();
			assert(h1 != h2 && h1 != h3 && h2 != h3);
			Vector e1 = h3->vertex()->point() - h1->vertex()->point();
			Vector e2 = h2->vertex()->point() - h1->vertex()->point();
			Vector ec = CGAL::cross_product(e1, e2);
			if (ec.x() == 0.0 && ec.y() == 0.0 && ec.z() == 0.0)
			{
				std::cout << "degenerate triangle!  " << h->id() << std::endl;
				Point p1 = h1->vertex()->point();
				Point p2 = h2->vertex()->point();
				Point p3 = h3->vertex()->point();
				std::cout << p1.x() << "  " << p1.y() << "  " << p1.z() << std::endl;
				std::cout << p2.x() << "  " << p2.y() << "  " << p2.z() << std::endl;
				std::cout << p3.x() << "  " << p3.y() << "  " << p3.z() << std::endl;
				error_message = "degenerate triangle!";
			}
		}
		return error_message.empty();
	}

protected:
	Polyhedron& _p;
	std::string error_message;
};
#endif // MESH_CHECKER_H