#pragma once
#ifndef REMOVE_MANIFOLD_VERTEX
#define REMOVE_MANIFOLD_VERTEX

#include "cgaltype.h"
#include <set>

class Manifold_Vertex_Remover
{
	typedef std::set<Halfedge_iterator> halfedge_erease;
	typedef halfedge_erease::iterator halfedge_erease_iterator;

	typedef std::map<Vertex_handle, int> Vertex_map;
	typedef Vertex_map::iterator Vertex_map_it;
public:
	static void Remove(Polyhedron& p)
	{
		//halfedge_erease need_remove;
		//int id = 0;
		//Vertex_map test_manifold;

		//std::set<Vertex_handle> non_manifold;
		//for(Halfedge_iterator hit = p.edges_begin(); hit != p.edges_end(); ++hit)
		//{
		//	if(!(*hit).is_border())
		//		continue;
		//	if(!test_manifold.insert(std::make_pair((*hit).vertex(), id++)).second){
		//		non_manifold.insert((*hit).vertex());
		//	}
		//}

		//for(std::set<Vertex_handle>::iterator it = non_manifold.begin(); it != non_manifold.end(); ++it)
		//{
		//	Halfedge_handle h = (*it)->halfedge();
		//	Halfedge_handle hh = h;
		//	std::vector<Halfedge_handle> need_erease;
		//	do{
		//		if(hh->face() != Facet_handle())
		//			need_erease.push_back(hh);
		//		hh = hh->next()->opposite();
		//	}while(hh != h);

		//	for(unsigned i = 0; i < need_erease.size(); ++i)
		//		p.erase_facet(need_erease[i]);
		//}
			
	}
};
#endif // REMOVE_MANIFOLD_VERTEX