#pragma once
#ifndef MESH_CHECKER_H
#define MESH_CHECKER_H

#include <set>
#include <vector>
#include <map>
template<class TPoly>
class Mesh_Checker
{
	typedef typename TPoly::Traits::Vector_3 Vector;
	typedef typename TPoly::Traits::Point_3 Point;
	typedef typename TPoly::Facet_handle Facet_handle;
	typedef typename TPoly::Halfedge_handle Halfedge_handle;
	typedef typename TPoly::Vertex_handle Vertex_handle;
	typedef typename TPoly::Facet_iterator Facet_iterator;
	typedef typename TPoly::Halfedge_iterator Halfedge_iterator;

public:
	static void Check(TPoly& poly)
	{
		RemoveInvalidFacet(poly);
		RemoveInvalidHalfedge(poly);
		RemoveNonmanifoldVertex(poly);
	}

private:
	static void RemoveInvalidFacet(TPoly& poly)
	{
		//std::vector<Halfedge_handle> facets;

		//std::set<Halfedge_handle> face_halfedge;
		//for (Facet_iterator fi = poly.facets_begin(); fi != poly.facets_end(); ++fi)
		//{
		//	if (!face_halfedge.insert(fi->halfedge()).second)
		//		facets.push_back(fi->halfedge());
		//}

		//for (std::vector<Halfedge_handle>::iterator it = facets.begin();
		//	it != facets.end(); ++it)
		//{
		//	poly.erase_facet(*it);
		//}
	}

	static void RemoveInvalidHalfedge(TPoly& poly)
	{
		//std::set<Halfedge_iterator> need_erease;
		//typedef std::set<Halfedge_iterator>::iterator iter;
		for (Halfedge_iterator hit = poly.halfedges_begin(); hit != poly.halfedges_end(); ++hit)
		{
			if (!(*hit).is_border())
				continue;

			if ((*hit).next() == Halfedge_handle() )
			{
				std::cout<<"************************error halfedge**********************"<<std::endl;
				//need_erease.insert(hit);
				continue;
			}
			if ((*hit).opposite() == (*hit).next())
			{
				std::cout<<"************************error halfedge**********************"<<std::endl;
				//need_erease.insert(hit);
				continue;
			}
			if ((*hit).prev() == Halfedge_handle())
			{
				std::cout<<"************************error halfedge**********************"<<std::endl;
				//need_erease.insert(hit);
				continue;
			}
			if ((*hit).prev()->next() != &(*hit))
			{
				std::cout<<"************************error halfedge**********************"<<std::endl;
				//need_erease.insert(hit);
				continue;
			}
			if ((*hit).next()->prev() != &(*hit))
			{
				std::cout<<"************************error halfedge**********************"<<std::endl;
				//need_erease.insert(hit);
				continue;
			}
		}
		//for (iter it = need_erease.begin(); it != need_erease.end(); ++it)
		//{
		//	poly.hds().edges_erase(*it);
		//}
	}

	static void RemoveNonmanifoldVertex(TPoly& poly)
	{
		//typedef std::map<Vertex_handle, int> Vertex_map;
		//typedef Vertex_map::iterator Vertex_map_iter;

		//Vertex_map tmp_vertex;
		//std::vector<Vertex_handle> non_manifold;
		//std::vector<Halfedge_handle> invalid_edge;
		//for (Halfedge_iterator it = poly.halfedges_begin(); it != poly.halfedges_end(); ++it)
		//{
		//	if ((*it).next() == (*it).next()->prev())
		//		invalid_edge.push_back(&*it);
		//	if (!(*it).is_border())
		//		continue;
		//	if (!tmp_vertex.insert(std::pair<Vertex_handle, int>((*it).vertex(), 0)).second)
		//		non_manifold.push_back((*it).vertex());
		//}

		//for (unsigned i = 0; i < non_manifold.size(); ++i)
		//{
		//	std::vector<Halfedge_handle> facets;
		//	Halfedge_handle h = non_manifold[i]->halfedge();
		//	do {
		//		if(h->face() != Facet_handle())
		//			facets.push_back(h);
		//		h = h->next()->opposite();
		//	} while (h != non_manifold[i]->halfedge());

		//	for (unsigned i = 0; i < facets.size(); ++i)
		//		poly.erase_facet(facets[i]);
		//}
	}
};
#endif // MESH_CHECKER_H