#ifndef _HOLE_BUILDER
#define _HOLE_BUILDER

#include "cgaltype.h"
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

typedef struct CGAL_HOLE_INFO
{
	unsigned patch;
	unsigned size;
	Halfedge_handle handle;
	unsigned id;
} cgal_hole_info;
typedef std::vector<cgal_hole_info> cgal_hole_infos;
typedef std::vector<cgal_hole_info>::iterator hole_iterator;

class HoleBuilder
{
	typedef std::map<Vertex_handle, int> Vertex_map;
	typedef Vertex_map::iterator Vertex_map_it;
public:
	static void SearchHole(std::vector<Halfedge_handle>& holes, Polyhedron& p)
	{
		typedef std::set<Halfedge_handle> halfedge_set;
		halfedge_set h_set;
		BOOST_FOREACH(Halfedge_handle h, halfedges(p))
		{
			if(h->is_border() && h_set.find(h) == h_set.end())
			{
				holes.push_back(h);
				Halfedge_handle hh(h);
				do {
					h_set.insert(hh);
				}while(( hh = hh->next()) != h);
			}
		}
	}

	static void Build(cgal_hole_infos& holes, Polyhedron& p)
	{
		std::vector<Halfedge_handle> tmp_holes;
		//SearchHole(tmp_holes, p);

		//std::set<Vertex_handle> non_manifold;
		//for(unsigned i = 0; i < tmp_holes.size(); ++i)
		//{
		//	int id = 0;
		//	Vertex_map test_manifold;
		//	Halfedge_handle circ(tmp_holes[i]), done(circ);
		//	do{
		//		if(!test_manifold.insert(std::make_pair(circ->vertex(), id++)).second){
		//			non_manifold.insert(circ->vertex());
		//		}
		//		circ = circ->next();
		//	}while(circ!=done);
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
		//tmp_holes.clear();

		SearchHole(tmp_holes, p);
		for(unsigned i = 0; i < tmp_holes.size(); ++i)
		{
			cgal_hole_info hinfo;
			hinfo.handle = tmp_holes[i];
			hinfo.patch = hinfo.handle->opposite()->face()->patch();
			static unsigned id = 0;
			hinfo.id = ++id;
			unsigned size = 0;
			Halfedge_handle hh = tmp_holes[i];
			do {
				++size;
			}while(( hh = hh->next()) != hinfo.handle);
			hinfo.size = size;
			holes.push_back(hinfo);
		}
	}

	static void Boarder(Polyhedron& p, Halfedge_handle h, std::vector<Vertex_handle>& boarders)
	{
		typedef CGAL::Halfedge_around_face_circulator<Polyhedron> HF_circulator;
		Halfedge_handle h_done(h);
		do {
			boarders.push_back(CGAL::target(h, p));
		}while(( h = h->next()) != h_done);
	}
};
#endif // _HOLD_BUILDER