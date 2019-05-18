#ifndef _CGAL_HOLE_FILLER
#define _CGAL_HOLE_FILLER
#include "cgaltype.h"
#include <CGAL/boost/graph/halfedge_graph_traits_Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#define HOLE_SIZE_DIFF 35

typedef enum Hole_Repair_Level
{
	e_triangulate,
	e_refine,
	e_fair,
} hole_repair_level;

class CGALHoleFiller
{
public:
	static bool Fill_One_Hole(Polyhedron& p, Halfedge_handle h, hole_repair_level level, std::vector<Facet_handle>& patch_facets, std::vector<Vertex_handle>& patch_vertices)
	{
		bool success = true;
		size_t input_size = patch_facets.size();
		assert(h->is_border());
		if(h->is_border())
		{

			if(level == e_fair)
			{
				//CGAL::Timer timer; timer.start();

				success = CGAL::cpp11::get<0>(
				CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(p, h, std::back_inserter(patch_facets), std::back_inserter(patch_vertices),
				CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, p)).geom_traits(Kernel())));

				size_t output_size = patch_facets.size();
				if(output_size <= input_size)
				{
					success = CGAL::cpp11::get<0>(
					CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(p, h, std::back_inserter(patch_facets), std::back_inserter(patch_vertices),
					CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, p)).geom_traits(Kernel()).use_weight_imcomplete(true)));
				}

				//std::cout<<"triangulate_refine_and_fair time cost :"<<timer.time()<<std::endl;
				success = patch_facets.size() > input_size;
			}else if(level == e_refine){
				CGAL::Polygon_mesh_processing::triangulate_and_refine_hole(p, h, std::back_inserter(patch_facets), std::back_inserter(patch_vertices),
					CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, p)).geom_traits(Kernel()));
				if(patch_facets.size() <= input_size)
				{
					CGAL::Polygon_mesh_processing::triangulate_and_refine_hole(p, h, std::back_inserter(patch_facets),std::back_inserter(patch_vertices),
						CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, p)).geom_traits(Kernel()).use_weight_imcomplete(true));
				}
				success = patch_facets.size() > input_size;
			}else if(level == e_triangulate){
				CGAL::Polygon_mesh_processing::triangulate_hole(p, h, std::back_inserter(patch_facets),
				CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, p)).geom_traits(Kernel()));
				if(patch_facets.size() <= input_size)
				{
					CGAL::Polygon_mesh_processing::triangulate_hole(p, h, std::back_inserter(patch_facets),
						CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, p)).geom_traits(Kernel()).use_weight_imcomplete(true));
				}
				success = patch_facets.size() > input_size;
			}
			//std::cout<<"Fairing : " << (success ? "succeeded" : "failed") << std::endl;
		}
		return success;
	}

	static void Fill_Holes(Polyhedron& p)
	{
		return;
		unsigned int nb_holes = 0;
		BOOST_FOREACH(Halfedge_handle h, halfedges(p))
		{
			if(h->is_border())
			{
				std::vector<Facet_handle> patch_facets;
				std::vector<Vertex_handle> patch_vertices;
				bool success = CGAL::cpp11::get<0>(
					CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(p, h, std::back_inserter(patch_facets), std::back_inserter(patch_vertices),
					CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, p)).geom_traits(Kernel())));

				std::cout<<"Fairing : " << (success ? "succeeded" : "failed") << std::endl;
				++nb_holes;
			}
		}
	}
};
#endif // _CGAL_HOLE_FILLER