#pragma once
#ifndef _VORONOI_AREA_COTANGENT
#define _VORONOI_AREA_COTANGENT
#include <CGAL\Polygon_mesh_processing\Weights.h>

template<class PolygonMesh>
struct Cotangent_vector_Meyer_impl
{
	typedef typename boost::graph_traits<PolygonMesh>::vertex_descriptor vertex_descriptor;

	template <class VertexPointMap>
	Vector operator()(vertex_descriptor v0, vertex_descriptor v1, vertex_descriptor v2, const VertexPointMap& ppmap)
	{
		Vector c = get(ppmap, v0) - get(ppmap, v2);
		Vector a = get(ppmap, v0) - get(ppmap, v1);
		Vector b = get(ppmap, v2) - get(ppmap, v1);

		double angle = acos( a*b / std::sqrt(a*a) / std::sqrt(b*b));
		assert(angle > 0.0 && angle < 3.1415926);
		double t = tan(CGAL_PI/2.0 - angle);
		return t * c;
	}
};

template<class PolygonMesh
	, class VertexPointMap = typename boost::property_map<PolygonMesh, CGAL::vertex_point_t>::type>
	class Cotangent_vector_Meyer
{
protected:
	typedef typename boost::graph_traits<PolygonMesh>::vertex_descriptor vertex_descriptor;
	typedef VertexPointMap Point_property_map;
	typedef typename boost::property_traits<Point_property_map>::value_type Point;

	PolygonMesh& pmesh_;
	Point_property_map ppmap;

public:

	Cotangent_vector_Meyer(PolygonMesh& pmesh_, VertexPointMap vpmap_)
		: pmesh_(pmesh_)
		, ppmap(vpmap_)
	{}

	PolygonMesh& pmesh()
	{
		return pmesh_;
	}

	Vector operator()(vertex_descriptor v0, vertex_descriptor v1, vertex_descriptor v2)
	{
		return Cotangent_vector_Meyer_impl<PolygonMesh>()(v0, v1, v2, ppmap);
	}
};

template<class PolygonMesh
	, class VertexPointMap = typename boost::property_map<PolygonMesh, CGAL::vertex_point_t>::type
	, class CotangentValue
	= Cotangent_vector_Meyer<PolygonMesh, VertexPointMap> >
	class Cotangent_vector : CotangentValue
{
	Cotangent_vector()
	{}

public:
	Cotangent_vector(PolygonMesh& pmesh_, VertexPointMap vpmap_)
		: CotangentValue(pmesh_, vpmap_)
	{}

	Cotangent_vector(PolygonMesh& pmesh_)
		: CotangentValue(pmesh_, get(CGAL::vertex_point, pmesh_))
	{}

	PolygonMesh& pmesh()
	{
		return CotangentValue::pmesh();
	}

	typedef typename boost::graph_traits<PolygonMesh>::halfedge_descriptor   halfedge_descriptor;
	typedef typename boost::graph_traits<PolygonMesh>::vertex_descriptor vertex_descriptor;

	typedef typename boost::property_map<PolygonMesh, CGAL::vertex_point_t>::type Point_property_map;
	typedef typename boost::property_traits<Point_property_map>::value_type Point;

	// Returns the cotangent weight of specified halfedge_descriptor
	// Edge orientation is trivial

	Vector half_calculate(halfedge_descriptor he)
	{
		vertex_descriptor v0 = target(he, pmesh());
		vertex_descriptor v1 = source(he, pmesh());
		// Only one triangle for border edges
		if (is_border_edge(he, pmesh()))
		{

			halfedge_descriptor he_cw = opposite(next(he, pmesh()), pmesh());
			vertex_descriptor v2 = source(he_cw, pmesh());
			if (is_border_edge(he_cw, pmesh()))
			{
				halfedge_descriptor he_ccw = prev(opposite(he, pmesh()), pmesh());
				v2 = source(he_ccw, pmesh());
			}
			return (CotangentValue::operator()(v0, v2, v1));
		}
		else
		{
			halfedge_descriptor he_cw = opposite(next(he, pmesh()), pmesh());
			vertex_descriptor v2 = source(he_cw, pmesh());
			halfedge_descriptor he_ccw = prev(opposite(he, pmesh()), pmesh());
			vertex_descriptor v3 = source(he_ccw, pmesh());

			return (CotangentValue::operator()(v0, v2, v1) + CotangentValue::operator()(v0, v3, v1));
		}
	}

	Vector operator()(vertex_descriptor he)
	{
		Vector v(0.0, 0.0, 0.0);
		unsigned i = 0;
		CGAL::Halfedge_around_target_circulator<Polyhedron>
			circ_vertex(he, pmesh()),
			done_vertex(circ_vertex);
		do
		{
			++i;
			v = v + half_calculate(*circ_vertex);

			//if (he->id() == 141)
			//{
			//	if ((*circ_vertex)->facet() == Facet_handle())
			//		std::cout << "boarder halfedge  " << i << std::endl;
			//}
		} while (++circ_vertex != done_vertex);

		if (he->id() == 141)
		{
			//std::cout << "141 halfedge_around: " << i << std::endl;
			//std::cout << " contangent" << v.x() / 4.0 << " " << v.y() / 4.0 << " " << v.z() / 4.0 << std::endl;
		}
		return v;
	}
};

template<class PolygonMesh
	, class VertexPointMap = typename boost::property_map<PolygonMesh, CGAL::vertex_point_t>::type
>
class Cotangent_weight_with_voronoi_area_caculator
{
	typedef PolygonMesh PM;
	typedef VertexPointMap VPMap;
	CGAL::internal::Voronoi_area<PM, VPMap> voronoi_functor;
	Cotangent_vector<PM, VPMap, Cotangent_vector_Meyer<PM, VPMap> > cotangent_functor;

	typedef typename boost::graph_traits<PM>::halfedge_descriptor   halfedge_descriptor;
	typedef typename boost::graph_traits<PM>::vertex_descriptor vertex_descriptor;
	std::map<vertex_descriptor, double> voronoi_area_map;
	std::map<vertex_descriptor, Vector> cotangent_map;
public:
	Cotangent_weight_with_voronoi_area_caculator(PM& pmesh_)
		: voronoi_functor(pmesh_, get(CGAL::vertex_point, pmesh_))
		, cotangent_functor(pmesh_, get(CGAL::vertex_point, pmesh_))
	{}

	Cotangent_weight_with_voronoi_area_caculator(PM& pmesh_, VPMap vpmap_)
		: voronoi_functor(pmesh_, vpmap_)
		, cotangent_functor(pmesh_, vpmap_)
	{}

	PM& pmesh()
	{
		return voronoi_functor.pmesh();
	}

	double voronoi_area(vertex_descriptor v_i)
	{
		std::map<vertex_descriptor, double>::iterator it = voronoi_area_map.find(v_i);
		if (it != voronoi_area_map.end())
			return (*it).second;

		double voronoi = voronoi_functor(v_i);
		std::pair<std::map<vertex_descriptor, double>::iterator, bool> pi =
			voronoi_area_map.insert(std::pair<vertex_descriptor, double>(v_i, voronoi));

		assert(pi.second);
		return voronoi;
	}

	Vector cotangent(vertex_descriptor v_i) {
		std::map<vertex_descriptor, Vector>::iterator it = cotangent_map.find(v_i);
		if (it != cotangent_map.end())
			return (*it).second;

		Vector cotangent = cotangent_functor(v_i);
		std::pair<std::map<vertex_descriptor, Vector>::iterator, bool> pi =
			cotangent_map.insert(std::pair<vertex_descriptor, Vector>(v_i, cotangent));

		assert(pi.second);
		return cotangent;
	}
};

#endif // _VORONOI_AREA_COTANGENT