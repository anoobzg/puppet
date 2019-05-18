#pragma once
#ifndef _DIHEDRAL
#define _DIHEDRAL
#include <CGAL\Polygon_mesh_processing\Weights.h>

template<class PolygonMesh
	, class VertexPointMap = typename boost::property_map<PolygonMesh, CGAL::vertex_point_t>::type >
class Dihedral_calculator_Impl
{
protected:
	typedef typename boost::graph_traits<PolygonMesh>::halfedge_descriptor halfedge_descriptor;
	typedef typename boost::graph_traits<PolygonMesh>::vertex_descriptor vertex_descriptor;
	typedef VertexPointMap Point_property_map;
	typedef typename boost::property_traits<Point_property_map>::value_type Point;
	typedef typename CGAL::Kernel_traits<Point>::Kernel::Vector_3 Vector;

	PolygonMesh& pmesh_;
	Point_property_map ppmap;
public:
	Dihedral_calculator_Impl(PolygonMesh& pmesh, VertexPointMap& vpm)
		:pmesh_(pmesh),ppmap(vpm)
	{}

	PolygonMesh& pmesh()
	{
		return pmesh_;
	}

	double operator()(halfedge_descriptor h)
	{
		if(CGAL::is_border_edge(h, pmesh()))
			return 0.0;

		vertex_descriptor vp0 = source(h, pmesh());
		vertex_descriptor vp1 = target(h, pmesh());
		vertex_descriptor vp2 = target(next(h, pmesh()), pmesh());
		vertex_descriptor vp3 = target(next(opposite(h, pmesh()), pmesh()),pmesh());
		Point v0 = get(ppmap, vp0);
		Point v1 = get(ppmap, vp1);
		Point v2 = get(ppmap, vp2);
		Point v3 = get(ppmap, vp3);

		Vector e01 = v1 - v0;
		Vector e02 = v2 - v0;
		Vector e03 = v3 - v0;
		Vector n1 = CGAL::cross_product(e01, e02);
		n1 = n1 / std::sqrt(n1*n1);
		Vector n2 = CGAL::cross_product(e03, e01);
		n2 = n2 / std::sqrt(n2*n2);
		Vector cross_n = CGAL::cross_product(n1, n2);
		double dot = e01*cross_n;
		double dihedral = acos(n1*n2);
		if (n1*n2 > 0.99999999)
			return CGAL_PI;

		if (dot > 0.0)
			dihedral = CGAL_PI + dihedral;
		else if (dot < 0.0)
			dihedral = CGAL_PI - dihedral;
		else
		{
			assert(false);
			std::cout << "wrong!" << std::endl;
		}

		return dihedral;
	}
};

template<class PolygonMesh
	, class VertexPointMap = typename boost::property_map<PolygonMesh, CGAL::vertex_point_t>::type >
	class Difference_caculator
{
protected:
	typedef typename boost::graph_traits<PolygonMesh>::halfedge_descriptor halfedge_descriptor;
	typedef typename boost::graph_traits<PolygonMesh>::vertex_descriptor vertex_descriptor;
	typedef VertexPointMap Point_property_map;
	typedef typename boost::property_traits<Point_property_map>::value_type Point;
	typedef typename CGAL::Kernel_traits<Point>::Kernel::Vector_3 Vector;

	PolygonMesh& pmesh_;
	Point_property_map ppmap;
public:
	Difference_caculator(PolygonMesh& pmesh, VertexPointMap& vpm)
		:pmesh_(pmesh), ppmap(vpm)
	{}

	PolygonMesh& pmesh()
	{
		return pmesh_;
	}

	double operator()(halfedge_descriptor h)
	{
		if (CGAL::is_border_edge(h, pmesh()))
			return 0.0;

		vertex_descriptor vp0 = source(h, pmesh());
		vertex_descriptor vp1 = target(h, pmesh());
		vertex_descriptor vp2 = target(next(h, pmesh()), pmesh());
		vertex_descriptor vp3 = target(next(opposite(h, pmesh()), pmesh()), pmesh());
		assert(vp0->id() != vp1->id());
		assert(vp0->id() != vp2->id());
		assert(vp0->id() != vp3->id());
		assert(vp1->id() != vp2->id());
		assert(vp1->id() != vp3->id());
		assert(vp2->id() != vp3->id());
		//if ((vp0->id() == vp1->id()) || (vp0->id() == vp2->id()) || (vp0->id() == vp3->id())
		//	|| (vp1->id() == vp2->id()) || (vp1->id() == vp3->id()) || (vp2->id() == vp3->id()))
		//	std::cout << "invalid topology." << std::endl;
		Point v0 = get(ppmap, vp0);
		Point v1 = get(ppmap, vp1);
		Point v2 = get(ppmap, vp2);
		Point v3 = get(ppmap, vp3);

		Vector e01 = v1 - v0;
		Vector e02 = v2 - v0;
		Vector e03 = v3 - v0;
		Vector n1 = CGAL::cross_product(e01, e02);
		if (n1.x() == 0.0 && n1.y() == 0.0 && n1.z() == 0.0)
			return 0.0;
		n1 = n1 / std::sqrt(n1*n1);
		Vector n2 = CGAL::cross_product(e03, e01);
		if (n2.x() == 0.0 && n2.y() == 0.0 && n2.z() == 0.0)
			return 0.0;
		n2 = n2 / std::sqrt(n2*n2);
		Vector cross_n = CGAL::cross_product(n1, n2);
		double dot = e01*cross_n;
		double dihedral = acos(n1*n2);
		if (n1*n2 > 0.9999999999)
			return 0.0;

		double dif = 0.0;
		double sigma = 0.0;
		if (dot > 0.0)
			sigma = 0.2;
		else if (dot < 0.0)
			sigma = 1.0;
		Vector n1n2 = n2 - n1;
		dif = sigma * std::sqrt(n1n2*n1n2) / 2.0;
		return dif;
	}
};

template<class PolygonMesh
	, class VertexPointMap = typename boost::property_map<PolygonMesh, CGAL::vertex_point_t>::type
>
class Dihedral_caculator
{
	typedef PolygonMesh PM;
	typedef VertexPointMap VPMap;
	Dihedral_calculator_Impl<PM, VPMap> dihedral_functor;
	Difference_caculator<PM, VPMap> difference_functor;

	typedef typename boost::graph_traits<PM>::halfedge_descriptor   halfedge_descriptor;
	typedef typename boost::graph_traits<PM>::vertex_descriptor vertex_descriptor;
	std::map<halfedge_descriptor, double> dihedral_map;
	std::map<halfedge_descriptor, double> difference_map;
public:
	Dihedral_caculator(PM& pmesh_)
		:dihedral_functor(pmesh_, get(CGAL::vertex_point, pmesh_))
		, difference_functor(pmesh_, get(CGAL::vertex_point, pmesh_))
	{}

	Dihedral_caculator(PM& pmesh_, VPMap vpmap_)
		:dihedral_functor(pmesh_, vpmap_)
		,difference_functor(pmesh_, vpmap_)
	{}

	PM& pmesh()
	{
		return dihedral_functor.pmesh();
	}

	double dihedral(halfedge_descriptor h_i)
	{
		std::map<halfedge_descriptor, double>::iterator it = dihedral_map.find(h_i);
		if (it != dihedral_map.end())
			return (*it).second;

		double dihedral = dihedral_functor(h_i);
		std::pair<std::map<halfedge_descriptor, double>::iterator, bool> pi =
			dihedral_map.insert(std::pair<halfedge_descriptor, double>(h_i, dihedral));

		assert(pi.second);
		return dihedral;
	}

	double difference(halfedge_descriptor h_i)
	{
		std::map<halfedge_descriptor, double>::iterator it = difference_map.find(h_i);
		if (it != difference_map.end())
			return (*it).second;

		double dif = difference_functor(h_i);
		assert(!isnan(dif));
		if (isnan(dif))
			std::cout << "nan value" << std::endl;
		std::pair<std::map<halfedge_descriptor, double>::iterator, bool> pi =
			difference_map.insert(std::pair<halfedge_descriptor, double>(h_i, dif));

		assert(pi.second);
		return dif;
	}
};

#endif // _DIHEDRAL