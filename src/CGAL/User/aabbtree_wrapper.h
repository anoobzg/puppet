#pragma once
#ifndef AABB_TREE_WRAPPER
#define AABB_TREE_WRAPPER

#include "cgaltype.h"

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Facet_Handle_Primitive;
typedef CGAL::AABB_traits<Kernel, Facet_Handle_Primitive> AABB_Facet_Handle_traits;
typedef CGAL::AABB_tree<AABB_Facet_Handle_traits> Facet_Handle_AABB_Tree;

typedef boost::optional< Facet_Handle_AABB_Tree::Intersection_and_primitive_id<Segment>::Type > Facet_Segment_intersection;
typedef boost::optional< Facet_Handle_AABB_Tree::Intersection_and_primitive_id<Plane>::Type > Facet_Plane_intersection;
typedef boost::optional< Facet_Handle_AABB_Tree::Intersection_and_primitive_id<Ray>::Type> Facet_Ray_intersection;
#endif // AABB_TREE_WRAPPER