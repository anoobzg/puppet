#ifndef ID_ITEM_POLYHEDRON
#define ID_ITEM_POLYHEDRON

#include "kernel.h"
#include <CGAL/Polyhedron_3.h>
#include <CGAL\Polyhedron_items_with_id_3.h>

typedef CGAL::Polyhedron_3 < FKernel, CGAL::Polyhedron_items_with_id_3 > ID_item_FPolyhedron;
typedef CGAL::Polyhedron_3 < DKernel, CGAL::Polyhedron_items_with_id_3 > ID_item_DPolyhedron;

#endif  // ID_ITEM_POLYHEDRON
