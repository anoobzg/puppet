#ifndef POLYHEDRON_2_ARRAY
#define POLYHEDRON_2_ARRAY
#include <osg\Array>
#include <osg\PrimitiveSet>
#include "normal_calculator.h"

template <class TPoly>
class Polyhedron_2_Array
{
protected:
	typedef typename TPoly::Traits::Vector_3 Vector;
	typedef typename TPoly::Traits::Point_3 Point;
	typedef typename TPoly::Vertex_iterator    Vertex_iterator;
	typedef typename TPoly::Facet_iterator     Facet_iterator;
	typedef typename TPoly::Halfedge_handle	   Halfedge_handle;
	typedef typename TPoly::Face_handle		   Face_handle;
	typedef typename TPoly::Vertex_handle	   Vertex_handle;
	typedef typename TPoly::Halfedge_around_facet_circulator HF_circulator;
	typedef typename TPoly::Halfedge_around_vertex_circulator HV_circulator;
public:
	static osg::Array* CreateCoordArray(TPoly& poly)
	{
		int num = poly.size_of_vertices();
		osg::Vec3Array* coord = new osg::Vec3Array();
		coord->reserve(num);
		for (Vertex_iterator vi = poly.vertices_begin(); vi != poly.vertices_end(); ++vi)
		{
			Point p = (*vi).point();
			coord->push_back(osg::Vec3(p.x(), p.y(), p.z()));
		}
		return coord;
	}

	static osg::Array* CreateSoupCoordArray(TPoly& poly)
	{
		int num = poly.size_of_facets() * 3;
		osg::Vec3Array* coord = new osg::Vec3Array();
		coord->reserve(num);

		for (Facet_iterator fi = poly.facets_begin(); fi != poly.facets_end(); ++fi)
		{
			assert(fi->is_triangle());
			HF_circulator hf = fi->facet_begin();

			do {
				Point p = hf->vertex()->point();
				coord->push_back(osg::Vec3(p.x(), p.y(), p.z()));
			} while (++hf != fi->facet_begin());
		}
		return coord;
	}

	static osg::Array* CreateNormalArray(TPoly& poly)
	{
		int num = poly.size_of_vertices();
		osg::Vec3Array* normal = new osg::Vec3Array();
		normal->reserve(num);
		for (Vertex_iterator vi = poly.vertices_begin(); vi != poly.vertices_end(); ++vi)
		{
			HV_circulator hv = vi->vertex_begin();
			Vector n = Normal_Caculator<TPoly>::CalculateNormal(hv);
			normal->push_back(osg::Vec3(n.x(), n.y(), n.z()));
		}
		return normal;
	}

	static osg::Array* CreateSoupNormalArray(TPoly& poly)
	{
		int num = poly.size_of_facets() * 3;
		osg::Vec3Array* normal = new osg::Vec3Array();
		normal->reserve(num);

		for (Facet_iterator fi = poly.facets_begin(); fi != poly.facets_end(); ++fi)
		{
			assert(fi->is_triangle());
			HF_circulator hf = fi->facet_begin();

			do {
				Vector n = Normal_Caculator<TPoly>::CalculateNormal(hf);
				normal->push_back(osg::Vec3(n.x(), n.y(), n.z()));
			} while (++hf != fi->facet_begin());
		}
		return normal;
	}

	static osg::PrimitiveSet* CreatePrimitiveSet(TPoly& poly)
	{
		static int size = 1 << 16;
		int num = poly.size_of_facets() * 3;
		osg::DrawElementsUInt* primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES);

		primitive->reserve(num);
		for (Facet_iterator fi = poly.facets_begin(); fi != poly.facets_end(); ++fi)
		{
			assert(fi->is_triangle());
			HF_circulator hf = fi->facet_begin();

			do {
				int id = hf->vertex()->id();
				primitive->push_back(id);
			} while (++hf != fi->facet_begin());
		}

		return primitive;
	}

	static osg::PrimitiveSet* CreateSoupPrimitiveSet(TPoly& poly)
	{
		int num = poly.size_of_facets() * 3;
		return new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, num);
	}
};
#endif // POLYHEDRON_2_ARRAY