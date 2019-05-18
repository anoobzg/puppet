#ifndef IMPORTER_MEMORY
#define IMPORTER_MEMORY

#include <CGAL\Modifier_base.h>
#include "polyhedron_builder.h"

template <class HDS>
class Memory_Modifier : public CGAL::Modifier_base<HDS>
{
public:
	Memory_Modifier(unsigned ver_num, float* vertices, unsigned fac_num, unsigned* index)
	:_ver_num(ver_num), _vertices(vertices), _fac_num(fac_num), _index(index) {}
	void operator()(HDS& hds);
protected:
	unsigned _ver_num;
	float* _vertices;
	unsigned _fac_num;
	unsigned* _index;
};

template <class HDS>
void Memory_Modifier<HDS>::operator()(HDS& target)
{
	CGAL::Polyhedron_Builder<HDS> B(target, true);
	typedef typename HDS::Traits Traits;
	typedef typename HDS::Vertex_handle Vertex_handle;
	typedef typename Traits::Point_3 Point;
	typedef typename HDS::Face_handle       Face_handle;
	typedef typename HDS::Face_handle       Facet_handle;

	B.begin_surface(_ver_num, _fac_num, 4*_fac_num);
	//read in all vertices
	for(unsigned i = 0; i < _ver_num; ++i)
	{
		float* ver = _vertices + 3*i;
		Vertex_handle h = B.add_vertex(Point(*ver, *(ver+1), *(ver+2)));
		h->id() = i;
	}

	if(B.error())
	{
		B.rollback();
		return;
	}

	//read in all facets
	for(unsigned i = 0; i < _fac_num; ++i)
	{
		B.save(*(_index + 3*i), *(_index + 3*i + 1), *(_index + 3*i + 2));
		Facet_handle h = B.begin_facet();
		h->id() = i;

		for(unsigned j = 0; j < 3; ++j)
		{
			unsigned* index = _index + 3*i + j;
			B.add_vertex_to_facet(*index);
		}
		B.end_facet();

		B.store();
	}

	if(B.error())
	{
		B.rollback();
		return;
	}

	if(B.check_unconnected_vertices())
	{
		if(!B.remove_unconnected_vertices())
		{
			B.rollback();
			return;
		}
	}

	B.end_surface();
}

template<class TPoly>
class Memory_Importer
{
	typedef typename TPoly::HalfedgeDS HalfedgeDS;
	typedef typename Memory_Modifier<HalfedgeDS> Modifier;
public:
	static bool Build_From_Memory(unsigned ver_num, float* vertices, unsigned fac_num, unsigned* index, TPoly& p)
	{
		if(ver_num <= 0 || fac_num <= 0 || !vertices || !index)
			return false;

		Modifier modifier(ver_num, vertices, fac_num, index);
		p.delegate(modifier);

		return true;
	}
};
#endif // _POLYHEDRON_BUILDER