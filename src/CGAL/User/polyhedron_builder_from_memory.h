#ifndef _POLYHEDRON_BUILDER
#define _POLYHEDRON_BUILDER
#include "cgaltype.h"
#include <CGAL\Modifier_base.h>
#include <CGAL\Polyhedron_incremental_builder_3.h>

template <class HDS>
class Polyhedron_Memory_Modifier : public CGAL::Modifier_base<HDS>
{
public:
	Polyhedron_Memory_Modifier(unsigned ver_num, float* vertices, unsigned fac_num, unsigned* index, std::vector<Vertex_handle>& index_vertex)
	:_ver_num(ver_num), _vertices(vertices), _fac_num(fac_num), _index(index),_error(false), _index_vertex(index_vertex) {}
	void operator()(HDS& hds);
	bool error() { return _error; }
protected:
	unsigned _ver_num;
	float* _vertices;
	unsigned _fac_num;
	unsigned* _index;
	std::vector<Vertex_handle>& _index_vertex;

	bool _error;
};

template <class HDS>
class Polyhedron_Memory_Modifier_1 : public CGAL::Modifier_base<HDS>
{
public:
	Polyhedron_Memory_Modifier_1(unsigned ver_num, float* vertices, unsigned fac_num, unsigned* index)
	:_ver_num(ver_num), _vertices(vertices), _fac_num(fac_num), _index(index) {}
	void operator()(HDS& hds);
protected:
	unsigned _ver_num;
	float* _vertices;
	unsigned _fac_num;
	unsigned* _index;
};

template <class HDS>
void Polyhedron_Memory_Modifier<HDS>::operator()(HDS& target)
{
	CGAL::Polyhedron_incremental_builder_3<HDS> B(target, true);
	typedef typename HDS::Traits Traits;

	B.begin_surface(_ver_num, _fac_num, 4*_fac_num);
	_index_vertex.reserve(_ver_num);
	//read in all vertices
	for(unsigned i = 0; i < _ver_num; ++i)
	{
		float* ver = _vertices + 3*i;
		Polyhedron::Vertex_handle h = B.add_vertex(Point(*ver, *(ver+1), *(ver+2)));
		h->id() = i;
		_index_vertex.push_back(h);
	}

	if(B.error())
	{
		B.rollback();
		_error = true;
		return;
	}

	//read in all facets
	for(unsigned i = 0; i < _fac_num; ++i)
	{
		B.save(*(_index + 3*i), *(_index + 3*i + 1), *(_index + 3*i + 2));
		Polyhedron::Facet_handle h = B.begin_facet();
		h->id() = i;

		for(unsigned j = 0; j < 3; ++j)
		{
			unsigned* index = _index + 3*i + j;
			B.add_vertex_to_facet(*index);
		}
		B.end_facet();

		B.restore(h);
	}

	if(B.error())
	{
		B.rollback();
		_error = true;
		return;
	}

	if(B.check_unconnected_vertices())
	{
		if(!B.remove_unconnected_vertices())
		{
			B.rollback();
			_error = true;
			return;
		}
	}

	B.end_surface();
}

template <class HDS>
void Polyhedron_Memory_Modifier_1<HDS>::operator()(HDS& target)
{
	CGAL::Polyhedron_incremental_builder_3<HDS> B(target, true);
	typedef typename HDS::Traits Traits;

	B.begin_surface(_ver_num, _fac_num, 4*_fac_num);
	//read in all vertices
	for(unsigned i = 0; i < _ver_num; ++i)
	{
		float* ver = _vertices + 3*i;
		Polyhedron::Vertex_handle h = B.add_vertex(Point(*ver, *(ver+1), *(ver+2)));
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
		Polyhedron::Facet_handle h = B.begin_facet();
		h->id() = i;

		for(unsigned j = 0; j < 3; ++j)
		{
			unsigned* index = _index + 3*i + j;
			B.add_vertex_to_facet(*index);
		}
		B.end_facet();

		B.restore(h);
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

class Polyhedron_Builder
{
public:
	static bool Build_From_Memory(unsigned ver_num, float* vertices, unsigned fac_num, unsigned* index, Polyhedron& p, std::vector<Vertex_handle>& index_vertex)
	{
		if(ver_num <= 0 || fac_num <= 0 || !vertices || !index)
			return false;


		typedef Polyhedron::HalfedgeDS HalfedgeDS;
		typedef Polyhedron_Memory_Modifier<HalfedgeDS> Modifier;
		Modifier modifier(ver_num, vertices, fac_num, index, index_vertex);
		p.delegate(modifier);

		return !modifier.error();
	}

	static bool Build_From_Memory(unsigned ver_num, float* vertices, unsigned fac_num, unsigned* index, Polyhedron& p)
	{
		if(ver_num <= 0 || fac_num <= 0 || !vertices || !index)
			return false;


		typedef Polyhedron::HalfedgeDS HalfedgeDS;
		typedef Polyhedron_Memory_Modifier_1<HalfedgeDS> Modifier;
		Modifier modifier(ver_num, vertices, fac_num, index);
		p.delegate(modifier);

		return true;
	}
};
#endif // _POLYHEDRON_BUILDER