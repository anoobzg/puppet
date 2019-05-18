#pragma once
#ifndef KEY_VERTEX_DIOLATER
#define KEY_VERTEX_DIOLATER
#include <algorithm>

#include "tag.h"

template<class TPoly>
class Keyvertex_Diolater
{
private:
	typedef typename TPoly::Traits Traits;
	typedef typename Traits::Point_3 Point;
	typedef typename Traits::Vector_3 Vector;
	typedef typename Traits::FT FT;
	typedef typename TPoly::Vertex Vertex;
	typedef typename TPoly::Vertex_iterator Vertex_iterator;
	typedef typename TPoly::Halfedge_handle Halfedge_handle;
	typedef typename TPoly::Vertex_handle Vertex_handle;

public:
	static void Diolate(TPoly& p, size_t ver_num, unsigned step)
	{
		{
			std::vector<Vertex_handle> diolated;
			for (Vertex_iterator it = p.vertices_begin(); it != p.vertices_end(); ++it) {
				if(it->tag() == e_none_region)
					continue;

				Halfedge_handle h = (*it).halfedge();
				Halfedge_handle he = h;
				do {
					Vertex_handle v = h->opposite()->vertex();
					if(v->tag() == e_none_region)
						diolated.push_back(v);
					h = h->next()->opposite();
				} while (h != he);
			}

			std::for_each(diolated.begin(), diolated.end(), [](Vertex_handle h){
				h->tag() = e_key_region_1;
			});
		}

		for (unsigned i = 1; i < step; ++i)
		{
			std::vector<Vertex_handle> diolated;
			for (Vertex_iterator it = p.vertices_begin(); it != p.vertices_end(); ++it) {
				key_region_tag n_tag = key_region_tag(i+2);
				if(it->tag() != n_tag)
					continue;

				Halfedge_handle h = (*it).halfedge();
				Halfedge_handle he = h;
				do {
					Vertex_handle v = h->opposite()->vertex();
					if(v->tag() == e_none_region)
						diolated.push_back(v);
					h = h->next()->opposite();
				} while (h != he);
			}

			std::for_each(diolated.begin(), diolated.end(), [&i](Vertex_handle h){
				h->tag() = key_region_tag(i+3);
			});
		}
	}
};

#endif // KEY_VERTEX_DIOLATER