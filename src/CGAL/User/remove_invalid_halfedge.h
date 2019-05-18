#pragma once
#ifndef REMOVE_INVALID_HALFEDGE
#define REMOVE_INVALID_HALFEDGE

#include "cgaltype.h"
#include <set>

class Invalid_Halfedge_Remover
{
	typedef std::set<Halfedge_iterator> halfedge_erease;
	typedef halfedge_erease::iterator halfedge_erease_iterator;
public:
	static void Remove(Polyhedron& p)
	{
		halfedge_erease need_remove;
		for(Halfedge_iterator hit = p.halfedges_begin(); hit != p.halfedges_end(); ++hit)
		{
			if(!(*hit).is_border())
				continue;

			if((*hit).next() == Halfedge_handle())
			{
				need_remove.insert(hit);
				continue;
			}
			if((*hit).opposite() == (*hit).next())
			{
				need_remove.insert(hit);
				continue;
			}
			if((*hit).prev() == Halfedge_handle())
			{
				need_remove.insert(hit);
				continue;
			}
			if((*hit).prev()->next() != &(*hit))
			{
				need_remove.insert(hit);
				continue;
			}
			if((*hit).next()->prev() != &(*hit))
			{
				need_remove.insert(hit);
				continue;
			}
		}

		for(halfedge_erease_iterator it = need_remove.begin(); it != need_remove.end(); ++it)
		{
			p.hds().edges_erase(*it);
		}
	}
};
#endif // REMOVE_INVALID_HALFEDGE