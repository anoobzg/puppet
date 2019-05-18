#pragma once
#ifndef PATCH_BUILDER_H
#define PATCH_BUILDER_H

#include "cgaltype.h"
#include <CGAL\boost\graph\graph_traits_Polyhedron_3.h>
#include <stack>

class Patch_Builder
{
public:
	static unsigned Build(Polyhedron& p)
	{
		unsigned face_patch = 0;
		Facet_handle t, s;

		std::stack<Facet_handle> trilist;
		for(Facet_iterator it = p.facets_begin(); it != p.facets_end(); ++it)
		{
			if(it->patch() == 0)
			{
				face_patch++;
				trilist.push(it);
				it->patch() = face_patch;
				while(trilist.size())
				{
					t = trilist.top();
					trilist.pop();
					if((s = t->halfedge()->opposite()->facet()) != NULL && (!s->patch())) { trilist.push(s); s->patch() = face_patch; }
					if((s = t->halfedge()->next()->opposite()->facet()) != NULL && (!s->patch())) { trilist.push(s); s->patch() = face_patch; }
					if((s = t->halfedge()->prev()->opposite()->facet()) != NULL && (!s->patch())) { trilist.push(s); s->patch() = face_patch; }
				}
			}
		}
		return face_patch;
	}
};
#endif // MESH_CHECKER_H