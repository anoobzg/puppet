/*
Szymon Rusinkiewicz
Princeton University

TriMesh_connectivity.cc
Manipulate data structures that describe connectivity between faces and verts.
*/


#include "TriMesh.h"
using namespace std;


namespace trimesh {

// Find the face across each edge from each other face (-1 on boundary)
// If topology is bad, not necessarily what one would expect...
void TriMesh::need_across_edge()
{
	if (!across_edge.empty())
		return;

	if (adjacentfaces.empty())
		return;

	dprintf("Finding across-edge maps... ");

	int nf = faces.size();
	across_edge.resize(nf, Face(-1,-1,-1));

#pragma omp parallel for
	for (int i = 0; i < nf; i++) {
		for (int j = 0; j < 3; j++) {
			int v1 = faces[i][NEXT_MOD3(j)];
			int v2 = faces[i][PREV_MOD3(j)];
			const vector<int> &a1 = adjacentfaces[v1];
			for (size_t k1 = 0; k1 < a1.size(); k1++) {
				int other = a1[k1];
				if (other == i)
					continue;
				int v2_in_other = faces[other].indexof(v2);
				if (v2_in_other < 0)
					continue;
				if (faces[other][NEXT_MOD3(v2_in_other)] != v1)
					continue;
				across_edge[i][j] = other;
				break;
			}
		}
	}

	dprintf("Done.\n");
}

} // namespace trimesh
