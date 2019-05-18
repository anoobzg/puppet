#ifndef OPTIMIZE_PARRALLE_RINGS_H_
#define OPTIMIZE_PARRALLE_RINGS_H_

#include <cassert>
#include <vector>
#include <map>

template < class TPoly >
class Optimize_Parralle_rings
{
	typedef typename TPoly::Traits Traits;
	typedef typename Traits::Point_3 Point;
	typedef typename TPoly::Vertex_handle Vertex_handle;
	typedef typename TPoly::Halfedge_handle Halfedge_handle;

protected:
	std::vector<size_t> visited_tag;
	//i >= 1; from a start vertex on the current i-1 ring, push non-visited neighbors
	//of start in the nextRing and set indices to i. Also add these vertices in all.
	void push_neighbours_of(const Vertex_handle start, const int ith,
		std::vector < Vertex_handle > &nextRing,
		std::vector < Point > &all);

	//i >= 1, from a currentRing i-1, collect all neighbors, set indices
	//to i and store them in nextRing and all.
	void collect_ith_ring(const int ith,
		std::vector < Vertex_handle > &currentRing,
		std::vector < Vertex_handle > &nextRing,
		std::vector < Point > &all);

public:
	Optimize_Parralle_rings();

	//collect i>=1 rings : all neighbours up to the ith ring,
	void collect_i_rings(const Vertex_handle v,
		const int ring_i,
		std::vector < Point >& all);

	//collect enough rings (at least 1), to get at least min_nb of neighbors
	void collect_enough_rings(const Vertex_handle v,
		const unsigned int min_nb,
		std::vector < Point >& all);
};

////IMPLEMENTATION/////////////////////////////////////////////////////////////////////

template < class TPoly >
Optimize_Parralle_rings <TPoly>::Optimize_Parralle_rings()
{
	visited_tag.clear();
}

template < class TPoly >
void Optimize_Parralle_rings <TPoly>::push_neighbours_of(const Vertex_handle start, const int ith,
	std::vector < Vertex_handle > &nextRing, std::vector < Point > &all)
{
	Halfedge_handle h = start->halfedge();
	Halfedge_handle he = h;

	
	do{
		Vertex_handle v = h->opposite()->vertex();
		h = h->next()->opposite();

		std::vector<size_t>::iterator it = std::find(visited_tag.begin(), visited_tag.end(), v->id());
		
		if (it == visited_tag.end())
		{
			visited_tag.push_back(v->id());
			nextRing.push_back(v);
			all.push_back(v->point());
		}
	}while(h != he);
}

template <class TPoly>
void Optimize_Parralle_rings <TPoly>::collect_ith_ring(const int ith, std::vector < Vertex_handle > &currentRing,
	std::vector < Vertex_handle > &nextRing, std::vector < Point > &all)
{
	typename std::vector < Vertex_handle >::const_iterator
		itb = currentRing.begin(), ite = currentRing.end();

	CGAL_For_all(itb, ite) push_neighbours_of(*itb, ith, nextRing, all);
}

template <class TPoly>
void Optimize_Parralle_rings <TPoly>::collect_i_rings(const Vertex_handle v,
	const int ring_i, std::vector < Point >& all)
{
	std::vector<Vertex_handle> current_ring, next_ring;
	std::vector<Vertex_handle> *p_current_ring, *p_next_ring;
	assert(ring_i >= 1);
	//initialize
	p_current_ring = &current_ring;
	p_next_ring = &next_ring;
	visited_tag.push_back(v->id());
	current_ring.push_back(v);
	all.push_back(v);

	for (int i = 1; i <= ring_i; i++)
	{
		collect_ith_ring(i, *p_current_ring, *p_next_ring, all);
		//next round must be launched from p_nextRing...
		p_current_ring->clear();
		std::swap(p_current_ring, p_next_ring);
	}

	visited_tag.clear();
}

template <class TPoly>
void Optimize_Parralle_rings <TPoly>::collect_enough_rings(const Vertex_handle v,
	const unsigned int min_nb, std::vector < Point >& all)
{
	std::vector<Vertex_handle> current_ring, next_ring;
	std::vector<Vertex_handle> *p_current_ring, *p_next_ring;

	//initialize
	p_current_ring = &current_ring;
	p_next_ring = &next_ring;
	current_ring.push_back(v);
	visited_tag.push_back(v->id());
	all.push_back(v->point());

	int i = 1;

	while ((all.size() < min_nb) && (p_current_ring->size() != 0))
	{
		collect_ith_ring(i, *p_current_ring, *p_next_ring, all);
		//next round must be launched from p_nextRing...
		p_current_ring->clear();
		std::swap(p_current_ring, p_next_ring);
		i++;
	}

	visited_tag.clear();
}

#endif // OPTIMIZE_PARRALLE_RINGS_H_
