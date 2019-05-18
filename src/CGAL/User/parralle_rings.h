#ifndef PARRALLE_RINGS_H_
#define PARRALLE_RINGS_H_

#include <cassert>
#include <vector>
#include <map>
#include <boost\graph\graph_traits.hpp>

//---------------------------------------------------------------------------
//T_PolyhedralSurf_rings
//---------------------------------------------------------------------------
template < class TPoly >
class T_Parralle_rings
{
	const TPoly& P;
protected:
	//Polyhedron
	typedef typename boost::graph_traits<TPoly>::vertex_descriptor                     Vertex_const_handle;
	typedef typename boost::graph_traits<TPoly>::halfedge_descriptor                   Halfedge_const_handle;
	typedef typename boost::graph_traits<TPoly>::vertex_iterator                   Vertex_const_iterator;
	typedef typename CGAL::Halfedge_around_target_circulator<TPoly> Halfedge_around_vertex_const_circulator;

	std::vector<size_t> visited_tag;
	//i >= 1; from a start vertex on the current i-1 ring, push non-visited neighbors
	//of start in the nextRing and set indices to i. Also add these vertices in all.
	void push_neighbours_of(const Vertex_const_handle start, const int ith,
		std::vector < Vertex_const_handle > &nextRing,
		std::vector < Vertex_const_handle > &all);

	//i >= 1, from a currentRing i-1, collect all neighbors, set indices
	//to i and store them in nextRing and all.
	void collect_ith_ring(const int ith,
		std::vector < Vertex_const_handle > &currentRing,
		std::vector < Vertex_const_handle > &nextRing,
		std::vector < Vertex_const_handle > &all);

public:
	T_Parralle_rings(const TPoly& P);

	//collect i>=1 rings : all neighbours up to the ith ring,
	void collect_i_rings(const Vertex_const_handle v,
		const int ring_i,
		std::vector < Vertex_const_handle >& all);

	//collect enough rings (at least 1), to get at least min_nb of neighbors
	void collect_enough_rings(const Vertex_const_handle v,
		const unsigned int min_nb,
		std::vector < Vertex_const_handle >& all);
};

////IMPLEMENTATION/////////////////////////////////////////////////////////////////////

template < class TPoly >
T_Parralle_rings <TPoly>::
T_Parralle_rings(const TPoly& P)
	: P(P)
{
	visited_tag.clear();
}

template < class TPoly >
void T_Parralle_rings <TPoly>::
push_neighbours_of(const Vertex_const_handle start, const int ith,
	std::vector < Vertex_const_handle > &nextRing,
	std::vector < Vertex_const_handle > &all)
{
	Vertex_const_handle v;
	Halfedge_around_vertex_const_circulator
		hedgeb(halfedge(start, P), P), hedgee = hedgeb;

	CGAL_For_all(hedgeb, hedgee)
	{
		v = target(opposite(*hedgeb, P), P);

		std::vector<size_t>::iterator it = std::find(visited_tag.begin(), visited_tag.end(), v->id());
		if (it != visited_tag.end())  continue;//if visited: next

		visited_tag.push_back(v->id());
		nextRing.push_back(v);
		all.push_back(v);
	}
}

template <class TPoly>
void T_Parralle_rings <TPoly>::
collect_ith_ring(const int ith, std::vector < Vertex_const_handle > &currentRing,
	std::vector < Vertex_const_handle > &nextRing,
	std::vector < Vertex_const_handle > &all)
{
	typename std::vector < Vertex_const_handle >::const_iterator
		itb = currentRing.begin(), ite = currentRing.end();

	CGAL_For_all(itb, ite) push_neighbours_of(*itb, ith, nextRing, all);
}

template <class TPoly>
void T_Parralle_rings <TPoly>::
collect_i_rings(const Vertex_const_handle v,
	const int ring_i,
	std::vector < Vertex_const_handle >& all)
{
	std::vector<Vertex_const_handle> current_ring, next_ring;
	std::vector<Vertex_const_handle> *p_current_ring, *p_next_ring;
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
void T_Parralle_rings <TPoly>::
collect_enough_rings(const Vertex_const_handle v,
	const unsigned int min_nb,
	std::vector < Vertex_const_handle >& all)
{
	std::vector<Vertex_const_handle> current_ring, next_ring;
	std::vector<Vertex_const_handle> *p_current_ring, *p_next_ring;

	//initialize
	p_current_ring = &current_ring;
	p_next_ring = &next_ring;
	current_ring.push_back(v);
	visited_tag.push_back(v->id());
	all.push_back(v);

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

#endif // PARRALLE_RINGS_H_
