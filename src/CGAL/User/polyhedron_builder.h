#ifndef POLYHEDRON_BUILDER_3_H
#define POLYHEDRON_BUILDER_3_H

#include <CGAL/basic.h>
#include <CGAL/HalfedgeDS_decorator.h>
#include <CGAL/Unique_hash_map.h>
#include <CGAL/IO/Verbose_ostream.h>
#include <vector>
#include <cstddef>

namespace CGAL {

template < class HalfedgeDS_>
class Polyhedron_Builder {
public:
    typedef HalfedgeDS_                     HDS; // internal
    typedef HalfedgeDS_                     HalfedgeDS;
    typedef typename HDS::Vertex            Vertex;
    typedef typename HDS::Halfedge          Halfedge;
    typedef typename HDS::Face              Face;
    typedef typename HDS::Vertex_handle     Vertex_handle;
    typedef typename HDS::Halfedge_handle   Halfedge_handle;
    typedef typename HDS::Face_handle       Face_handle;
    typedef typename HDS::Face_handle       Facet_handle;
    typedef typename Vertex::Base           VBase;
    typedef typename Halfedge::Base         HBase;
    typedef typename Vertex::Point          Point_3;
    typedef typename HDS::size_type         size_type;

protected:
    typedef typename HDS::Vertex_iterator           Vertex_iterator;
    typedef typename HDS::Halfedge_iterator         Halfedge_iterator;
    typedef std::vector<Vertex_handle>				Random_access_index;

    bool                      m_error;
    bool                      m_verbose;
    HDS&                      hds;
    size_type                 new_vertices;
    size_type                 new_faces;
    size_type                 new_halfedges;
    Face_handle               current_face;
    Random_access_index       index_to_vertex_map;

    Halfedge_handle           g1;      // first halfedge, 0 denotes none.
    Halfedge_handle           gprime;
    Halfedge_handle           h1;      // current halfedge
    size_type                 w1;      // first vertex.
    size_type                 w2;      // second vertex.
    size_type                 v1;      // current vertex
    bool                      first_vertex;
    bool                      last_vertex;

	size_type				  m_index_edge;
	size_type			  m_neighbour_face[3];
    //save
    std::map<Vertex_handle, Halfedge_handle> m_vertex_save;
	std::vector<Halfedge_handle> m_new_halfedges;
	std::map<Halfedge_handle, Halfedge_handle> m_next_halfedge_save;

    Halfedge_handle get_vertex_to_edge_map( size_type i) {
        return index_to_vertex_map[i]->halfedge();
    }

    void set_vertex_to_edge_map( size_type i, Halfedge_handle h) {
        index_to_vertex_map[i]->VBase::set_halfedge(h);
    }

public:
    bool error() const { return m_error; }
    void save(std::size_t v1, std::size_t v2, std::size_t v3);
	void store();

    Polyhedron_Builder( HDS& h, bool verbose = false)
      : m_error( false), m_verbose( verbose), hds(h), m_index_edge(0) {
    }

    ~Polyhedron_Builder() CGAL_NOEXCEPT(CGAL_NO_ASSERTIONS_BOOL)
    {
    }

    void begin_surface( std::size_t v, std::size_t f, std::size_t h = 0);

    Vertex_handle add_vertex( const Point_3& p) {
        // adds p to the vertex list.
        if ( hds.size_of_vertices() >= hds.capacity_of_vertices()) {
            Verbose_ostream verr( m_verbose);
            verr << " " << std::endl;
            verr << "CGAL::Polyhedron_Builder<HDS>::"
                 << std::endl;
            verr << "add_vertex(): capacity error: more than " << new_vertices
                 << " vertices added." << std::endl;
            m_error = true;
            return Vertex_handle();
        }
        HalfedgeDS_decorator<HDS> decorator(hds);
        Vertex_handle v = decorator.vertices_push_back( Vertex(p));
        index_to_vertex_map.push_back( v);
        decorator.set_vertex_halfedge( v, Halfedge_handle());

        v->id() = new_vertices++;
        return v;
    }

    Facet_handle begin_facet() {
        // starts a facet.
        if ( m_error)
            return Facet_handle();

        if ( hds.size_of_faces() >= hds.capacity_of_faces()) {
            Verbose_ostream verr( m_verbose);
            verr << " " << std::endl;
            verr << "CGAL::Polyhedron_Builder<HDS>::"
                 << std::endl;
            verr << "begin_facet(): capacity error: more than " << new_vertices
                 << " facets added." << std::endl;
            m_error = true;
            return Facet_handle();
        }
		m_index_edge = 0;
		m_neighbour_face[0] = -1;
		m_neighbour_face[1] = -2;
		m_neighbour_face[2] = -3;

        // initialize all status variables.
        first_vertex = true;  // denotes 'no vertex yet'
        g1 =  Halfedge_handle();  // denotes 'no halfedge yet'
        last_vertex = false;

        HalfedgeDS_decorator<HDS> decorator(hds);
        current_face = decorator.faces_push_back( Face());
		current_face->id() = new_faces++;
        return current_face;
    }

    void add_vertex_to_facet( std::size_t i);

    Halfedge_handle end_facet() {
        // ends a facet.
        if ( m_error)
            return Halfedge_handle();

        CGAL_assertion( ! first_vertex);
        // cleanup all static status variables
        add_vertex_to_facet( w1);
        if ( m_error)
            return Halfedge_handle();
        last_vertex = true;
        add_vertex_to_facet( w2);
        if ( m_error)
            return Halfedge_handle();

        HalfedgeDS_items_decorator<HDS> decorator;
        Halfedge_handle h = get_vertex_to_edge_map(w1);
        decorator.set_face_halfedge( current_face, h);
        return h;
    }

    void end_surface();
        // ends the construction.

    bool check_unconnected_vertices();

    bool remove_unconnected_vertices();

    void rollback();

protected:
    Halfedge_handle lookup_hole( std::size_t w) {
        CGAL_assertion( w < new_vertices);
        return lookup_hole( get_vertex_to_edge_map( w));
    }

    Halfedge_handle lookup_halfedge( size_type w, size_type v) {
        // Pre: 0 <= w,v < new_vertices
        // Case a: It exists an halfedge g from w to v:
        //     g must be a border halfedge and the facet of g->opposite()
        //     must be set and different from the current facet.
        //     Set the facet of g to the current facet. Return the
        //     halfedge pointing to g.
        // Case b: It exists no halfedge from w to v:
        //     Create a new pair of halfedges g and g->opposite().
        //     Set the facet of g to the current facet and g->opposite()
        //     to a border halfedge. Assign the vertex references.
        //     Set g->opposite()->next() to g. Return g->opposite().
        CGAL_assertion( ! last_vertex);
        HalfedgeDS_items_decorator<HDS> decorator;
        Halfedge_handle e = get_vertex_to_edge_map( w);
        if ( e != Halfedge_handle()) {
            CGAL_assertion( e->vertex() == index_to_vertex_map[w]);
            // check that the facet has no self intersections
            if ( current_face != Face_handle() && current_face == decorator.get_face(e))
			{
                Verbose_ostream verr( m_verbose);
                verr << " " << std::endl;
                verr << "CGAL::Polyhedron_Builder<HDS>::"<< std::endl;
                verr << "lookup_halfedge(): input error: facet "
                     << current_face->id() << " has a self intersection at vertex "
                     << w << "." << std::endl;
                m_error = true;
                return Halfedge_handle();
            }
            Halfedge_handle start_edge( e);
            do {
                if ( e->next()->vertex() == index_to_vertex_map[v]) {
                    if ( ! e->next()->is_border()) {
                        Verbose_ostream verr( m_verbose);
                        verr << " " << std::endl;
                        verr << "CGAL::Polyhedron_Builder"
                                "<HDS>::" << std::endl;
                        verr << "lookup_halfedge(): input error: facet "
                             << current_face->id() << " shares a halfedge from "
                                "vertex " <<  w << " to vertex " << v
                             << " with";
                        if (  m_verbose && current_face != Face_handle())
                            verr << " facet "
                                 << decorator.get_face(e->next())->id()
                                 << '.' << std::endl;
                        else
                            verr << " another facet." << std::endl;
                        m_error = true;
                        return Halfedge_handle();
                    }
                    CGAL_assertion( ! e->next()->opposite()->is_border());
                    if ( current_face != Face_handle() && current_face ==
                         decorator.get_face( e->next()->opposite())) {
                        Verbose_ostream verr( m_verbose);
                        verr << " " << std::endl;
                        verr << "CGAL::Polyhedron_Builder"
                                "<HDS>::" << std::endl;
                        verr << "lookup_halfedge(): input error: facet "
                             << current_face->id() << " has a self intersection "
                                "at the halfedge from vertex " << w
                             << " to vertex " << v << "." << std::endl;
                        m_error = true;
                        return Halfedge_handle();
                    }
                    decorator.set_face( e->next(), current_face);
                    // The following line prevents e->next() to be picked
                    // by get_vertex_to_edge_map(v) in an upcoming call
                    // of lookup_halfedge(v, *)
                    set_vertex_to_edge_map( v, e->next()->next()->opposite());

					m_neighbour_face[m_index_edge++] = e->next()->opposite()->facet()->id();
					if(m_index_edge >= 3 && (m_neighbour_face[0] == m_neighbour_face[1]) && (m_neighbour_face[1] == m_neighbour_face[2]))
					{
                        Verbose_ostream verr( m_verbose);
                        verr << " " << std::endl;
                        verr << "CGAL::Polyhedron_Builder"
                                "<HDS>::" << std::endl;
                        verr << "lookup_halfedge(): input error: facet "
                             << current_face->id() << "already exist."
                             <<std::endl;
                        m_error = true;
                        return Halfedge_handle();
					}
                    return e;
                }
                e = e->next()->opposite();
            } while ( e != start_edge);
        }
        // create a new halfedge
        if ( hds.size_of_halfedges() >= hds.capacity_of_halfedges()) {
            Verbose_ostream verr( m_verbose);
            verr << " " << std::endl;
            verr << "CGAL::Polyhedron_Builder<HDS>::"
                 << std::endl;
            verr << "lookup_halfedge(): capacity error: more than "
                 << new_halfedges << " halfedges added while creating facet"
                 << current_face->id() << '.' << std::endl;
            m_error = true;
            return Halfedge_handle();
        }
        e = hds.edges_push_back( Halfedge(), Halfedge());
        e->id() = new_halfedges++;

        decorator.set_face( e, current_face);
        e->HBase::set_vertex( index_to_vertex_map[v]);
        e->HBase::set_next( Halfedge_handle());
        decorator.set_prev( e, e->opposite());
        e = e->opposite();

		e->id() = new_halfedges++;
        e->HBase::set_vertex( index_to_vertex_map[w]);
        e->HBase::set_next( e->opposite());

		m_new_halfedges.push_back(e);
        return e;
    }

    Halfedge_handle lookup_hole( Halfedge_handle e) {
        // Halfedge e points to a vertex w. Walk around w to find a hole
        // in the facet structure. Report an error if none exist. Return
        // the halfedge at this hole that points to the vertex w.
        CGAL_assertion( e != Halfedge_handle());
        HalfedgeDS_items_decorator<HDS> decorator;
        Halfedge_handle start_edge( e);
        do {
            if ( e->next()->is_border()) {
                return e;
            }
            e = e->next()->opposite();
        } while ( e != start_edge);

        Verbose_ostream verr( m_verbose);
        verr << " " << std::endl;
        verr << "CGAL::Polyhedron_Builder<HDS>::" << std::endl;
        verr << "lookup_hole(): input error: at vertex "
             << e->vertex()->id()
             << " a closed surface already exists and facet "
             << current_face->id() << " is nonetheless adjacent." << std::endl;
        if (  m_verbose && current_face != Face_handle()) {
            verr << "             The closed cycle of facets is:";
            do {
                if ( ! e->is_border())
                    verr << " " << decorator.get_face(e)->id();
                e = e->next()->opposite();
            } while ( e != start_edge);
            verr << '.' << std::endl;
        }
        m_error = true;
        return Halfedge_handle();
    }
};

template < class HDS>
void Polyhedron_Builder<HDS>::rollback()
{
    hds.clear();
    m_error = false;
}

template < class HDS>  CGAL_MEDIUM_INLINE
void Polyhedron_Builder<HDS>::begin_surface( std::size_t v, std::size_t f, std::size_t h)
{
    CGAL_assertion( ! m_error);
    new_vertices  = 0;
    new_faces     = 0;
    new_halfedges = 0;

    if ( h == 0) {
        // Use the Eulerian equation for connected planar graphs. We do
        // not know the number of facets that are holes and we do not
        // know the genus of the surface. So we add 12 and a factor of
        // 5 percent.
      h = (std::size_t)((double)(v + f - 2 + 12) * 2.1);
    }
    hds.reserve( hds.size_of_vertices()  + v,
                 hds.size_of_halfedges() + h,
                 hds.size_of_faces()     + f);

    index_to_vertex_map.clear();
    index_to_vertex_map.reserve(v);
}

template < class HDS>
void Polyhedron_Builder<HDS>::add_vertex_to_facet( std::size_t v2)
{
    if ( m_error)
        return;

    if ( v2 >= new_vertices) {
        Verbose_ostream verr( m_verbose);
        verr << " " << std::endl;
        verr << "CGAL::Polyhedron_Builder<HDS>::"
             << std::endl;
        verr << "add_vertex_to_facet(): vertex index " << v2
             << " is out-of-range [0," << new_vertices-1 << "]."
             << std::endl;
        m_error = true;
        return;
    }
    HalfedgeDS_items_decorator<HDS> decorator;

    if ( first_vertex) {
        CGAL_assertion( ! last_vertex);
        w1 = v2;
        first_vertex = false;
        return;
    }
    if ( g1 == Halfedge_handle()) {
        CGAL_assertion( ! last_vertex);
        gprime  = lookup_halfedge( w1, v2);
        if ( m_error)
            return;
        h1 = g1 = gprime->next();
        v1 = w2 = v2;
        return;
    }
    // g1, h1, v1, w1, w2 are set. Insert halfedge.
    // Lookup v1-->v2
    Halfedge_handle hprime;
    if ( last_vertex)
        hprime = gprime;
    else {
        hprime = lookup_halfedge( v1, v2);
        if ( m_error)
            return;
    }
    Halfedge_handle h2 = hprime->next();
    CGAL_assertion( ! last_vertex || h2 == g1);
    Halfedge_handle prev = h1->next();
    h1->HBase::set_next( h2);
    decorator.set_prev( h2, h1);

    if ( get_vertex_to_edge_map( v1) == Halfedge_handle()) {  // case 1:
        h2->opposite()->HBase::set_next( h1->opposite());
        decorator.set_prev( h1->opposite(), h2->opposite());
    } else {                                                  // case 2:
        bool b1 = h1->opposite()->is_border();
        bool b2 = h2->opposite()->is_border();
        if ( b1 && b2) {
            Halfedge_handle hole = lookup_hole( v1);
            if ( m_error)
                return;
            CGAL_assertion( hole != Halfedge_handle());
            h2->opposite()->HBase::set_next( hole->next());
            decorator.set_prev( hole->next(), h2->opposite());
            hole->HBase::set_next( h1->opposite());
            decorator.set_prev( h1->opposite(), hole);
        } else if ( b2) {                                     // case 2.b:
            CGAL_assertion( prev->is_border());
            h2->opposite()->HBase::set_next( prev);
            decorator.set_prev( prev, h2->opposite());
        } else if ( b1) {                                     // case 2.c:
            CGAL_assertion( hprime->is_border());
            hprime->HBase::set_next( h1->opposite());
            decorator.set_prev( h1->opposite(), hprime);
        } else if ( h2->opposite()->next() == h1->opposite()) {// case 2.d:
            // f1 == f2
            CGAL_assertion( decorator.get_face( h1->opposite()) ==
                       decorator.get_face( h2->opposite()));
        } else {                                              // case 2.e:
            if ( prev == h2) {                                // case _i:
                // nothing to be done, hole is closed.
            } else {                                          // case _ii:
                CGAL_assertion( prev->is_border());
                CGAL_assertion( hprime->is_border());
                hprime->HBase::set_next( prev);
                decorator.set_prev( prev, hprime);
                // Check whether the halfedges around v1 are connected.
                // It is sufficient to check it for h1 to prev.
                // Assert loop termination:
                CGAL_assertion_code( std::size_t k = 0;)
                // Look for a hole in the facet complex starting at h1.
                Halfedge_handle hole;
                Halfedge_handle e = h1;
                do {
                    if ( e->is_border())
                        hole = e;
                    e = e->next()->opposite();
                    CGAL_assertion( k++ < hds.size_of_halfedges());
                } while ( e->next() != prev && e != h1);
                if ( e == h1) {
                    // disconnected facet complexes
                    if ( hole != Halfedge_handle()) {
                        // The complex can be connected with
                        // the hole at hprime.
                        hprime->HBase::set_next( hole->next());
                        decorator.set_prev( hole->next(), hprime);
                        hole->HBase::set_next( prev);
                        decorator.set_prev( prev, hole);
                    } else {
                        Verbose_ostream verr( m_verbose);
                        verr << " " << std::endl;
                        verr << "CGAL::Polyhedron_Builder<"
                                "HDS>::" << std::endl;
                        verr << "add_vertex_to_facet(): input error: "
                                "disconnected facet complexes at vertex "
                             << v1 << ":" << std::endl;

                        if ( m_verbose && current_face != Face_handle()) {
                            verr << "           involved facets are:";
                            do {
                                if ( ! e->is_border())
                                    verr << " " <<decorator.get_face(e)->id();
                                e = e->next()->opposite();
                            } while ( e != h1);
                            verr << " (closed cycle) and";
                            e = hprime;
                            do {
                                if ( ! e->is_border())
                                    verr << " " << decorator.get_face(e)->id();
                            } while ( e != hprime);
                            verr << "." << std::endl;
                        }
                        m_error = true;
                        return;
                    }
                }
            }
        }
    }
    if ( h1->vertex() == index_to_vertex_map[v1])
        set_vertex_to_edge_map( v1, h1);
    CGAL_assertion( h1->vertex() == index_to_vertex_map[v1]);
    h1 = h2;
    v1 = v2;
}

template < class HDS>  CGAL_MEDIUM_INLINE
void Polyhedron_Builder<HDS>::end_surface()
{
    if ( m_error)
        return;
}

template < class HDS>
bool Polyhedron_Builder<HDS>::check_unconnected_vertices()
{
    if ( m_error)
        return false;
    bool unconnected = false;
    Verbose_ostream verr( m_verbose);
    for ( std::size_t i = 0; i < new_vertices; i++) {
        if ( get_vertex_to_edge_map( i) == Halfedge_handle()) {
            verr << "CGAL::Polyhedron_Builder<HDS>::\n"
                 << "check_unconnected_vertices( verb = true): "
                 << "vertex " << i << " is unconnected." << std::endl;
            unconnected = true;
        }
    }
    return unconnected;
}

template < class HDS>
bool Polyhedron_Builder<HDS>::remove_unconnected_vertices()
{
    if ( m_error)
        return true;
    for( std::size_t i = 0; i < new_vertices; i++) {
        if( get_vertex_to_edge_map( i) == Halfedge_handle()) {
            hds.vertices_erase( index_to_vertex_map[i]);
        }
    }
    return true;
}

template <class HDS>
void Polyhedron_Builder<HDS>::save(std::size_t v1, std::size_t v2, std::size_t v3)
{
	m_vertex_save.clear();
	m_new_halfedges.clear();

    if(v1 < new_vertices)
        m_vertex_save.insert(std::pair<Vertex_handle, Halfedge_handle>(index_to_vertex_map[v1], index_to_vertex_map[v1]->halfedge()));
    if(v2 < new_vertices)
        m_vertex_save.insert(std::pair<Vertex_handle, Halfedge_handle>(index_to_vertex_map[v2], index_to_vertex_map[v2]->halfedge()));
    if(v3 < new_vertices)
        m_vertex_save.insert(std::pair<Vertex_handle, Halfedge_handle>(index_to_vertex_map[v3], index_to_vertex_map[v3]->halfedge()));

    typename std::map<Vertex_handle, Halfedge_handle>::iterator vit = m_vertex_save.begin();
    while(vit != m_vertex_save.end())
    {
        Halfedge_handle end = (*vit).second;
        if(end != Halfedge_handle())
        {
            Halfedge_handle e = end;
            do {
                if ( e->is_border())
                {
                    m_next_halfedge_save.insert( std::pair<Halfedge_handle, Halfedge_handle>(e, e->next()));
                }
                e = e->next()->opposite();
            } while ( e != end);
        }
        ++vit;
    }
}

template <class HDS>
void Polyhedron_Builder<HDS>::store()
{
    if(!m_error)
    {
        m_vertex_save.clear();
		m_new_halfedges.clear();
		m_next_halfedge_save.clear();
        return;
    }

    HalfedgeDS_items_decorator<HDS> decorator;

    typename std::map<Vertex_handle, Halfedge_handle>::iterator vit = m_vertex_save.begin();
    while(vit != m_vertex_save.end())
    {
        set_vertex_to_edge_map( (*vit).first->id(), (*vit).second);
        ++vit;
    }

	typename std::map<Halfedge_handle, Halfedge_handle>::iterator hiter = m_next_halfedge_save.begin();
    while(hiter != m_next_halfedge_save.end())
    {
		Halfedge_handle hh = (*hiter).first;
		Halfedge_handle hhe = (*hiter).second;

        (*hiter).first->HBase::set_next((*hiter).second);
        decorator.set_prev( (*hiter).second, (*hiter).first);
        decorator.set_face((*hiter).first, Facet_handle());
        ++hiter;
    }

	typename std::vector<Halfedge_handle>::iterator hit = m_new_halfedges.begin();
	while(hit != m_new_halfedges.end())
	{
		hds.edges_pop_back();
		++hit;
	}

	hds.faces_pop_back();
    m_error = false;
    m_vertex_save.clear();
	m_new_halfedges.clear();
	m_next_halfedge_save.clear();
}

}
#endif // POLYHEDRON_BUILDER_3_H //
