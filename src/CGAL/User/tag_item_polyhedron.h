#ifndef TAG_ITEM_POLYHEDRON
#define TAG_ITEM_POLYHEDRON

#include "kernel.h"
#include <CGAL/Polyhedron_3.h>
#include <CGAL/HalfedgeDS_vertex_max_base_with_id.h>
#include <CGAL/HalfedgeDS_halfedge_max_base_with_id.h>
#include <CGAL/HalfedgeDS_face_max_base_with_id.h>

template < class Refs, class P, class Tag>
class HalfedgeDS_vertex_max_base_with_tag : public CGAL::HalfedgeDS_vertex_max_base_with_id<Refs, P, std::size_t>
{
public:
    typedef HalfedgeDS_vertex_max_base_with_id<Refs, P, std::size_t> Base ;
    typedef P Point ;
	typedef Tag Vertex_Tag;
private:
	Tag m_tag;
public:

    HalfedgeDS_vertex_max_base_with_tag() : m_tag(Tag(0)) {}
    HalfedgeDS_vertex_max_base_with_tag( Point const& p) : Base(p), m_tag(Tag(0)) {}
    
    Tag&       tag()       { return m_tag; }
    Tag const& tag() const { return m_tag; }
};

template<class VTag>
class Polyhedron_items_with_tag_3 {
public:
    template < class Refs, class Traits>
    struct Vertex_wrapper {
        typedef typename Traits::Point_3 Point;
        typedef HalfedgeDS_vertex_max_base_with_tag< Refs, Point, VTag> Vertex;
    };
    template < class Refs, class Traits>
    struct Halfedge_wrapper {
        typedef CGAL::HalfedgeDS_halfedge_max_base_with_id<Refs, std::size_t> Halfedge;
    };
    template < class Refs, class Traits>
    struct Face_wrapper {
        typedef CGAL::HalfedgeDS_face_max_base_with_id< Refs, CGAL::Tag_false, std::size_t>  Face;
    };
};

#endif  // TAG_ITEM_POLYHEDRON
