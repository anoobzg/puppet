#ifndef KEY_REGION_ITEM_POLYHEDRON
#define KEY_REGION_ITEM_POLYHEDRON

#include "kernel.h"
#include "tag_item_polyhedron.h"
#include <CGAL/Polyhedron_3.h>
#include <CGAL/HalfedgeDS_vertex_max_base_with_id.h>
#include <CGAL/HalfedgeDS_halfedge_max_base_with_id.h>
#include <CGAL/HalfedgeDS_face_max_base_with_id.h>

template < class Refs, class P, class V, class F, class Tag>
class HalfedgeDS_vertex_key : public HalfedgeDS_vertex_max_base_with_tag<Refs, P, Tag>
{
public:
    typedef HalfedgeDS_vertex_max_base_with_tag<Refs, P, Tag> Base ;
    typedef P Point ;
	typedef V Vector;
	typedef F FT;

private:
	Vector m_vertex_d1;
	Vector m_vertex_d2;
	Vector m_vertex_normal;
	FT m_vertex_P1;
	FT m_vertex_P2;

	FT m_coefficients[11];
	bool m_p1_calculated;
	bool m_p2_calculated;
public:
	void comply_wrt_given_normal(const Vector& given_normal)
	{
	  if ( given_normal*m_vertex_normal < 0 )
		{
		  m_vertex_normal = -m_vertex_normal;
		  std::swap(m_vertex_d1, m_vertex_d2);

		std::swap(m_coefficients[0],m_coefficients[1]);
		std::swap(m_coefficients[2],m_coefficients[5]);
		std::swap(m_coefficients[3],m_coefficients[4]);
		std::swap(m_coefficients[6],m_coefficients[10]);
		std::swap(m_coefficients[7],m_coefficients[9]);
		for(unsigned i = 0; i < 11; ++i)
		{
			m_coefficients[i] = -m_coefficients[i];
		}
	  }
	}

	FT&		  coff(int i) { return m_coefficients[i];}

    FT&       k1()       { return m_coefficients[0]; }
    FT const& k1() const { return m_coefficients[0]; }

    FT&       k2()       { return m_coefficients[1]; }
    FT const& k2() const { return m_coefficients[1]; }

    FT&       b0()       { return m_coefficients[2]; }
    FT const& b0() const { return m_coefficients[2]; }

    FT&       b3()       { return m_coefficients[5]; }
    FT const& b3() const { return m_coefficients[5]; }

    //FT&       P1()       { return m_vertex_P1; }
    FT& P1()
	{ 
		if(m_p1_calculated)
			return m_vertex_P1;

		m_vertex_P1 =
				3 * m_coefficients[3] * m_coefficients[3]
				+ (m_coefficients[0] - m_coefficients[1])*(m_coefficients[6]
					- 3 * m_coefficients[0] * m_coefficients[0]* m_coefficients[0]);
		return m_vertex_P1;
	}

    //FT&       P2()       { return m_vertex_P2; }
    FT& P2() 
	{ 		
		if(m_p2_calculated)
			return m_vertex_P2;

		m_vertex_P2 =
				3 * m_coefficients[4] * m_coefficients[4]
				+ (-m_coefficients[0] + m_coefficients[1])*(m_coefficients[10]
					- 3 * m_coefficients[1] * m_coefficients[1]* m_coefficients[1]);
		return m_vertex_P2;
	}

    Vector&       d1()       { return m_vertex_d1; }
    Vector const& d1() const { return m_vertex_d1; }

    Vector&       d2()       { return m_vertex_d2; }
    Vector const& d2() const { return m_vertex_d2; }

    Vector&       vernorm()       { return m_vertex_normal; }
    Vector const& vernorm() const { return m_vertex_normal; }

	HalfedgeDS_vertex_key(){ m_p1_calculated = false; m_p2_calculated = false;}
    HalfedgeDS_vertex_key( Point const& p) : Base(p){ m_p1_calculated = false; m_p2_calculated = false;}
};

template<class VTag>
class Polyhedron_items_for_key_region_3 {
public:
    template < class Refs, class Traits>
    struct Vertex_wrapper {
        typedef typename Traits::Point_3 Point;
		typedef typename Traits::Vector_3 Vector;
		typedef typename Traits::FT FT;
        typedef HalfedgeDS_vertex_key< Refs, Point, Vector, FT, VTag> Vertex;
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

#endif  // KEY_REGION_ITEM_POLYHEDRON
