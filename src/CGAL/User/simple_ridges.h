#ifndef SIMPLE_RIDGE_3_H_
#define SIMPLE_RIDGE_3_H_

#include <utility>
#include <list>
#include <map>

#include <CGAL/basic.h>
#include <CGAL/Optimisation_d_traits_3.h>
#include <CGAL/barycenter.h>
#include <CGAL/property_map.h>
#include <CGAL/assertions.h>
#include <boost/type_traits/is_same.hpp>
#include <boost/graph/graph_traits.hpp>
#include <CGAL/boost/graph/helpers.h>

namespace Simple {
 
enum Ridge_interrogation_type {MAX_RIDGE, MIN_RIDGE, CREST_RIDGE};

enum Ridge_type {NO_RIDGE=0, 
		 MAX_ELLIPTIC_RIDGE, MAX_HYPERBOLIC_RIDGE, MAX_CREST_RIDGE, 
		 MIN_ELLIPTIC_RIDGE, MIN_HYPERBOLIC_RIDGE, MIN_CREST_RIDGE};
  
template < class TriangleMesh > class Ridge_line
{
  typedef typename boost::property_map<TriangleMesh,CGAL::vertex_point_t>::type VPM;
  typedef typename boost::property_traits<VPM>::value_type Point_3;
  typedef typename CGAL::Kernel_traits<Point_3>::Kernel Kernel; 
public:
  
  typedef typename Kernel::FT         FT;
  typedef typename boost::graph_traits<TriangleMesh>::halfedge_descriptor halfedge_descriptor;
  typedef std::pair< halfedge_descriptor, FT> ridge_halfhedge; 

  Ridge_type line_type() const {return m_line_type;}
  Ridge_type& line_type() {return m_line_type;}

  const FT strength() const {return m_strength;}
  FT& strength() {return m_strength;}

  const std::list<ridge_halfhedge>* line() const { return &m_line;}
  std::list<ridge_halfhedge>* line() { return &m_line;}

  //constructor
  Ridge_line(const TriangleMesh& P);
  
protected:
  const TriangleMesh& P;
  //one of MAX_ELLIPTIC_RIDGE, MAX_HYPERBOLIC_RIDGE, MAX_CREST_RIDGE,
  //MIN_ELLIPTIC_RIDGE, MIN_HYPERBOLIC_RIDGE or MIN_CREST_RIDGE
  Ridge_type m_line_type;  
  std::list<ridge_halfhedge> m_line;
  FT m_strength;// = integral of ppal curvature along the line
};

//--------------------------------------------------------------------------
// IMPLEMENTATION OF Ridge_line members
//--------------------------------------------------------------------------

 //constructor
template < class TriangleMesh >
Ridge_line<TriangleMesh>::
Ridge_line(const TriangleMesh& P) 
  : P(P), m_strength(0.)
{}

//---------------------------------------------------------------------------
//Ridge_approximation
//--------------------------------------------------------------------------
template < class TriangleMesh> 
class Ridge_approximation
{
  typedef typename boost::property_map<TriangleMesh,CGAL::vertex_point_t>::const_type VPM;
  typedef typename boost::property_traits<VPM>::value_type Point_3;
  typedef typename CGAL::Kernel_traits<Point_3>::Kernel Kernel;
 public:  
  typedef typename Kernel::FT        FT;
  typedef typename Kernel::Vector_3  Vector_3;
  typedef typename boost::graph_traits<TriangleMesh>::vertex_descriptor     vertex_descriptor;
  typedef typename boost::graph_traits<TriangleMesh>::halfedge_descriptor   halfedge_descriptor;
  typedef typename boost::graph_traits<TriangleMesh>::face_descriptor      face_descriptor;
  typedef typename boost::graph_traits<TriangleMesh>::face_iterator    face_iterator;
  typedef typename TriangleMesh::Face Face;

  typedef std::pair< halfedge_descriptor, FT>    Ridge_halfhedge;
  typedef Ridge_line<TriangleMesh>  Ridge_line;

  Ridge_approximation(const TriangleMesh &P);
  
  template <class OutputIterator>
  OutputIterator compute_ridges(Ridge_interrogation_type r_type, 
			  OutputIterator ridge_lines_it);

 protected:
  const TriangleMesh& P;

  typedef std::map<face_descriptor, bool> Facet2bool_map_type;
  Facet2bool_map_type is_visited_map;

  VPM vpm;

  Ridge_type facet_ridge_type(const face_descriptor f, 
			      halfedge_descriptor& he1, 
			      halfedge_descriptor& he2,
			      Ridge_interrogation_type r_type);
  
  void xing_on_edge(const halfedge_descriptor he, 
		    bool& is_crossed, 
		    Ridge_interrogation_type color);
 
  bool tag_as_elliptic_hyperbolic(const Ridge_interrogation_type color,
				  const halfedge_descriptor he1, 
				  const halfedge_descriptor he2);

  int b_sign_pointing_to_ridge(const vertex_descriptor v1, 
			       const vertex_descriptor v2,
			       const vertex_descriptor v3,
			       const Vector_3 r1, const Vector_3 r2, 
			       const Ridge_interrogation_type color);

  //a ridge line begins with a segment in a triangle given by the 2 he
  //crossed
  void init_ridge_line(Ridge_line* ridge_line, 
		       const halfedge_descriptor h1, 
		       const halfedge_descriptor h2, 
		       const Ridge_type r_type);

  void addback(Ridge_line* ridge_line, 
	       const halfedge_descriptor he, 
	       const Ridge_type r_type);
  void addfront(Ridge_line* ridge_line, 
		const halfedge_descriptor he,
		const Ridge_type r_type);

  FT bary_coord(const halfedge_descriptor he, 
		const Ridge_type r_type);
};

template < class TriangleMesh> 
Ridge_approximation< TriangleMesh>::Ridge_approximation(const TriangleMesh &p)
  : P(p),vpm(get(CGAL::vertex_point,p))
{
  //init the is_visited_map and check that the mesh is a triangular one.
  face_iterator itb,ite; 
  boost::tie(itb,ite) = faces(P);
  for(;itb!=ite;itb++) {
    is_visited_map[*itb] = false;
  }
  CGAL_precondition( is_triangle_mesh(p) );
}

template < class TriangleMesh> 
template <class OutputIterator>
OutputIterator Ridge_approximation<TriangleMesh>::compute_ridges(Ridge_interrogation_type r_type, OutputIterator ridge_lines_it)
{
  //reinit the is_visited_map
  face_iterator itb,ite;
  boost::tie(itb,ite) = faces(P);
  for(;itb!=ite;itb++)
    {
      face_descriptor f = *itb;
      if (is_visited_map.find(f)->second) continue;
      is_visited_map.find(f)->second = true;
      halfedge_descriptor h1, h2, curhe1, curhe2, curhe;
      
      //h1 h2 are the hedges crossed if any, r_type should be
      //MAX_RIDGE, MIN_RIDGE or CREST_RIDGE ; cur_ridge_type should be
      //MAX_ELLIPTIC_RIDGE, MAX_HYPERBOLIC_RIDGE, MAX_CREST_RIDGE,
      //MIN_ELLIPTIC_RIDGE, MIN_HYPERBOLIC_RIDGE, MIN_CREST_RIDGE or NO_RIDGE
      Ridge_type cur_ridge_type = facet_ridge_type(f,h1,h2,r_type);
      if ( cur_ridge_type == NO_RIDGE ) continue;
      
      //a ridge_line is begining and stored
      Ridge_line* cur_ridge_line = new Ridge_line(P);
      init_ridge_line(cur_ridge_line, h1, h2, cur_ridge_type);
      *ridge_lines_it++ = cur_ridge_line;
    
      //next triangle adjacent to h1 (push_front)
      if ( ! is_border_edge(h1,P) ) 
	{
	  f = face(opposite(h1,P),P);
	  curhe = h1;
	  while (cur_ridge_type == facet_ridge_type(f,curhe1,curhe2,r_type))
	    {
	      //follow the ridge from curhe
	      if (is_visited_map.find(f)->second) break;
	      is_visited_map.find(f)->second = true;
	      if (opposite(curhe,P) == curhe1) curhe = curhe2;
	      else curhe = curhe1;//curhe stays at the ridge extremity
	      addfront(cur_ridge_line, curhe, cur_ridge_type);
	      if ( ! is_border_edge(curhe,P) ) f = face(opposite(curhe,P),P);
	      else break;
	    }
	  //exit from the while if
	  //1. border or already visited (this is a ridge loop)
	  //2. not same type, then do not set visisted cause a MAX_ELLIPTIC_RIDGE
	  //	  follows a MAX_HYPERBOLIC_RIDGE
	}

      //next triangle adjacent to h2 (push_back)
      if ( ! is_border_edge(h2,P) ) 
	{
	  f = face(opposite(h2,P),P);
	  curhe = h2;
	  while (cur_ridge_type ==
		 facet_ridge_type(f,curhe1,curhe2,r_type))
	    {
	      //follow the ridge from curhe
	      if (is_visited_map.find(f)->second) break;
	      is_visited_map.find(f)->second = true;
	      if (opposite(curhe,P) == curhe1) curhe = curhe2;
	      else curhe = curhe1;
	      addback(cur_ridge_line, curhe, cur_ridge_type);
	      if ( ! is_border_edge(curhe,P) ) f = face(opposite(curhe,P),P);
	      else break;
	    }
	} 
    }
  return ridge_lines_it;
}


template < class TriangleMesh> 
Ridge_type Ridge_approximation< TriangleMesh>::facet_ridge_type(const face_descriptor f, halfedge_descriptor& he1, halfedge_descriptor&
		 he2, Ridge_interrogation_type r_type)
{
  //polyhedral data
  //we have v1--h1-->v2--h2-->v3--h3-->v1
  const halfedge_descriptor h1 = halfedge(f,P);
  const vertex_descriptor v2 = target(h1,P);
  const halfedge_descriptor h2 = next(h1,P);
  const vertex_descriptor v3 = target(h2,P);
  const halfedge_descriptor h3 = next(h2,P);
  const vertex_descriptor v1 = target(h3,P);

  //check for regular facet
  //i.e. if there is a coherent orientation of ppal dir at the facet vertices
  if ( v1->d1()*v2->d1() * v1->d1()*v3->d1() * v2->d1()*v3->d1() < 0 ) 
    return NO_RIDGE;
   
  //determine potential crest color
  //MAX_CREST_RIDGE if |sum(k1)|>|sum(k2)| sum over facet vertices vi
  //MIN_CREST_RIDGE if |sum(k1)|<|sum(k2)|
  Ridge_type crest_color = NO_RIDGE;
  if (r_type == CREST_RIDGE) 
    {
      if ( CGAL::abs(v1->k1()+v2->k1()+v3->k1()) > CGAL::abs(v1->k2()+v2->k2()+v3->k2()) ) 
	crest_color = MAX_CREST_RIDGE; 
      if ( CGAL::abs(v1->k1()+v2->k1()+v3->k1()) < CGAL::abs(v1->k2()+v2->k2()+v3->k2()) ) 
	crest_color = MIN_CREST_RIDGE;
      if ( CGAL::abs(v1->k1()+v2->k1()+v3->k1()) == CGAL::abs(v1->k2()+v2->k2()+v3->k2()) ) 
	return NO_RIDGE;
    }
  
  //compute Xing on the 3 edges
  bool h1_is_crossed = false, h2_is_crossed = false, h3_is_crossed = false;
  if ( r_type == MAX_RIDGE || crest_color == MAX_CREST_RIDGE ) 
    {
      xing_on_edge(h1, h1_is_crossed, MAX_RIDGE);
      xing_on_edge(h2, h2_is_crossed, MAX_RIDGE);
      xing_on_edge(h3, h3_is_crossed, MAX_RIDGE);
    }
  if ( r_type == MIN_RIDGE || crest_color == MIN_CREST_RIDGE ) 
    {
      xing_on_edge(h1, h1_is_crossed, MIN_RIDGE);
      xing_on_edge(h2, h2_is_crossed, MIN_RIDGE);
      xing_on_edge(h3, h3_is_crossed, MIN_RIDGE);
    }

  //there are either 0 or 2 crossed edges
  if ( !h1_is_crossed && !h2_is_crossed && !h3_is_crossed ) 
    return NO_RIDGE; 
  if (h1_is_crossed && h2_is_crossed && !h3_is_crossed)
    {
      he1 = h1; 
      he2 = h2;
    }
  if (h1_is_crossed && !h2_is_crossed && h3_is_crossed)
    {
      he1 = h1; 
      he2 = h3;
    }
  if (!h1_is_crossed && h2_is_crossed && h3_is_crossed)
    {
      he1 = h2; 
      he2 = h3;
    }
  //check there is no other case (just one edge crossed)
  CGAL_postcondition ( !( (h1_is_crossed && !h2_is_crossed && !h3_is_crossed)
			  || (!h1_is_crossed && h2_is_crossed && !h3_is_crossed)
			  || (!h1_is_crossed && !h2_is_crossed && h3_is_crossed)) );

  //There is a ridge segment in the triangle, determine its type elliptic/hyperbolic
  bool is_elliptic;  
  if ( r_type == MAX_RIDGE || crest_color == MAX_CREST_RIDGE ) 
    is_elliptic = tag_as_elliptic_hyperbolic(MAX_RIDGE, he1, he2);
  else is_elliptic = tag_as_elliptic_hyperbolic(MIN_RIDGE, he1, he2);
  
  if (r_type == MAX_RIDGE) 
    {if (is_elliptic) return MAX_ELLIPTIC_RIDGE;
    else return MAX_HYPERBOLIC_RIDGE; }
  if (crest_color == MAX_CREST_RIDGE && is_elliptic) return MAX_CREST_RIDGE;

  if (r_type == MIN_RIDGE) 
    {if (is_elliptic) return MIN_ELLIPTIC_RIDGE;
    else return MIN_HYPERBOLIC_RIDGE; }
  if (crest_color == MIN_CREST_RIDGE && is_elliptic) return MIN_CREST_RIDGE;
  
  return NO_RIDGE;
}

template < class TriangleMesh > 
void Ridge_approximation< TriangleMesh>::xing_on_edge(const halfedge_descriptor he, bool& is_crossed, Ridge_interrogation_type color)
{
  is_crossed = false;
  FT sign = 0;
  FT b_p, b_q; // extremalities at p and q for he: p->q
  Vector_3  d_p = target(opposite(he,P),P)->d1(),
    d_q = target(he,P)->d1(); //ppal dir
  if ( color == MAX_RIDGE ) {
    b_p = target(opposite(he,P),P)->b0();
    b_q = target(he,P)->b0();
  }
  else {     
    b_p = target(opposite(he,P),P)->b3();
    b_q = target(he,P)->b3();
  }
  if ( b_p == 0 && b_q == 0 ) return;
  if ( b_p == 0 && b_q !=0 ) sign = d_p*d_q * b_q;
  if ( b_p != 0 && b_q ==0 ) sign = d_p*d_q * b_p;
  if ( b_p != 0 && b_q !=0 ) sign = d_p*d_q * b_p * b_q;
  if ( sign < 0 ) is_crossed = true;
}


template < class TriangleMesh> 
bool Ridge_approximation< TriangleMesh>::tag_as_elliptic_hyperbolic(const Ridge_interrogation_type color,
                           const halfedge_descriptor he1, 
                           const halfedge_descriptor he2)
{
  const vertex_descriptor v_p1 = target(opposite(he1,P),P), v_q1 = target(he1,P),
    v_p2 = target(opposite(he2,P),P), v_q2 = target(he2,P); // hei: pi->qi

  FT coord1, coord2;
  if (color == MAX_RIDGE) 
    {
      coord1 = CGAL::abs(v_q1->b0()) / ( CGAL::abs(v_p1->b0()) + CGAL::abs(v_q1->b0()) );
      coord2 = CGAL::abs(v_q2->b0()) / ( CGAL::abs(v_p2->b0()) + CGAL::abs(v_q2->b0()) ); 
    }
  else 
    {
      coord1 = CGAL::abs(v_q1->b3()) / ( CGAL::abs(v_p1->b3()) + CGAL::abs(v_q1->b3()) );
      coord2 = CGAL::abs(v_q2->b3()) / ( CGAL::abs(v_p2->b3()) + CGAL::abs(v_q2->b3()) ); 
    }

    FT sign_P;
    if (color == MAX_RIDGE) 
      sign_P =  v_p1->P1()*coord1 + v_q1->P1()*(1-coord1) 
	+ v_p2->P1()*coord2 + v_q2->P1()*(1-coord2);
    else sign_P =  v_p1->P2()*coord1 + v_q1->P2()*(1-coord1) 
	+ v_p2->P2()*coord2 + v_q2->P2()*(1-coord2);

    if ( sign_P < 0 ) return true; else return false;
}


template < class TriangleMesh> 
int Ridge_approximation< TriangleMesh>::b_sign_pointing_to_ridge(const vertex_descriptor v1, 
                         const vertex_descriptor v2,
                         const vertex_descriptor v3,
                         const Vector_3 r1, const Vector_3 r2, 
                         const Ridge_interrogation_type color)
{
  Vector_3 r = r2 - r1, dv1, dv2, dv3;
  FT bv1, bv2, bv3;
  if ( color == MAX_RIDGE ) {
    bv1 = v1->b0();
    bv2 = v2->b0();
    bv3 = v3->b0();
    dv1 = v1->d1();
    dv2 = v2->d1();
    dv3 = v3->d1();
  }
  else {
    bv1 = v1->b3();
    bv2 = v2->b3();
    bv3 = v3->b3();
    dv1 = d2[v1];
    dv2 = d2[v2];
    dv3 = d2[v3];    
  }
  if ( r != CGAL::NULL_VECTOR ) r = r/CGAL::sqrt(r*r);
  FT sign1, sign2, sign3;
  sign1 = bv1*(r1 - (get(vpm, v1)-CGAL::ORIGIN) + (((get(vpm, v1)-CGAL::ORIGIN)-r1)*r)*r )*dv1;
  sign2 = bv2*(r1 - (get(vpm, v2)-CGAL::ORIGIN) + (((get(vpm, v2)-CGAL::ORIGIN)-r1)*r)*r )*dv2;
  sign3 = bv3*(r1 - (get(vpm, v3)-CGAL::ORIGIN) + (((get(vpm, v3)-CGAL::ORIGIN)-r1)*r)*r )*dv3;
  
  int compt = 0;
  if ( sign1 > 0 ) compt++; else if (sign1 < 0) compt--;
  if ( sign2 > 0 ) compt++; else if (sign2 < 0) compt--;
  if ( sign3 > 0 ) compt++; else if (sign3 < 0) compt--;
  
  if (compt > 0) return 1; else return -1;
}


template < class TriangleMesh> 
void Ridge_approximation< TriangleMesh>::init_ridge_line(Ridge_line* ridge_line, 
		const halfedge_descriptor h1, 
		const halfedge_descriptor h2, 
		const Ridge_type r_type)
{
  ridge_line->line_type() = r_type;
  ridge_line->line()->push_back(Ridge_halfhedge(h1, bary_coord(h1,r_type)));
  addback(ridge_line, h2, r_type);
}


template < class TriangleMesh> 
void Ridge_approximation< TriangleMesh>::addback(Ridge_line* ridge_line, const halfedge_descriptor he, const Ridge_type r_type)
{
  halfedge_descriptor he_cur = ( --(ridge_line->line()->end()) )->first;
  FT coord_cur = ( --(ridge_line->line()->end()) )->second;//bary_coord(he_cur);
  FT coord = bary_coord(he,r_type);
  vertex_descriptor v_p = target(opposite(he,P),P), v_q = target(he,P),
    v_p_cur = target(opposite(he_cur,P),P), v_q_cur = target(he_cur,P); // he: p->q
  Vector_3 segment = CGAL::barycenter(get(vpm, v_p), coord, get(vpm, v_q)) -
                     CGAL::barycenter(get(vpm, v_p_cur), coord_cur, get(vpm, v_q_cur));

  FT k1x, k2x; //abs value of the ppal curvatures at the Xing point on he.

  k1x = CGAL::abs(v_p->k1()) * coord + CGAL::abs(v_q->k1()) * (1-coord) ;   
  k2x = CGAL::abs(v_p->k2()) * coord + CGAL::abs(v_q->k2()) * (1-coord) ;   

  if ( (ridge_line->line_type() == MAX_ELLIPTIC_RIDGE) 
       || (ridge_line->line_type() == MAX_HYPERBOLIC_RIDGE) 
       || (ridge_line->line_type() == MAX_CREST_RIDGE) ) {
    ridge_line->strength() += k1x * CGAL::sqrt(segment * segment); 
  }
  if ( (ridge_line->line_type() == MIN_ELLIPTIC_RIDGE) 
       || (ridge_line->line_type() == MIN_HYPERBOLIC_RIDGE) 
       || (ridge_line->line_type() == MIN_CREST_RIDGE) ) {
   ridge_line->strength() += k2x * CGAL::sqrt(segment * segment); 
   } 
  ridge_line->line()->push_back( Ridge_halfhedge(he, coord));
}


template < class TriangleMesh> 
void Ridge_approximation< TriangleMesh>::addfront(Ridge_line* ridge_line, 
	 const halfedge_descriptor he, 
	 const Ridge_type r_type)
{
  halfedge_descriptor he_cur = ( ridge_line->line()->begin() )->first;
  FT coord_cur = ( ridge_line->line()->begin() )->second;
  FT coord = bary_coord(he,r_type);
  vertex_descriptor v_p = target(opposite(he,P),P), v_q = target(he,P),
    v_p_cur = target(opposite(he_cur,P),P), v_q_cur = target(he_cur,P); // he: p->q
  Vector_3 segment = CGAL::barycenter(get(vpm, v_p), coord, get(vpm, v_q)) -
                     CGAL::barycenter(get(vpm, v_p_cur), coord_cur, get(vpm, v_q_cur));

  FT k1x, k2x; //abs value of the ppal curvatures at the Xing point on he.

  k1x = CGAL::abs(v_p->k1()) * coord + CGAL::abs(v_q->k1()) * (1-coord) ;   
  k2x = CGAL::abs(v_p->k2()) * coord + CGAL::abs(v_q->k2()) * (1-coord) ;   

  if ( (ridge_line->line_type() == MAX_ELLIPTIC_RIDGE) 
       || (ridge_line->line_type() == MAX_HYPERBOLIC_RIDGE) 
       || (ridge_line->line_type() == MAX_CREST_RIDGE) ) {
    ridge_line->strength() += k1x * CGAL::sqrt(segment * segment); 
  }
  if ( (ridge_line->line_type() == MIN_ELLIPTIC_RIDGE) 
       || (ridge_line->line_type() == MIN_HYPERBOLIC_RIDGE) 
       || (ridge_line->line_type() == MIN_CREST_RIDGE) ) {
   ridge_line->strength() += k2x * CGAL::sqrt(segment * segment); 
   } 
  ridge_line->line()->push_front( Ridge_halfhedge(he, coord));
}


template < class TriangleMesh> 
typename Ridge_approximation<TriangleMesh>::FT Ridge_approximation<TriangleMesh>::bary_coord(const halfedge_descriptor he, const Ridge_type r_type)
{
  FT b_p = 0., b_q = 0.; // extremalities at p and q for he: p->q
  if ( (r_type == MAX_ELLIPTIC_RIDGE) 
       || (r_type == MAX_HYPERBOLIC_RIDGE) 
       || (r_type == MAX_CREST_RIDGE) ) {
    b_p = target(opposite(he,P),P)->b0();
    b_q = target(he,P)->b0();
  }
  if ( (r_type == MIN_ELLIPTIC_RIDGE) 
       || (r_type == MIN_HYPERBOLIC_RIDGE) 
       || (r_type == MIN_CREST_RIDGE) ) {
    b_p = target(opposite(he,P),P)->b3();
    b_q = target(he,P)->b3();    
  }
  //denominator cannot be 0 since there is no crossing when both extremalities are 0
  return CGAL::abs(b_q) / ( CGAL::abs(b_q) + CGAL::abs(b_p) );
}

} //namespace CGAL

#endif // SIMPLE_RIDGE_3_H_
