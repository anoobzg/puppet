#pragma once
#ifndef LAPLACIAN_SMOOTH_H
#define LAPLACIAN_SMOOTH_H
#include <algorithm>
#include "parralle_rings.h"

#include "tag.h"

#include <ppl.h>
template<class TPoly>
class Laplacian_Smooth
{
private:
	typedef typename TPoly::Traits Traits;
	typedef typename Traits::Point_3 Point;
	typedef typename Traits::Vector_3 Vector;
	typedef typename Traits::FT FT;
	typedef typename TPoly::Vertex Vertex;
	typedef typename TPoly::Vertex_iterator Vertex_iterator;
	typedef typename TPoly::Halfedge_handle Halfedge_handle;

	typedef typename T_Parralle_rings<TPoly> Poly_rings;
	typedef typename boost::graph_traits<TPoly>::vertex_descriptor  vertex_descriptor;
	typedef typename boost::graph_traits<TPoly>::vertex_iterator vertex_iterator;
private:
	class LaplacianInfo
	{
	public:
		LaplacianInfo(const Point &_p, const FT _n) :sum(_p), cnt(_n) {}
		LaplacianInfo() {}
		Point sum;
		FT cnt;
	};
	
	void Smooth_impl(TPoly& p, std::vector<Point>& position, bool fix_boarder, FT max_off)
	{
		for (Vertex_iterator vi = p.vertices_begin(); vi != p.vertices_end(); ++vi)
		{
			size_t id = vi->id();
			Point pp = vi->point();
			Point in_points(0.0, 0.0, 0.0);
			size_t size = 0;

			Halfedge_handle h = vi->halfedge();
			Halfedge_handle hend = h;
			bool is_boarder = false;
			do {
				if (h->is_border())
					is_boarder = true;
				Point vp = h->opposite()->vertex()->point();
				
				in_points = Point(in_points.x()+vp.x(), in_points.y()+vp.y(), in_points.z()+vp.z());
				++size;
				h = h->next()->opposite();
			} while (h != hend);

			if ((fix_boarder && is_boarder) || size == 0)
			{
				position[id] = pp;
				continue;
			}
			const FT key_region_coff = keyregion_smooth_coff[vi->tag()];
			const FT no_key_coff = (1.0 - key_region_coff)/(FT)size;
			position[id] = Point(in_points.x()*no_key_coff+pp.x()*key_region_coff,
				in_points.y()*no_key_coff+pp.y()*key_region_coff,
				in_points.z()*no_key_coff+pp.z()*key_region_coff);
			//if(vi->tag() == e_key_region)
			//{
			//	const FT no_key_coff = (1.0 - key_region_coff)/(FT)size;
			//	Point target = Point(in_points.x()*no_key_coff+pp.x()*key_region_coff,
			//		in_points.y()*no_key_coff+pp.y()*key_region_coff,
			//		in_points.z()*no_key_coff+pp.z()*key_region_coff);

			//	//Vector d = target - pp;
			//	//FT delta = std::sqrt(d * d);
			//	//if(false/*delta > max_off*/)
			//	//{
			//	//	const FT coff = max_off / delta;
			//	//	const FT ncoff = 1.0 - coff;
			//	//	position[id] = Point(target.x() * coff + pp.x() * ncoff,
			//	//		target.y() * coff + pp.y() * ncoff,
			//	//		target.z() * coff + pp.z() * ncoff);
			//	//}else
			//	//{
			//	//	position[id] = target;
			//	}
			//}else{
			//	FT div = (FT)(size+1);
			//	position[id] = Point((in_points.x()+pp.x())/div, (in_points.y()+pp.y())/div, (in_points.z()+pp.z())/div);
			//}
		}
	}

	void AccumulateLaplacianInfo(TPoly& p, std::vector<LaplacianInfo>& infos)
	{
		FT weight = 1.0;

		for (Vertex_iterator vi = p.vertices_begin(); vi != p.vertices_end(); ++vi)
		{
			infos[vi->id()].sum = vi->point();
			infos[vi->id()].cnt = 0.0;
			if(vi->tag() == e_key_region)
				continue;

			Point sum(0.0, 0.0, 0.0);
			FT cnt = 0.0;
			Halfedge_handle h = vi->halfedge();
			Halfedge_handle hend = h;
			bool is_boarder = false;
			do {
				if (h->is_border())
				{
					is_boarder = true;
					break;
				}
				Point vp = h->opposite()->vertex()->point();
				sum = Point(sum.x() + vp.x()*weight, sum.y() + vp.y()*weight, sum.z() + vp.z()*weight);
				cnt += weight;
				h = h->next()->opposite();
			} while (h != hend);

			if (is_boarder)
			{
				continue;
			}
			infos[vi->id()].sum = sum;
			infos[vi->id()].cnt = cnt;
		}
	}

	void Smooth_impl_parrale(TPoly& p, std::vector<Point>& position)
	{
		Concurrency::parallel_for_each(p.vertices_begin(), p.vertices_end(), [&p, &position, this](Vertex& vertex) {
			size_t id = vertex.id();
			if(vertex.tag() == e_key_region)
			{
				position[id] = vertex.point();
			}else
			{
				Halfedge_handle h = vertex.halfedge();
				Halfedge_handle hend = h;
				bool is_boarder = false;
				do {
					if (h->is_border())
					{
						is_boarder = true;
						break;
					}
					h = h->next()->opposite();
				} while (h != hend);

				if (is_boarder)
				{
					position[id] = vertex.point();
				}else
				{
					Poly_rings poly_rings(p);
					std::vector<Point> in_points;
					gather_fitting_points(&vertex, in_points, poly_rings, 1);
					size_t size = in_points.size();
					if(size > 0)
					{
						FT sx = 0.0, sy = 0.0, sz = 0.0;
						for(unsigned i = 0; i < size; ++i)
						{
							sx = sx + in_points[i].x();
							sy = sy + in_points[i].y();
							sz = sz + in_points[i].z();
						}
						sx = sx / (FT)size;
						sy = sy / (FT)size;
						sz = sz / (FT)size;
						Point sum(sx, sy, sz);
						position[id] = sum;
					}
				}
			}
		});
	}

	void gather_fitting_points(vertex_descriptor v, std::vector<Point> &in_points, Poly_rings& poly_rings, int nb_rings)
	{
		//container to collect vertices of v on the PolyhedralSurf
		std::vector<vertex_descriptor> gathered;
		//initialize
		in_points.clear();

		poly_rings.collect_i_rings(v, nb_rings, gathered);
		//store the gathered points
		std::vector<vertex_descriptor>::const_iterator
			itb = gathered.begin(), ite = gathered.end();
		CGAL_For_all(itb, ite) in_points.push_back((*itb)->point());
	}
public:
	void Smooth_Parralle(TPoly& p, size_t ver_num, unsigned step)
	{
		std::vector<Point> save_point(ver_num);

		for (unsigned i = 0; i < step; ++i)
		{
			Smooth_impl_parrale(p, save_point);
			Concurrency::parallel_for_each(p.vertices_begin(), p.vertices_end(), [&save_point](Vertex& vertex) {
				size_t id = vertex.id();
				vertex.point() = save_point[id];
			});
		}
	}

	void Smooth(TPoly& p, size_t ver_num, unsigned step, FT max_off)
	{
		std::vector<Point> save_point(ver_num);
		for (unsigned i = 0; i < step; ++i)
		{
			Smooth_impl(p, save_point, true, max_off);
			for(Vertex_iterator it = p.vertices_begin(); it != p.vertices_end(); ++it)
			{
				size_t id = (*it).id();
				(*it).point() = save_point[id];
			}
		}
	}
};

#endif // LAPLACIAN_SMOOTH_H