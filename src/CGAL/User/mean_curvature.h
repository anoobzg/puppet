#pragma once
#ifndef MEAN_CURVATURE_H
#define MEAN_CURVATURE_H
#include <vector>
#include "normal_calculator.h"

#include <fstream>

template <class TPoly>
class Mean_Curvature
{
	typedef typename TPoly::Traits Traits;
	typedef typename Traits::Point_3 Point;
	typedef typename Traits::Vector_3 Vector;
	typedef typename Traits::FT FT;
	typedef typename TPoly::Vertex Vertex;
	typedef typename TPoly::Vertex_iterator Vertex_iterator;
	typedef typename TPoly::Facet_iterator Facet_iterator;

	typedef typename TPoly::Vertex_handle Vertex_handle;
	typedef typename TPoly::Halfedge_handle Halfedge_handle;
	typedef typename TPoly::Halfedge_around_vertex_circulator HV_circulator;
public:
	static void Curvature(TPoly& p, std::vector<double>& curv, unsigned ver_num)
	{
		std::vector<double> voronoi_area;
		std::vector<Vector> voronoi_vector;
		voronoi_area.resize(ver_num, 0.0);
		voronoi_vector.resize(ver_num, Vector(0.0, 0.0, 0.0));

		Vertex_iterator vit = p.vertices_begin();
		Vertex_iterator vend = p.vertices_end();

		for(Facet_iterator fi = p.facets_begin(); fi != p.facets_end(); ++fi)
		{
		// angles
			Halfedge_handle h = fi->halfedge();
			Vertex_handle v0 = h->vertex();
			Vertex_handle v1 = h->next()->vertex();
			Vertex_handle v2 = h->next()->next()->vertex();

			Vector v01 = v1->point() - v0->point();
			double e01 = v01*v01;
			double len01 = sqrt(e01);
			Vector v02 = v2->point() - v0->point();
			double e20 = v02*v02;
			double len02 = sqrt(e20);
			double angle0 = std::acos(v01*v02/len01/len02);

			Vector pd = CGAL::cross_product(v01, v02);
			double A = sqrt(pd*pd);

			Vector v10 = v0->point() - v1->point();
			double len10 = sqrt(v10*v10);
			Vector v12 = v2->point() - v1->point();
			double e12 = v12*v12;
			double len12 = sqrt(e12);
			double angle1 = std::acos(v10*v12/len10/len12);
			
			Vector v20 = v0->point() - v2->point();
			double angle2 = 3.1415926 - angle0 - angle1;

			if((angle0 < 1.5707063) && (angle1 < 1.5707063) && (angle2 < 1.5707063))  // triangolo non ottuso
			{
				double area0 = ( e20*(1.0/tan(angle1)) + e01*(1.0/tan(angle2)) ) / 8.0;
				double area1 = ( e01*(1.0/tan(angle2)) + e12*(1.0/tan(angle0)) ) / 8.0;
				double area2 = ( e12*(1.0/tan(angle0)) + e20*(1.0/tan(angle1)) ) / 8.0;

				voronoi_area[v0->id()]  += area0;
				voronoi_area[v1->id()]  += area1;
				voronoi_area[v2->id()]  += area2;
			}
			else // obtuse
			{
				if(angle0 >= 1.5707063)
				{
					voronoi_area[v0->id()] += A / 4.0;
					voronoi_area[v1->id()] += A / 8.0;
					voronoi_area[v2->id()] += A / 8.0;
				}
				else if(angle1 >= 1.5707063)
				{
					voronoi_area[v0->id()] += A / 8.0;
					voronoi_area[v1->id()] += A / 4.0;
					voronoi_area[v2->id()] += A / 8.0;
				}
				else
				{
					voronoi_area[v0->id()] += A / 8.0;
					voronoi_area[v1->id()] += A / 8.0;
					voronoi_area[v2->id()] += A / 4.0;
				}
			}
		}

		for(Facet_iterator fi = p.facets_begin(); fi != p.facets_end(); ++fi)
		{
			Halfedge_handle h = fi->halfedge();
			Vertex_handle v0 = h->vertex();
			Vertex_handle v1 = h->next()->vertex();
			Vertex_handle v2 = h->next()->next()->vertex();

			Vector v01 = v1->point() - v0->point();
			double len01 = std::sqrt(v01*v01);
			Vector v02 = v2->point() - v0->point();
			double len02 = std::sqrt(v02*v02);
			double angle0 = std::acos(v01*v02/len01/len02);

			Vector v10 = v0->point() - v1->point();
			double len10 = std::sqrt(v10*v10);
			Vector v12 = v2->point() - v1->point();
			double len12 = std::sqrt(v12*v12);
			double angle1 = std::acos(v10*v12/len10/len12);
			
			Vector v20 = v0->point() - v2->point();
			double angle2 = 3.1415926 - angle0 - angle1;

			// Skip degenerate triangles.
			if(angle0 == 0 || angle1 == 0 || angle2 == 0) continue;

			double x = voronoi_vector[v0->id()].x();
			double y = voronoi_vector[v0->id()].y();
			double z = voronoi_vector[v0->id()].z();
			x += ( v20.x() * (1.0/tan(angle1)) - v01.x() * (1.0/tan(angle2)) ) / 4.0;
			y += ( v20.y() * (1.0/tan(angle1)) - v01.y() * (1.0/tan(angle2)) ) / 4.0;
			z += ( v20.z() * (1.0/tan(angle1)) - v01.z() * (1.0/tan(angle2)) ) / 4.0;
			voronoi_vector[v0->id()] = Vector(x, y, z);

			x = voronoi_vector[v1->id()].x();
			y = voronoi_vector[v1->id()].y();
			z = voronoi_vector[v1->id()].z();
			x += ( v01.x() * (1.0/tan(angle2)) - v12.x() * (1.0/tan(angle0)) ) / 4.0;
			y += ( v01.y() * (1.0/tan(angle2)) - v12.y() * (1.0/tan(angle0)) ) / 4.0;
			z += ( v01.z() * (1.0/tan(angle2)) - v12.z() * (1.0/tan(angle0)) ) / 4.0;
			voronoi_vector[v1->id()] = Vector(x, y, z);

			x = voronoi_vector[v2->id()].x();
			y = voronoi_vector[v2->id()].y();
			z = voronoi_vector[v2->id()].z();
			x += ( v12.x() * (1.0/tan(angle0)) - v20.x() * (1.0/tan(angle1)) ) / 4.0;
			y += ( v12.y() * (1.0/tan(angle0)) - v20.y() * (1.0/tan(angle1)) ) / 4.0;
			z += ( v12.z() * (1.0/tan(angle0)) - v20.z() * (1.0/tan(angle1)) ) / 4.0;
			voronoi_vector[v2->id()] = Vector(x, y, z);
		}

		for(vit = p.vertices_begin(); vit != vend; ++vit)
		{
			double voronoi = voronoi_area[vit->id()];

			if(voronoi <= std::numeric_limits<FT>::epsilon())
				curv[vit->id()] = 0.0;
			else
			{
				double r_area = 1.0 / voronoi;
				Vector v = voronoi_vector[vit->id()];
				HV_circulator hv = vit->vertex_begin();
				Vector normal_mesh = Normal_Caculator<TPoly>::CalculateNormal(hv);
				double d = (normal_mesh*v) > 0 ? 1.0 : -1.0;
				Vector result = Vector(v.x() * r_area, v.y() * r_area, v.z() * r_area);

				curv[vit->id()]  = d * std::sqrt(result * result);
			}
		}
	}
};
#endif // CGAL_IMPORTER_H