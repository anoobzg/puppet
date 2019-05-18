#pragma once
#ifndef SURF_NORMAL_CACULATOR
#define SURF_NORMAL_CACULATOR
#include <vector>

template<class TPoly>
class Normal_Caculator
{
public:
	typedef typename TPoly::Traits::Vector_3 Vector;
	typedef typename TPoly::Traits::Point_3 Point;
	typedef typename TPoly::Facet_handle Facet_handle;
	typedef typename TPoly::Halfedge_around_facet_circulator HF_circulator;
	typedef typename TPoly::Halfedge_around_vertex_circulator HV_circulator;
public:
	static Vector CalculateNormal(HF_circulator& hf)
	{
		Vector n(0.0, 0.0, 0.0);
		HF_circulator hf_ = hf;

		Point v[3];
		unsigned i = 0;
		do {
			if (i >= 3)
				break;
			v[i] = hf_->vertex()->point();
			++i;
		} while (++hf_ != hf);

		n = CGAL::cross_product((v[1] - v[0]) , (v[2] - v[0]));
		n = (n == Vector(0, 0, 0)) ? n : n / std::sqrt(n * n);

		return n;
	}

	static Vector CalculateNormal(HV_circulator& hv)
	{
		Vector n(0.0, 0.0, 0.0);
		HV_circulator hv_ = hv;

		std::vector<Vector> normals;

		do {
			if (!hv_->is_border_edge())
			{
				HF_circulator hf = hv_->face()->facet_begin();
				Vector normal = CalculateNormal(hf);
				normals.push_back(normal);
			}
		} while (++hv_ != hv);

		for (unsigned i = 0; i < normals.size(); ++i)
		{
			n = n + normals[i];
		}

		n = (n == Vector(0, 0, 0)) ? n : n / std::sqrt(n * n);
		return n;
	}
};
#endif // SURF_NORMAL_CACULATOR