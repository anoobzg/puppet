#pragma once
#include "PPolynomial.h"

#include "key.h"
#include "node_key.h"

class DotCaculator
{
public:
	DotCaculator(unsigned depth)
	{
		PPolynomial<2> recon_func = PPolynomial<2>::GaussianApproximation();
		recon_func.printnl();

		base_func = recon_func / recon_func(0);
		base_func.printnl();
		d_base_func = base_func.derivative();
		d_base_func.printnl();
	}

	~DotCaculator()
	{

	}

	double start()
	{
		return base_func.polys[0].start;
	}

	double end()
	{
		return base_func.polys[base_func.polyCount - 1].start;
	}

	double dotProduct(const double& c1, const double& w1, const double& c2, const double& w2) const
	{
		double r = 3.0;
		return (base_func * base_func.scale(w2 / w1).shift((c2 - c1) / w1)).integral(-r, r) * w1;
	}

	double dDotProduct(const double& c1, const double& w1, const double& c2, const double& w2) const
	{
		double r = 3.0;
		return (d_base_func * base_func.scale(w2 / w1).shift((c2 - c1) / w1)).integral(-r, r);
	}

	double d2DotProduct(const double& c1, const double& w1, const double& c2, const double& w2) const
	{
		double r = 3.0;
		return (d_base_func * d_base_func.scale(w2 / w1).shift((c2 - c1) / w1)).integral(-r, r) / w2;
	}

	void node_width_center(double& cx, double& cy, double& cz, double& w, const key& k, unsigned depth)
	{
		w = 1.0 / double(1 << depth);
		unsigned xi, yi, zi;
		key2index(k, xi, yi, zi, depth);

		double wx = 0.0;
		double wy = 0.0;
		double wz = 0.0;
		index2centerandwidth(xi, cx, wx);
		index2centerandwidth(yi, cy, wy);
		index2centerandwidth(zi, cz, wz);
	}

	double node_value(const vec3& v, double cx, double cy, double cz, double w)
	{
		return node_value(cx, w, v.x) * node_value(cy, w, v.y) * node_value(cz, w, v.z);
	}

	double node_value(const vec3& v, const key& k, unsigned depth)
	{
		double w = 1.0 / double(1 << depth);
		unsigned xi, yi, zi;
		key2index(k, xi, yi, zi, depth);

		double cx = 0.0;
		double cy = 0.0;
		double cz = 0.0;
		double wx = 0.0;
		double wy = 0.0;
		double wz = 0.0;
		index2centerandwidth(xi, cx, wx);
		index2centerandwidth(yi, cy, wy);
		index2centerandwidth(zi, cz, wz);
		double x = (double)v.x;
		double y = (double)v.y;
		double z = (double)v.z;

		return node_value(cx, wx, x) * node_value(cy, wx, y) * node_value(cz, wx, z);
	}

	double node_value(const double& c, const double& w, const double& v)
	{
		return base_func.scale(w).shift(c)(v);
	}

	PPolynomial<2> base_func;
	PPolynomial<1> d_base_func;
};