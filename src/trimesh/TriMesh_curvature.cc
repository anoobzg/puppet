/*
Szymon Rusinkiewicz
Princeton University

TriMesh_curvature.cc
Computation of per-vertex principal curvatures and directions.

Uses algorithm from
 Rusinkiewicz, Szymon.
 "Estimating Curvatures and Their Derivatives on Triangle Meshes,"
 Proc. 3DPVT, 2004.
*/

#include "TriMesh.h"
#include "TriMesh_algo.h"
#include "lineqn.h"
using namespace std;


namespace trimesh {

// Rotate a coordinate system to be perpendicular to the given normal
static void rot_coord_sys(const vec &old_u, const vec &old_v,
                          const vec &new_norm,
                          vec &new_u, vec &new_v)
{
	new_u = old_u;
	new_v = old_v;
	vec old_norm = old_u CROSS old_v;
	float ndot = old_norm DOT new_norm;
	if (unlikely(ndot <= -1.0f)) {
		new_u = -new_u;
		new_v = -new_v;
		return;
	}

	// Perpendicular to old_norm and in the plane of old_norm and new_norm
	vec perp_old = new_norm - ndot * old_norm;

	// Perpendicular to new_norm and in the plane of old_norm and new_norm
	// vec perp_new = ndot * new_norm - old_norm;

	// perp_old - perp_new, with normalization constants folded in
	vec dperp = 1.0f / (1 + ndot) * (old_norm + new_norm);

	// Subtracts component along perp_old, and adds the same amount along
	// perp_new.  Leaves unchanged the component perpendicular to the
	// plane containing old_norm and new_norm.
	new_u -= dperp * (new_u DOT perp_old);
	new_v -= dperp * (new_v DOT perp_old);
}


// Reproject a curvature tensor from the basis spanned by old_u and old_v
// (which are assumed to be unit-length and perpendicular) to the
// new_u, new_v basis.
void proj_curv(const vec &old_u, const vec &old_v,
               float old_ku, float old_kuv, float old_kv,
               const vec &new_u, const vec &new_v,
               float &new_ku, float &new_kuv, float &new_kv)
{
	vec r_new_u, r_new_v;
	rot_coord_sys(new_u, new_v, old_u CROSS old_v, r_new_u, r_new_v);

	float u1 = r_new_u DOT old_u;
	float v1 = r_new_u DOT old_v;
	float u2 = r_new_v DOT old_u;
	float v2 = r_new_v DOT old_v;
	new_ku  = old_ku * u1*u1 + old_kuv * (2.0f  * u1*v1) + old_kv * v1*v1;
	new_kuv = old_ku * u1*u2 + old_kuv * (u1*v2 + u2*v1) + old_kv * v1*v2;
	new_kv  = old_ku * u2*u2 + old_kuv * (2.0f  * u2*v2) + old_kv * v2*v2;
}


// Like the above, but for dcurv
void proj_dcurv(const vec &old_u, const vec &old_v,
		const Vec<4> old_dcurv,
		const vec &new_u, const vec &new_v,
		Vec<4> &new_dcurv)
{
	vec r_new_u, r_new_v;
	rot_coord_sys(new_u, new_v, old_u CROSS old_v, r_new_u, r_new_v);

	float u1 = r_new_u DOT old_u;
	float v1 = r_new_u DOT old_v;
	float u2 = r_new_v DOT old_u;
	float v2 = r_new_v DOT old_v;

	new_dcurv[0] = old_dcurv[0]*u1*u1*u1 +
	               old_dcurv[1]*3.0f*u1*u1*v1 +
	               old_dcurv[2]*3.0f*u1*v1*v1 +
	               old_dcurv[3]*v1*v1*v1;
	new_dcurv[1] = old_dcurv[0]*u1*u1*u2 +
	               old_dcurv[1]*(u1*u1*v2 + 2.0f*u2*u1*v1) +
	               old_dcurv[2]*(u2*v1*v1 + 2.0f*u1*v1*v2) +
	               old_dcurv[3]*v1*v1*v2;
	new_dcurv[2] = old_dcurv[0]*u1*u2*u2 +
	               old_dcurv[1]*(u2*u2*v1 + 2.0f*u1*u2*v2) +
	               old_dcurv[2]*(u1*v2*v2 + 2.0f*u2*v2*v1) +
	               old_dcurv[3]*v1*v2*v2;
	new_dcurv[3] = old_dcurv[0]*u2*u2*u2 +
	               old_dcurv[1]*3.0f*u2*u2*v2 +
	               old_dcurv[2]*3.0f*u2*v2*v2 +
	               old_dcurv[3]*v2*v2*v2;
}


// Given a curvature tensor, find principal directions and curvatures
// Makes sure that pdir1 and pdir2 are perpendicular to normal
void diagonalize_curv(const vec &old_u, const vec &old_v,
                      float ku, float kuv, float kv,
                      const vec &new_norm,
                      vec &pdir1, vec &pdir2, float &k1, float &k2)
{
	vec r_old_u, r_old_v;
	rot_coord_sys(old_u, old_v, new_norm, r_old_u, r_old_v);

	float c = 1, s = 0, tt = 0;
	if (likely(kuv != 0.0f)) {
		// Jacobi rotation to diagonalize
		float h = 0.5f * (kv - ku) / kuv;
		tt = (h < 0.0f) ?
			1.0f / (h - sqrt(1.0f + h*h)) :
			1.0f / (h + sqrt(1.0f + h*h));
		c = 1.0f / sqrt(1.0f + tt*tt);
		s = tt * c;
	}

	k1 = ku - tt * kuv;
	k2 = kv + tt * kuv;

	if (fabs(k1) >= fabs(k2)) {
		pdir1 = c*r_old_u - s*r_old_v;
	} else {
		swap(k1, k2);
		pdir1 = s*r_old_u + c*r_old_v;
	}
	pdir2 = new_norm CROSS pdir1;
}

} // namespace trimesh
