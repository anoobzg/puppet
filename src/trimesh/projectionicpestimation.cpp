#include "projectionicpestimation.h"
#include "lineqn.h"
#include "vector_util.h"

#define MIN_PAIRS 10
#define DESIRED_PAIRS 500
#define DESIRED_PAIRS_FINAL 5000
#define CDF_UPDATE_INTERVAL 20
#define REJECT_BDY false
#define USE_NORMCOMPAT true
#define REGULARIZATION 0.005f
#define MEDIAN_TO_SIGMA 1.4826f
#define dprintf TriMesh::dprintf

namespace trimesh
{

	// Determinant of a 3x3 matrix
	static float det(const float(&A)[3][3])
	{
		return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) +
			A[0][1] * (A[1][2] * A[2][0] - A[1][0] * A[2][2]) +
			A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
	}
	// Do rigid-body point-to-point alignment.  (This is done in the early stages
	// of registration to improve stability.)
	static void align_pt2pt(const std::vector<PtPair> &pairs,
		const trimesh::point &centroid1, const trimesh::point &centroid2,
		trimesh::xform &alignxf)
	{
		size_t n = pairs.size();

		float A[3][3] = { { 0 } };
		for (size_t i = 0; i < n; i++) {
			trimesh::vec v1 = pairs[i].p1 - centroid1;
			trimesh::vec v2 = pairs[i].p2 - centroid2;
			for (int j = 0; j < 3; j++)
				for (int k = 0; k < 3; k++)
					A[j][k] += v1[j] * v2[k];
		}
		float s[3], V[3][3];
		trimesh::svd<float, 3, 3>(A, s, V);
		if ((det(A) < 0.0f) ^ (det(V) < 0.0f)) {
			V[2][0] = -V[2][0];
			V[2][1] = -V[2][1];
			V[2][2] = -V[2][2];
		}
		alignxf = trimesh::xform::trans(centroid1) *
			trimesh::xform::fromarray(A) * trimesh::transp(trimesh::xform::fromarray(V)) *
			trimesh::xform::trans(-centroid2);
	}

	// Do symmetric point-to-plane alignment, returning alignxf
	// as well as eigenvectors and inverse eigenvalues
	static void align_pt2pl(const std::vector<PtPair> &pairs, float scale,
		const trimesh::point &centroid1, const trimesh::point &centroid2,
		float(&evec)[6][6], float(&einv)[6],
		trimesh::xform &alignxf)
	{
		int n = (int)pairs.size();

		float b[6] = { 0 };
		for (int i = 0; i < n; i++) {
			trimesh::vec p1 = pairs[i].p1 - centroid1;
			//cout << "p1.x"<<p1.x << endl;
			trimesh::vec p2 = pairs[i].p2 - centroid2;
			//cout << "p1:" << p1.x << endl;
			const trimesh::vec &n1 = pairs[i].n1;
			const trimesh::vec &n2 = pairs[i].n2;
			trimesh::vec nn = n1 + n2;

			trimesh::vec c = scale * (p1 CROSS n2 + p2 CROSS n1);
			trimesh::vec p12 = p1 - p2;

			float d = scale * (p12 DOT nn);

			float x[6] = { c[0], c[1], c[2], nn[0], nn[1], nn[2] };
			for (int j = 0; j < 6; j++) {
				b[j] += d * x[j];
				for (int k = j; k < 6; k++)
					evec[j][k] += x[j] * x[k];
			}

			// Regularization for rotational component - point to point
			float reg = REGULARIZATION * trimesh::sqr(scale);
			evec[0][0] += reg * (trimesh::sqr(p2[1]) + trimesh::sqr(p2[2]));
			evec[0][1] -= reg * p2[0] * p2[1];
			evec[0][2] -= reg * p2[0] * p2[2];
			evec[1][1] += reg * (trimesh::sqr(p2[0]) + trimesh::sqr(p2[2]));
			evec[1][2] -= reg * p2[1] * p2[2];
			evec[2][2] += reg * (trimesh::sqr(p2[0]) + trimesh::sqr(p2[1]));
		}

		// Regularization for translational component
		evec[3][3] += REGULARIZATION * n;
		evec[4][4] += REGULARIZATION * n;
		evec[5][5] += REGULARIZATION * n;

		// Make matrix symmetric
		for (int j = 0; j < 6; j++)
			for (int k = 0; k < j; k++)
				evec[j][k] = evec[k][j];

		// Eigen-decomposition and inverse
		float eval[6];
		trimesh::eigdc<float, 6>(evec, eval);
		for (int i = 0; i < 6; i++)
			einv[i] = 1.0f / eval[i];

		// Solve system
		trimesh::eigmult<float, 6>(evec, einv, b);
		trimesh::vec rot(b[0], b[1], b[2]), trans(b[3], b[4], b[5]);
		float rotangle = atan(len(rot));
		float cosangle = cos(rotangle);
		trans *= cosangle / ((1.0f + cosangle) * scale);
		alignxf = trimesh::xform::trans(trans + centroid1) *
			trimesh::xform::rot(rotangle, rot) *
			trimesh::xform::trans(trans - centroid2);
	}

	// Compute point-to-point or point-to-plane squared distances
	static void compute_dist2(const std::vector<PtPair> &pairs,
		std::vector<float> &distances2, ICP_iter_type iter_type)
	{
		int np = (int)pairs.size();
		if (np == 0)
		{
			std::cout << " no correspondent point! " << std::endl;
			return;
		}
		distances2.clear();
		distances2.resize(np);
		if (iter_type == ICP_POINT_TO_POINT) {
			//#pragma omp parallel for 
			for (int i = 0; i < np; i++)
				distances2[i] = dist2(pairs[i].p1, pairs[i].p2);
		}
		else /* ICP_POINT_TO_PLANE */ {
			float p2pl_mult = 0.5f / (1.0f + REGULARIZATION);
			float p2pt_mult = REGULARIZATION / (1.0f + REGULARIZATION);
			//#pragma omp parallel for 
			for (int i = 0; i < np; i++) {
				trimesh::vec v = pairs[i].p1 - pairs[i].p2;
				distances2[i] = p2pl_mult * (trimesh::sqr(v DOT pairs[i].n1) +
					trimesh::sqr(v DOT pairs[i].n2));
				distances2[i] += p2pt_mult * len2(v);
			}
		}
	}
	// Find the median of a list of numbers - makes a copy of vals (since it's
	// passed by value, not by reference) so that we can modify it
	static float median(std::vector<float> vals)
	{
		size_t n = vals.size();
		if (!n)
			return 0.0f;

		size_t mid = n / 2;
		nth_element(vals.begin(), vals.begin() + mid, vals.end());
		return vals[mid];
	}

	ProjectionICPEstimation::ProjectionICPEstimation()
		:m_iter_type(ICP_POINT_TO_POINT), m_tracer(NULL)
	{
		m_dist_thresh_mult = 5.0f;
		m_normdot_thresh = 0.6f;
	}

	ProjectionICPEstimation::~ProjectionICPEstimation()
	{

	}

	void ProjectionICPEstimation::SetTracer(ProjectionICPTracer* tracer)
	{
		m_tracer = tracer;
	}

	float ProjectionICPEstimation::Estimate(std::vector<PtPair>& pairs, trimesh::xform& source_form)
	{
		int npairs = (int)pairs.size();
		// Compute median point-to-point or point-to-plane distance.
		std::vector<float> distances2;
		compute_dist2(pairs, distances2, m_iter_type);
		float median_dist = sqrt(median(distances2));

		// Now compute threshold for rejection, as a multiple of sigma
		// (estimated robustly based on the median).
		float sigma = MEDIAN_TO_SIGMA * median_dist;
		float dist_thresh = m_dist_thresh_mult * sigma;
		float dist_thresh2 = trimesh::sqr(dist_thresh);

		// Reject
		float err = 0.0f;
		size_t next = 0;

		for (int i = 0; i < npairs; i++) {
			if (distances2[i] > dist_thresh2)
				continue;
			if ((pairs[i].n1 DOT pairs[i].n2) < m_normdot_thresh)
				continue;
			pairs[next++] = pairs[i];
			err += distances2[i];
		}

		pairs.erase(pairs.begin() + next, pairs.end());
		npairs = (int)pairs.size();

		if (npairs < MIN_PAIRS)
		{
			return -1.0f;
		}

		err = sqrt(err / npairs);
		// Compute centroids and scale
		trimesh::point centroid1, centroid2;
		for (int i = 0; i < npairs; i++)
		{
			centroid1 += pairs[i].p1;
			centroid2 += pairs[i].p2;
			//cout << "pairs["<<i<<"].p1" << pairs[i].p1 << endl;
			//cout << "pairs[" << i << "].p1" << pairs[i].p2 << endl;
		}
		centroid1 /= npairs;
		centroid2 /= npairs;
		float scale = 0.0f;

		for (int i = 0; i < npairs; i++) {
			scale += dist2(pairs[i].p1, centroid1);
			scale += dist2(pairs[i].p2, centroid2);
		}
		scale = sqrt(scale / (2 * npairs));
		scale = 1.0f / scale;

		// Do the minimization
		float evec[6][6] = { { 0 } }, einv[6] = { 0 };
		trimesh::xform alignxf;

		// First do rigid-body alignment
		if (m_iter_type == ICP_POINT_TO_POINT) {
			align_pt2pt(pairs, centroid1, centroid2, alignxf);
		}
		else {
			align_pt2pl(pairs, scale, centroid1, centroid2,
				evec, einv, alignxf);
		}

		source_form = alignxf * source_form;
		orthogonalize(source_form);
		trimesh::xform nalignxf = norm_xf(alignxf);

		for (int i = 0; i < npairs; i++) {
			pairs[i].p2 = alignxf * pairs[i].p2;
			pairs[i].n2 = nalignxf * pairs[i].n2;
		}
		compute_dist2(pairs, distances2, m_iter_type);
		err = sqrt(vector_mean(distances2));

		return err;
	}
}