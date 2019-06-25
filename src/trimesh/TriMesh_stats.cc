/*
Szymon Rusinkiewicz
Princeton University

TriMesh_stats.cc
Computation of various statistics on the mesh.
*/

#include "TriMesh.h"
#include <numeric>
using namespace std;


namespace trimesh {

// Compute a variety of statistics.  Takes a type of statistic to compute,
// and what to do with it.
float TriMesh::stat(StatOp op, StatVal val)
{
	vector<float> vals;

	switch (val) {
		case STAT_X: {
			int nv = vertices.size();
			for (int i = 0; i < nv; i++)
				vals.push_back(vertices[i][0]);
			break;
		}
		case STAT_Y: {
			int nv = vertices.size();
			for (int i = 0; i < nv; i++)
				vals.push_back(vertices[i][1]);
			break;
		}
		case STAT_Z: {
			int nv = vertices.size();
			for (int i = 0; i < nv; i++)
				vals.push_back(vertices[i][2]);
			break;
		}
		default:
			return 0.0f;
	}

	int n = vals.size();
	if (!n)
		return 0.0f;

	// Take absolute value or square
	switch (op) {
		case STAT_MINABS:
		case STAT_MAXABS:
		case STAT_SUMABS:
		case STAT_MEANABS:
			for (int i = 0; i < n; i++) {
				if (vals[i] < 0.0f)
					vals[i] = -vals[i];
			}
			break;

		case STAT_SUMSQR:
		case STAT_RMS:
			for (int i = 0; i < n; i++)
				vals[i] *= vals[i];
			break;

		default:
			break;
	}

	// Now do the computation
	switch (op) {
		case STAT_MIN:
		case STAT_MINABS:
			return *min_element(vals.begin(), vals.end());

		case STAT_MAX:
		case STAT_MAXABS:
			return *max_element(vals.begin(), vals.end());

		case STAT_SUM:
		case STAT_SUMABS:
		case STAT_SUMSQR:
			return accumulate(vals.begin(), vals.end(), 0.0f);

		case STAT_MEAN:
		case STAT_MEANABS:
			return accumulate(vals.begin(), vals.end(), 0.0f) / n;

		case STAT_RMS:
			return sqrt(accumulate(vals.begin(), vals.end(), 0.0f)
				/ n);

		case STAT_MEDIAN:
			if (n & 1) {
				nth_element(vals.begin(),
				            vals.begin() + n/2,
				            vals.end());
				return vals[n/2];
			} else {
				nth_element(vals.begin(),
				            vals.begin() + n/2 - 1,
				            vals.end());
				float tmp = vals[n/2 - 1];
				nth_element(vals.begin(),
				            vals.begin() + n/2,
				            vals.end());
				return 0.5f * (tmp + vals[n/2]);
			}

		case STAT_STDEV: {
			float mean = accumulate(vals.begin(), vals.end(), 0.0f)
				/ n;
			for (int i = 0; i < n; i++)
				vals[i] = sqr(vals[i] - mean);
			return sqrt(accumulate(vals.begin(), vals.end(), 0.0f)
				/ n);
		}

		default:
			return 0.0f; // Can't happen, I hope.
	}
}

} // namespace trimesh
