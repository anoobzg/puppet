#ifndef TRIMESH_H
#define TRIMESH_H
/*
Szymon Rusinkiewicz
Princeton University

TriMesh.h
Class for triangle meshes.
*/

#include "Vec.h"
#include "Box.h"
#include "Color.h"
#include "strutil.h"
#include <vector>


namespace trimesh {

template <class T>
static inline void clear_and_release(::std::vector<T> &v)
{
	// Standard trick to release a vector's storage, since clear() doesn't
	::std::vector<T>().swap(v);
}

class TriMesh {
public:
	typedef Box<3,float> BBox;

	struct BSphere {
		point center;
		float r;
		bool valid;
		BSphere() : valid(false)
			{}
	};

	//
	// Enums
	//
	enum TstripRep { TSTRIP_LENGTH, TSTRIP_TERM };
	enum { GRID_INVALID = -1 };
	enum StatOp {
		STAT_MIN, STAT_MINABS, STAT_MAX, STAT_MAXABS,
		STAT_SUM, STAT_SUMABS, STAT_SUMSQR,
		STAT_MEAN, STAT_MEANABS, STAT_RMS,
		STAT_MEDIAN, STAT_STDEV };
	enum StatVal { STAT_VALENCE, STAT_FACEAREA, STAT_ANGLE,
		STAT_DIHEDRAL, STAT_EDGELEN, STAT_X, STAT_Y, STAT_Z };

	//
	// Constructor
	//
	TriMesh() : grid_width(-1), grid_height(-1), flag_curr(0)
		{}

	//
	// Members
	//

	// The basics: vertices and faces
	::std::vector<point> vertices;

	// Triangle strips
	::std::vector<int> tstrips;

	// Grid, if present
	::std::vector<int> grid;
	int grid_width, grid_height;

	// Other per-vertex properties
	::std::vector<Color> colors;
	::std::vector<float> confidences;
	::std::vector<unsigned> flags;
	unsigned flag_curr;

	::std::vector<int> best_frame_idx; // 2019.5.28 add by HeJinYing

	// Computed per-vertex properties
	::std::vector<vec> normals;
	::std::vector<vec> pdir1, pdir2;
	::std::vector<float> curv1, curv2;
	::std::vector< Vec<4,float> > dcurv;
	::std::vector<vec> cornerareas;
	::std::vector<float> pointareas;

	// Bounding structures
	BBox bbox;
	BSphere bsphere;

	void need_tstrips(TstripRep rep = TSTRIP_LENGTH);
	void unpack_tstrips();
	void resize_grid(int width, int height)
	{
		grid_width = width;
		grid_height = height;
		grid.clear();
		grid.resize(grid_width * grid_height, GRID_INVALID);
	}
	void triangulate_grid(bool remove_slivers = true);
	void need_normals(bool simple_area_weighted = false);
	void need_bbox();
	void need_bsphere();

	//
	// Delete everything and release storage
	//
	void clear_vertices()      { clear_and_release(vertices); }
	void clear_tstrips()       { clear_and_release(tstrips); }
	void clear_grid()          { clear_and_release(grid);
	                             grid_width = grid_height = -1;}
	void clear_colors()        { clear_and_release(colors); }
	void clear_confidences()   { clear_and_release(confidences); }
	void clear_flags()         { clear_and_release(flags); flag_curr = 0; }
	void clear_normals()       { clear_and_release(normals); }
	void clear_curvatures()    { clear_and_release(pdir1);
	                             clear_and_release(pdir2);
	                             clear_and_release(curv1);
	                             clear_and_release(curv2); }
	void clear_dcurv()         { clear_and_release(dcurv); }
	void clear_pointareas()    { clear_and_release(pointareas);
	                             clear_and_release(cornerareas); }
	void clear_bbox()          { bbox.clear(); }
	void clear_bsphere()       { bsphere.valid = false; }
	void clear()
	{
		clear_vertices(); clear_tstrips(); clear_grid();
		clear_colors(); clear_confidences(); clear_flags();
		clear_normals(); clear_curvatures(); clear_dcurv();
		clear_pointareas(); clear_bbox(); clear_bsphere();
		best_frame_idx.clear();
	}

	//
	// Input and output
	//
protected:
	static bool read_helper(const char *filename, TriMesh *mesh);
public:
	static TriMesh *read(const char *filename);
	static TriMesh *read(const ::std::string &filename);
	bool write(const char *filename);
	bool write(const ::std::string &filename);

	// Statistics
	float stat(StatOp op, StatVal val);

	//
	// Debugging
	//

	// Debugging printout, controllable by a "verbose"ness parameter
	static int verbose;
	static void set_verbose(int);
	static void (*dprintf_hook)(const char *);
	static void set_dprintf_hook(void (*hook)(const char *));
	static void dprintf(const char *format, ...);

	// Same as above, but fatal-error printout
	static void (*eprintf_hook)(const char *);
	static void set_eprintf_hook(void (*hook)(const char *));
	static void eprintf(const char *format, ...);

};

using namespace ::std;
struct renderingData
{
	vector<int> idxs;
	vector<point> points;
	vector<vec> norms;
	void clear()
	{
		idxs.clear();
		points.clear();
		norms.clear();
	};
};

} // namespace trimesh

#endif
