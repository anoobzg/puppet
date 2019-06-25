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
#include "Xform.h"
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
	TriMesh() : grid_width(-1), grid_height(-1), frame(-1)
		{}

	xform global;
	int frame;

	// The basics: vertices and faces
	::std::vector<point> vertices;
	// Grid, if present
	::std::vector<int> grid;
	int grid_width, grid_height;

	// Other per-vertex properties
	::std::vector<Color> colors;
	::std::vector<float> confidences;
	::std::vector<unsigned> flags;

	// Computed per-vertex properties
	::std::vector<vec> normals;

	// Bounding structures
	BBox bbox;
	BSphere bsphere;

	void resize_grid(int width, int height)
	{
		grid_width = width;
		grid_height = height;
		grid.clear();
		grid.resize(grid_width * grid_height, GRID_INVALID);
	}
	void need_normals(bool simple_area_weighted = false);
	void need_bbox();
	void need_bsphere();

	//
	// Delete everything and release storage
	//
	void clear_vertices()      { clear_and_release(vertices); }
	void clear_grid()          { clear_and_release(grid);
	                             grid_width = grid_height = -1;}
	void clear_colors()        { clear_and_release(colors); }
	void clear_confidences()   { clear_and_release(confidences); }
	void clear_flags()         { clear_and_release(flags); }
	void clear_normals()       { clear_and_release(normals); }

	void clear_bbox()          { bbox.clear(); }
	void clear_bsphere()       { bsphere.valid = false; }
	void clear()
	{
		clear_vertices(); clear_grid();
		clear_colors(); clear_confidences(); clear_flags();
		clear_normals(); clear_bbox(); clear_bsphere();
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
};

} // namespace trimesh

#endif
