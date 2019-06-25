/*
Szymon Rusinkiewicz
Princeton University

TriMesh_io.cc
Input and output of triangle meshes
Can read: PLY (triangle mesh and range grid), OFF, OBJ, RAY, SM, 3DS, VVD, STL
Can write: PLY (triangle mesh and range grid), OFF, OBJ, RAY, SM, STL, C++, DAE
*/

#include "TriMesh.h"
#include "endianutil.h"

#include <cstdio>
#include <cerrno>
#include <cctype>
#include <cstdarg>
using namespace std;

#define dprintf TriMesh::dprintf
#define eprintf TriMesh::eprintf


#define GET_LINE() do { if (!fgets(buf, 1024, f)) return false; } while (0)
#define GET_WORD() do { if (fscanf(f, " %1023s", buf) != 1) return false; } while (0)
#define COND_READ(cond, where, len) do { if ((cond) && !fread((void *)&(where), (len), 1, f)) return false; } while (0)
#define FPRINTF(...) do { if (fprintf(__VA_ARGS__) < 0) return false; } while (0)
#define FWRITE(ptr, size, nmemb, stream) do { if (fwrite((ptr), (size), (nmemb), (stream)) != (nmemb)) return false; } while (0)
#define LINE_IS(text) begins_with(buf, text)

#define BIGNUM 1.0e10f


namespace trimesh {

// Forward declarations
static bool read_ply(FILE *f, TriMesh *mesh);
static bool read_obj(FILE *f, TriMesh *mesh);

static bool read_verts_bin(FILE *f, TriMesh *mesh, bool &need_swap,
	int nverts, int vert_len, int vert_pos, int vert_norm,
	int vert_color, bool float_color, int vert_conf);
static bool slurp_verts_bin(FILE *f, TriMesh *mesh, bool need_swap,
	int nverts);
static bool read_verts_asc(FILE *f, TriMesh *mesh,
	int nverts, int vert_len, int vert_pos, int vert_norm,
	int vert_color, bool float_color, int vert_conf);
static bool read_strips_bin(FILE *f, TriMesh *mesh, bool need_swap);
static bool read_strips_asc(FILE *f, TriMesh *mesh);
static bool read_grid_bin(FILE *f, TriMesh *mesh, bool need_swap);
static bool read_grid_asc(FILE *f, TriMesh *mesh);

static int ply_type_len(const char *buf, bool binary);
static bool ply_property(const char *buf, int &len, bool binary);
static void check_need_swap(const point &p, bool &need_swap);
static void check_ind_range(TriMesh *mesh);
static void skip_comments(FILE *f);

static bool write_ply_ascii(TriMesh *mesh, FILE *f,
	bool write_norm, bool write_grid, bool float_color);
static bool write_ply_binary(TriMesh *mesh, FILE *f,
	bool little_endian, bool write_norm, bool write_grid, bool float_color);
static bool write_verts_asc(TriMesh *mesh, FILE *f,
                            const char *before_vert,
                            const char *before_norm,
                            const char *before_color,
                            bool float_color,
                            const char *before_conf,
                            const char *after_line);
static bool write_verts_bin(TriMesh *mesh, FILE *f, bool need_swap,
                            bool write_norm, bool write_color,
                            bool float_color, bool write_conf);
static bool write_strips_asc(TriMesh *mesh, FILE *f);
static bool write_strips_bin(TriMesh *mesh, FILE *f, bool need_swap);
static bool write_grid_asc(TriMesh *mesh, FILE *f);
static bool write_grid_bin(TriMesh *mesh, FILE *f, bool need_swap);


// unget a whole string of characters
static void pushback(const char *buf, FILE *f)
{
	const char *c = buf;
	while (*c)
		c++;
	while ((--c) >= buf)
		ungetc(*c, f);
}


// std::string versions of read/write
TriMesh *TriMesh::read(const ::std::string &filename)
{
	return read(filename.c_str());
}

bool TriMesh::write(const ::std::string &filename)
{
	return write(filename.c_str());
}


// Read a TriMesh from a file.  Defined to use a helper function to make
// subclassing easier.
TriMesh *TriMesh::read(const char *filename)
{
	TriMesh *mesh = new TriMesh();

	if (read_helper(filename, mesh))
		return mesh;

	delete mesh;
	return NULL;
}


// Actually read a mesh.  Tries to figure out type of file from first
// few bytes.  Filename can be "-" for stdin.
// STL doesn't have a magic number, nor any other way of recognizing it.
// Recognize file.stl and stl:- constructions.
bool TriMesh::read_helper(const char *filename, TriMesh *mesh)
{
	if (!filename || *filename == '\0')
		return false;

	FILE *f = NULL;
	bool ok = false;
	int c;

	if (strcmp(filename, "-") == 0) {
		f = stdin;
		filename = "standard input";
	} else if (begins_with(filename, "stl:-")) {
		f = stdin;
		filename = "standard input";
	} else {
		f = fopen(filename, "rb");
		if (!f) {
			eprintf("Error opening [%s] for reading: %s.\n", filename,
				strerror(errno));
			return false;
		}
	}
	dprintf("Reading %s... ", filename);

	// Else recognize based on header
	c = fgetc(f);
	if (c == EOF) {
		eprintf("Can't read header.\n");
		goto out;
	}

	if (c == 'p') {
		// See if it's a ply file
		char buf[4];
		if (!fgets(buf, 4, f)) {
			eprintf("Can't read header.\n");
			goto out;
		}
		if (strncmp(buf, "ly", 2) == 0)
			ok = read_ply(f, mesh);
	} else if (c == '#') {
		char buf[1024];
		GET_LINE();
		// Assume an obj file
		ok = read_obj(f, mesh);
	} else if (c == 'v' || c == 'u' || c == 'f' || c == 'g' ||
	           c == 's' || c == 'o' || c == 'm') {
		// Assume an obj file
		ungetc(c, f);
		ok = read_obj(f, mesh);
	} else {
		eprintf("Unknown file type.\n");
	}

out:
	if (f)
		fclose(f);
	if (!ok) {
		eprintf("Error reading file [%s].\n", filename);
		return false;
	}

	return true;
}


// Read a ply file
static bool read_ply(FILE *f, TriMesh *mesh)
{
	char buf[1024];
	bool binary = false, need_swap = false, float_color = false;
	int result, nverts = 0, nfaces = 0, nstrips = 0, ngrid = 0;
	int vert_len = 0, vert_pos = -1, vert_norm = -1;
	int vert_color = -1, vert_conf = -1;
	int face_len = 0, face_count = -1, face_idx = -1;

	// Read file format
	GET_LINE();
	while (buf[0] && isspace(buf[0]))
		GET_LINE();
	if (LINE_IS("format binary_big_endian 1.0")) {
		binary = true;
		need_swap = we_are_little_endian();
	} else if (LINE_IS("format binary_little_endian 1.0")) {
		binary = true;
		need_swap = we_are_big_endian();
	} else if (LINE_IS("format ascii 1.0")) {
		binary = false;
	} else {
		eprintf("Unknown ply format or version.\n");
		return false;
	}

	// Skip comments and unknown obj_info lines
	GET_LINE();
	while (LINE_IS("obj_info") || LINE_IS("comment")) {
		if (LINE_IS("obj_info num_cols"))
			sscanf(buf, "obj_info num_cols %d", &mesh->grid_width);
		if (LINE_IS("obj_info num_rows"))
			sscanf(buf, "obj_info num_rows %d", &mesh->grid_height);
		GET_LINE();
	}

	// Skip until we find vertices
	int skip1 = 0;
	while (!LINE_IS("end_header") && !LINE_IS("element vertex")) {
		char elem_name[1024];
		int nelem = 0, elem_len = 0;
		sscanf(buf, "element %s %d", elem_name, &nelem);
		GET_LINE();
		while (LINE_IS("property")) {
			if (!ply_property(buf, elem_len, binary))
				return false;
			GET_LINE();
		}
		skip1 += nelem * elem_len;
	}

	// Find number of vertices
	result = sscanf(buf, "element vertex %d\n", &nverts);
	if (result != 1) {
		eprintf("Expected \"element vertex\".\n");
		return false;
	}

	// Parse vertex properties
	GET_LINE();
	while (LINE_IS("property")) {
		if (LINE_IS("property float x") ||
		    LINE_IS("property float32 x"))
			vert_pos = vert_len;
		if (LINE_IS("property float nx") ||
		    LINE_IS("property float32 nx"))
			vert_norm = vert_len;
		if (LINE_IS("property uchar diffuse_red") ||
		    LINE_IS("property uint8 diffuse_red") ||
		    LINE_IS("property uchar red") ||
		    LINE_IS("property uint8 red"))
			vert_color = vert_len;
		if (LINE_IS("property float diffuse_red") ||
		    LINE_IS("property float32 diffuse_red") ||
		    LINE_IS("property float red") ||
		    LINE_IS("property float32 red"))
			vert_color = vert_len, float_color = true;
		if (LINE_IS("property float confidence") ||
		    LINE_IS("property float32 confidence"))
			vert_conf = vert_len;

		if (!ply_property(buf, vert_len, binary))
			return false;

		GET_LINE();
	}

	// Skip until we find faces
	int skip2 = 0;
	while (!LINE_IS("end_header") && !LINE_IS("element face") &&
	       !LINE_IS("element tristrips") && !LINE_IS("element range_grid")) {
		char elem_name[1024];
		int nelem = 0, elem_len = 0;
		sscanf(buf, "element %s %d", elem_name, &nelem);
		GET_LINE();
		while (LINE_IS("property")) {
			if (!ply_property(buf, elem_len, binary))
				return false;
			GET_LINE();
		}
		skip2 += nelem * elem_len;
	}


	// Look for faces, tristrips, or range grid
	if (LINE_IS("element face")) {
		if (sscanf(buf, "element face %d\n", &nfaces) != 1)
			return false;
		GET_LINE();
		while (LINE_IS("property")) {
			char count_type[256], ind_type[256];
			if (sscanf(buf, "property list %255s %255s vertex_ind",
					count_type, ind_type) == 2) {
				int count_len = ply_type_len(count_type, binary);
				int ind_len = ply_type_len(ind_type, binary);
				if (count_len && ind_len) {
					face_count = face_len;
					face_idx = face_len + count_len;
					face_len += count_len;
				}
			} else if (!ply_property(buf, face_len, binary))
				return false;
			GET_LINE();
		}
	} else if (LINE_IS("element tristrips")) {
		nstrips = 1;
		GET_LINE();
		if (!LINE_IS("property list int int vertex_ind") &&
		    !LINE_IS("property list int32 int32 vertex_ind"))
			return false;
		GET_LINE();
	} else if (LINE_IS("element range_grid")) {
		if (sscanf(buf, "element range_grid %d\n", &ngrid) != 1)
			return false;
		if (ngrid != mesh->grid_width*mesh->grid_height) {
			eprintf("Range grid size does not equal num_rows*num_cols.\n");
			return false;
		}
		GET_LINE();
		if (!LINE_IS("property list uchar int vertex_ind") &&
		    !LINE_IS("property list uint8 int32 vertex_ind") &&
		    !LINE_IS("property list char int vertex_ind") &&
		    !LINE_IS("property list int8 int32 vertex_ind"))
			return false;
		GET_LINE();
	}

	while (LINE_IS("property")) {
		if (!ply_property(buf, face_len, binary))
			return false;
		GET_LINE();
	}

	// Skip to the end of the header
	while (!LINE_IS("end_header"))
		GET_LINE();
	if (binary && buf[10] == '\r') {
		eprintf("Warning: possibly corrupt file. (Transferred as ASCII instead of BINARY?)\n");
	}

	// Actually read everything in
	if (skip1) {
		if (binary)
			fseek(f, skip1, SEEK_CUR);
		else
			for (int i = 0; i < skip1; i++)
				GET_WORD();
	}
	if (binary) {
		if (!read_verts_bin(f, mesh, need_swap, nverts, vert_len,
		                    vert_pos, vert_norm, vert_color,
		                    float_color, vert_conf))
			return false;
	} else {
		if (!read_verts_asc(f, mesh, nverts, vert_len,
		                    vert_pos, vert_norm, vert_color,
		                    float_color, vert_conf))
			return false;
	}

	if (skip2) {
		if (binary)
			fseek(f, skip2, SEEK_CUR);
		else
			for (int i = 0; i < skip2; i++)
				GET_WORD();
	}

	if (ngrid) {
		if (binary) {
			if (!read_grid_bin(f, mesh, need_swap))
				return false;
		} else {
			if (!read_grid_asc(f, mesh))
				return false;
		}
	}
	return true;
}


#define CHUNK_3DS_MAIN  0x4D4Du
#define CHUNK_3DS_MODEL 0x3D3Du
#define CHUNK_3DS_OBJ   0x4000u
#define CHUNK_3DS_MESH  0x4100u
#define CHUNK_3DS_VERT  0x4110u
#define CHUNK_3DS_FACE  0x4120u

// Read an obj file
static bool read_obj(FILE *f, TriMesh *mesh)
{
	vector<int> thisface;
	while (1) {
		skip_comments(f);
		if (feof(f))
			break;
		char buf[1024];
		GET_LINE();
		if (LINE_IS("v ") || LINE_IS("v\t")) {
			float x, y, z;
			if (sscanf(buf+1, "%f %f %f", &x, &y, &z) != 3) {
				return false;
			}
			mesh->vertices.push_back(point(x,y,z));
		} else if (LINE_IS("vn ") || LINE_IS("vn\t")) {
			float x, y, z;
			if (sscanf(buf+2, "%f %f %f", &x, &y, &z) != 3) {
				return false;
			}
			mesh->normals.push_back(vec(x,y,z));
		}
	}

	// XXX - FIXME
	// Right now, handling of normals is fragile: we assume that
	// if we have the same number of normals as vertices,
	// the file just uses per-vertex normals.  Otherwise, we can't
	// handle it.
	if (mesh->vertices.size() != mesh->normals.size())
		mesh->normals.clear();

	return true;
}

// Read nverts vertices from a binary file.
// vert_len = total length of a vertex record in bytes
// vert_pos, vert_norm, vert_color, vert_conf =
//   position of vertex coordinates / normals / color / confidence in record
// need_swap = swap for opposite endianness
// float_color = colors are 4-byte float * 3, vs 1-byte uchar * 3
static bool read_verts_bin(FILE *f, TriMesh *mesh, bool &need_swap,
	int nverts, int vert_len, int vert_pos, int vert_norm,
	int vert_color, bool float_color, int vert_conf)
{
	const int vert_size = 12;
	const int norm_size = 12;
	const int color_size = float_color ? 12 : 3;
	const int conf_size = 4;

	if (nverts < 0 || vert_len < 12 || vert_pos < 0)
		return false;
	if (nverts == 0)
		return true;

	int old_nverts = mesh->vertices.size();
	int new_nverts = old_nverts + nverts;
	mesh->vertices.resize(new_nverts);

	bool have_norm = (vert_norm >= 0);
	bool have_color = (vert_color >= 0);
	bool have_conf = (vert_conf >= 0);
	if (have_norm)
		mesh->normals.resize(new_nverts);
	if (have_color)
		mesh->colors.resize(new_nverts);
	if (have_conf)
		mesh->confidences.resize(new_nverts);

	unsigned char *buf = new unsigned char[vert_len];
	COND_READ(true, buf[0], vert_len);

	int i = old_nverts;
	memcpy(&mesh->vertices[i][0], &buf[vert_pos], vert_size);
	if (have_norm)
		memcpy(&mesh->normals[i][0], &buf[vert_norm], norm_size);
	if (have_color && float_color)
		memcpy(&mesh->colors[i][0], &buf[vert_color], color_size);
	if (have_color && !float_color)
		mesh->colors[i] = Color(&buf[vert_color]);
	if (have_conf)
		memcpy(&mesh->confidences[i], &buf[vert_conf], conf_size);

	check_need_swap(mesh->vertices[i], need_swap);
	if (need_swap) {
		swap_float(mesh->vertices[i][0]);
		swap_float(mesh->vertices[i][1]);
		swap_float(mesh->vertices[i][2]);
		if (have_norm) {
			swap_float(mesh->normals[i][0]);
			swap_float(mesh->normals[i][1]);
			swap_float(mesh->normals[i][2]);
		}
		if (have_color && float_color) {
			swap_float(mesh->colors[i][0]);
			swap_float(mesh->colors[i][1]);
			swap_float(mesh->colors[i][2]);
		}
		if (have_conf)
			swap_float(mesh->confidences[i]);
	}

	dprintf("\n  Reading %d vertices... ", nverts);
	if (vert_len == 12 && sizeof(point) == 12 && nverts > 1)
		return slurp_verts_bin(f, mesh, need_swap, nverts);
	while (++i < new_nverts) {
		COND_READ(true, buf[0], vert_len);
		memcpy(&mesh->vertices[i][0], &buf[vert_pos], vert_size);
		if (have_norm)
			memcpy(&mesh->normals[i][0], &buf[vert_norm], norm_size);
		if (have_color && float_color)
			memcpy(&mesh->colors[i][0], &buf[vert_color], color_size);
		if (have_color && !float_color)
			mesh->colors[i] = Color(&buf[vert_color]);
		if (have_conf)
			memcpy(&mesh->confidences[i], &buf[vert_conf], conf_size);

		if (need_swap) {
			swap_float(mesh->vertices[i][0]);
			swap_float(mesh->vertices[i][1]);
			swap_float(mesh->vertices[i][2]);
			if (have_norm) {
				swap_float(mesh->normals[i][0]);
				swap_float(mesh->normals[i][1]);
				swap_float(mesh->normals[i][2]);
			}
			if (have_color && float_color) {
				swap_float(mesh->colors[i][0]);
				swap_float(mesh->colors[i][1]);
				swap_float(mesh->colors[i][2]);
			}
			if (have_conf)
				swap_float(mesh->confidences[i]);
		}
	}

	return true;
}


// Optimized reader for the simple case of just vertices w/o other properties
static bool slurp_verts_bin(FILE *f, TriMesh *mesh, bool need_swap, int nverts)
{
	int first = mesh->vertices.size() - nverts + 1;
	COND_READ(true, mesh->vertices[first][0], (nverts-1)*12);
	if (need_swap) {
	    for (size_t i = first; i < mesh->vertices.size(); i++) {
			swap_float(mesh->vertices[i][0]);
			swap_float(mesh->vertices[i][1]);
			swap_float(mesh->vertices[i][2]);
		}
	}
	return true;
}


// Read a bunch of vertices from an ASCII file.
// Parameters are as in read_verts_bin, but offsets are in
// (white-space-separated) words, rather than in bytes
static bool read_verts_asc(FILE *f, TriMesh *mesh,
	int nverts, int vert_len, int vert_pos, int vert_norm,
	int vert_color, bool float_color, int vert_conf)
{
	if (nverts < 0 || vert_len < 3 || vert_pos < 0)
		return false;
	if (nverts == 0)
		return true;

	int old_nverts = mesh->vertices.size();
	int new_nverts = old_nverts + nverts;
	mesh->vertices.resize(new_nverts);
	if (vert_norm > 0)
		mesh->normals.resize(new_nverts);
	if (vert_color > 0)
		mesh->colors.resize(new_nverts);
	if (vert_conf > 0)
		mesh->confidences.resize(new_nverts);

	char buf[1024];
	skip_comments(f);
	dprintf("\n  Reading %d vertices... ", nverts);
	for (int i = old_nverts; i < new_nverts; i++) {
		for (int j = 0; j < vert_len; j++) {
			if (j == vert_pos) {
				if (fscanf(f, "%f %f %f",
				              &mesh->vertices[i][0],
				              &mesh->vertices[i][1],
				              &mesh->vertices[i][2]) != 3)
					return false;
				j += 2;
			} else if (j == vert_norm) {
				if (fscanf(f, "%f %f %f",
				              &mesh->normals[i][0],
				              &mesh->normals[i][1],
				              &mesh->normals[i][2]) != 3)
					return false;
				j += 2;
			} else if (j == vert_color && float_color) {
				float r, g, b;
				if (fscanf(f, "%f %f %f", &r, &g, &b) != 3)
					return false;
				mesh->colors[i] = Color(r,g,b);
				j += 2;
			} else if (j == vert_color && !float_color) {
				int r, g, b;
				if (fscanf(f, "%d %d %d", &r, &g, &b) != 3)
					return false;
				mesh->colors[i] = Color(r,g,b);
				j += 2;
			} else if (j == vert_conf) {
				if (fscanf(f, "%f", &mesh->confidences[i]) != 1)
					return false;
			} else {
				GET_WORD();
			}
		}
	}

	return true;
}

// Read triangle strips from a binary file
static bool read_strips_bin(FILE *f, TriMesh *mesh, bool need_swap)
{
	int striplen;
	COND_READ(true, striplen, 4);
	if (need_swap)
		swap_int(striplen);

	int old_striplen = mesh->tstrips.size();
	int new_striplen = old_striplen + striplen;
	mesh->tstrips.resize(new_striplen);

	dprintf("\n  Reading triangle strips... ");
	COND_READ(true, mesh->tstrips[old_striplen], 4*striplen);
	if (need_swap) {
		for (int i = old_striplen; i < new_striplen; i++)
			swap_int(mesh->tstrips[i]);
	}

	return true;
}


// Read triangle strips from an ASCII file
static bool read_strips_asc(FILE *f, TriMesh *mesh)
{
	skip_comments(f);
	int striplen;
	if (fscanf(f, "%d", &striplen) != 1)
		return false;
	int old_striplen = mesh->tstrips.size();
	int new_striplen = old_striplen + striplen;
	mesh->tstrips.resize(new_striplen);

	dprintf("\n  Reading triangle strips... ");
	skip_comments(f);
	for (int i = old_striplen; i < new_striplen; i++)
		if (fscanf(f, "%d", &mesh->tstrips[i]) != 1)
			return false;

	return true;
}


// Read range grid data from a binary file
static bool read_grid_bin(FILE *f, TriMesh *mesh, bool need_swap)
{
	dprintf("\n  Reading range grid... ");
	int ngrid = mesh->grid_width * mesh->grid_height;
	mesh->grid.resize(ngrid, TriMesh::GRID_INVALID);
	for (int i = 0; i < ngrid; i++) {
		int n = fgetc(f);
		if (n == EOF)
			return false;
		while (n--) {
			if (!fread((void *)&(mesh->grid[i]), 4, 1, f))
				return false;
			if (need_swap)
				swap_int(mesh->grid[i]);
		}
	}

	//mesh->triangulate_grid();
	return true;
}


// Read range grid data from an ASCII file
static bool read_grid_asc(FILE *f, TriMesh *mesh)
{
	dprintf("\n  Reading range grid... ");
	int ngrid = mesh->grid_width * mesh->grid_height;
	mesh->grid.resize(ngrid, TriMesh::GRID_INVALID);
	for (int i = 0; i < ngrid; i++) {
		int n;
		if (fscanf(f, "%d", &n) != 1)
			return false;
		while (n--) {
			if (fscanf(f, "%d", &(mesh->grid[i])) != 1)
				return false;
		}
	}

	//mesh->triangulate_grid();
	return true;
}


// Return the length in bytes of a ply property type, 0 if can't parse
static int ply_type_len(const char *buf, bool binary)
{
	if (begins_with(buf, "char") ||
	    begins_with(buf, "uchar") ||
	    begins_with(buf, "int8") ||
	    begins_with(buf, "uint8")) {
		return 1;
	} else if (begins_with(buf, "short") ||
	           begins_with(buf, "ushort") ||
	           begins_with(buf, "int16") ||
	           begins_with(buf, "uint16")) {
		return (binary ? 2 : 1);
	} else if (begins_with(buf, "int") ||
	           begins_with(buf, "uint") ||
	           begins_with(buf, "float") ||
	           begins_with(buf, "int32") ||
	           begins_with(buf, "uint32") ||
	           begins_with(buf, "float32")) {
		return (binary ? 4 : 1);
	} else if (begins_with(buf, "double") ||
	           begins_with(buf, "float64")) {
		return (binary ? 8 : 1);
	}
	return 0;
}


// Parse a PLY property line, and figure how many bytes it represents
// Increments "len" by the number of bytes, or by 1 if !binary
static bool ply_property(const char *buf, int &len, bool binary)
{
	int type_len = ply_type_len(buf+9, binary);
	if (type_len) {
		len += type_len;
		return true;
	}

	eprintf("Unsupported vertex property: [%s].\n", buf);
	return false;
}


// Figure out whether the need_swap setting makes sense, or whether this
// file incorrectly declares its endianness
static void check_need_swap(const point &p, bool &need_swap)
{
	float p0 = p[0], p1 = p[1], p2 = p[2];
	if (need_swap) {
		swap_float(p0);
		swap_float(p1);
		swap_float(p2);
	}
	bool makes_sense = (p0 > -BIGNUM && p0 < BIGNUM &&
	                    p1 > -BIGNUM && p1 < BIGNUM &&
	                    p2 > -BIGNUM && p2 < BIGNUM);
	if (makes_sense)
		return;

	swap_float(p0);
	swap_float(p1);
	swap_float(p2);

	bool makes_sense_swapped = (p0 > -BIGNUM && p0 < BIGNUM &&
	                            p1 > -BIGNUM && p1 < BIGNUM &&
	                            p2 > -BIGNUM && p2 < BIGNUM);
	if (makes_sense_swapped) {
		dprintf("Compensating for bogus endianness...\n");
		need_swap = !need_swap;
	}
}

// Skip comments in an ASCII file (lines beginning with #)
static void skip_comments(FILE *f)
{
	int c;
	bool in_comment = false;
	while (1) {
		c = fgetc(f);
		if (c == EOF)
			return;
		if (in_comment) {
			if (c == '\n')
				in_comment = false;
		} else if (c == '#') {
			in_comment = true;
		} else if (!isspace(c)) {
			break;
		}
	}
	ungetc(c, f);
}

// Write mesh to a file
bool TriMesh::write(const char *filename)
{
	if (!filename || *filename == '\0') {
		eprintf("Can't write to empty filename.\n");
		return false;
	}

	if (vertices.empty()) {
		eprintf("Empty mesh - nothing to write.\n");
		return false;
	}

	enum { PLY_ASCII, PLY_BINARY_BE, PLY_BINARY_LE,
	       RAY, OBJ, OFF, SM, STL, CC, DAE } filetype;
	// Set default file type to be native-endian binary ply
	filetype = we_are_little_endian() ? PLY_BINARY_LE : PLY_BINARY_BE;

	bool write_norm = false;
	bool write_grid = !grid.empty();
	bool float_color = false;

	// Infer file type from file extension
	if (ends_with(filename, ".ply"))
		filetype = we_are_little_endian() ?
				PLY_BINARY_LE : PLY_BINARY_BE;
	else if (ends_with(filename, ".ray"))
		filetype = RAY;
	else if (ends_with(filename, ".obj"))
		filetype = OBJ;
	else if (ends_with(filename, ".off"))
		filetype = OFF;
	else if (ends_with(filename, ".sm"))
		filetype = SM;
	else if (ends_with(filename, ".stl"))
		filetype = STL;
	else if (ends_with(filename, ".cc"))
		filetype = CC;
	else if (ends_with(filename, ".c++"))
		filetype = CC;
	else if (ends_with(filename, ".cpp"))
		filetype = CC;
	else if (ends_with(filename, ".C"))
		filetype = CC;
	else if (ends_with(filename, ".dae"))
		filetype = DAE;

	// Handle filetype:filename.foo constructs
	while (1) {
		if (begins_with(filename, "norm:")) {
			filename += 5;
			write_norm = true;
		} else if (begins_with(filename, "nogrid:")) {
			filename += 7;
			write_grid = false;
		} else if (begins_with(filename, "cflt:")) {
			filename += 5;
			float_color = true;
		} else if (begins_with(filename, "ply:")) {
			filename += 4;
			filetype = we_are_little_endian() ?
					PLY_BINARY_LE :
					PLY_BINARY_BE;
		} else if (begins_with(filename, "ply_binary:")) {
			filename += 11;
			filetype = we_are_little_endian() ?
					PLY_BINARY_LE :
					PLY_BINARY_BE;
		} else if (begins_with(filename, "ply_binary_be:")) {
			filename += 14;
			filetype = PLY_BINARY_BE;
		} else if (begins_with(filename, "ply_binary_le:")) {
			filename += 14;
			filetype = PLY_BINARY_LE;
		} else if (begins_with(filename, "ply_ascii:")) {
			filename += 10;
			filetype = PLY_ASCII;
		} else if (begins_with(filename, "ply_asc:")) {
			filename += 8;
			filetype = PLY_ASCII;
		} else if (begins_with(filename, "ascii:")) {
			filename += 6;
			filetype = PLY_ASCII;
		} else if (begins_with(filename, "asc:")) {
			filename += 4;
			filetype = PLY_ASCII;
		} else if (begins_with(filename, "be:")) {
			filename += 3;
			filetype = PLY_BINARY_BE;
		} else if (begins_with(filename, "le:")) {
			filename += 3;
			filetype = PLY_BINARY_LE;
		} else if (begins_with(filename, "ray:")) {
			filename += 4;
			filetype = RAY;
		} else if (begins_with(filename, "obj:")) {
			filename += 4;
			filetype = OBJ;
		} else if (begins_with(filename, "off:")) {
			filename += 4;
			filetype = OFF;
		} else if (begins_with(filename, "sm:")) {
			filename += 3;
			filetype = SM;
		} else if (begins_with(filename, "stl:")) {
			filename += 4;
			filetype = STL;
		} else if (begins_with(filename, "cc:")) {
			filename += 3;
			filetype = CC;
		} else if (begins_with(filename, "dae:")) {
			filename += 4;
			filetype = DAE;
		} else {
			break;
		}
	}


	FILE *f = NULL;

	if (strcmp(filename, "-") == 0) {
		f = stdout;
		filename = "standard output";
	} else {
		f = fopen(filename, "wb");
		if (!f) {
			eprintf("Error opening [%s] for writing: %s.\n", filename,
				strerror(errno));
			return false;
		}
	}

	dprintf("Writing %s... ", filename);

	bool ok = false;
	switch (filetype) {
		case PLY_ASCII:
			ok = write_ply_ascii(this, f, write_norm, write_grid, float_color);
			break;
		case PLY_BINARY_BE:
			ok = write_ply_binary(this, f,
				false, write_norm, write_grid, float_color);
			break;
		case PLY_BINARY_LE:
			ok = write_ply_binary(this, f,
				true, write_norm, write_grid, float_color);
			break;
	}

	fclose(f);
	if (!ok) {
		eprintf("Error writing file [%s].\n", filename);
		return false;
	}

	dprintf("Done.\n");
	return true;
}


// Write a ply header
static bool write_ply_header(TriMesh *mesh, FILE *f, const char *format,
                             bool write_grid, bool write_tstrips,
                             bool write_norm, bool float_color)
{
	FPRINTF(f, "ply\nformat %s 1.0\n", format);
	if (write_grid) {
		FPRINTF(f, "obj_info num_cols %d\n", mesh->grid_width);
		FPRINTF(f, "obj_info num_rows %d\n", mesh->grid_height);
	}
	FPRINTF(f, "element vertex %lu\n",
		(unsigned long) mesh->vertices.size());
	FPRINTF(f, "property float x\n");
	FPRINTF(f, "property float y\n");
	FPRINTF(f, "property float z\n");
	if (write_norm && !mesh->normals.empty()) {
		FPRINTF(f, "property float nx\n");
		FPRINTF(f, "property float ny\n");
		FPRINTF(f, "property float nz\n");
	}
	if (!mesh->colors.empty() && float_color) {
		FPRINTF(f, "property float diffuse_red\n");
		FPRINTF(f, "property float diffuse_green\n");
		FPRINTF(f, "property float diffuse_blue\n");
	}
	if (!mesh->colors.empty() && !float_color) {
		FPRINTF(f, "property uchar diffuse_red\n");
		FPRINTF(f, "property uchar diffuse_green\n");
		FPRINTF(f, "property uchar diffuse_blue\n");
	}
	if (!mesh->confidences.empty()) {
		FPRINTF(f, "property float confidence\n");
	}
	if (write_grid) {
		int ngrid = mesh->grid_width * mesh->grid_height;
		FPRINTF(f, "element range_grid %d\n", ngrid);
		FPRINTF(f, "property list uchar int vertex_indices\n");
	} else if (write_tstrips) {
		FPRINTF(f, "element tristrips 1\n");
		FPRINTF(f, "property list int int vertex_indices\n");
	}
	FPRINTF(f, "end_header\n");
	return true;
}


// Write an ASCII ply file
static bool write_ply_ascii(TriMesh *mesh, FILE *f, bool write_norm,
	bool write_grid, bool float_color)
{
	if (write_norm)
		mesh->need_normals();

	bool write_tstrips = !write_grid && !mesh->tstrips.empty();

	if (!write_ply_header(mesh, f, "ascii", write_grid, write_tstrips,
	                      write_norm, float_color))
		return false;
	if (!write_verts_asc(mesh, f, "", write_norm ? " " : 0, " ",
	                     float_color, " ", ""))
		return false;
	if (write_grid) {
		return write_grid_asc(mesh, f);
	} 
	// else write faces
	return true;
}


// Write a binary ply file
static bool write_ply_binary(TriMesh *mesh, FILE *f, bool little_endian,
                             bool write_norm, bool write_grid, bool float_color)
{
	if (write_norm)
		mesh->need_normals();

	bool write_tstrips = !write_grid && !mesh->tstrips.empty();
	bool need_swap = little_endian ^ we_are_little_endian();

	const char *format = little_endian ?
		"binary_little_endian" : "binary_big_endian";
	if (!write_ply_header(mesh, f, format, write_grid, write_tstrips,
	                      write_norm, float_color))
		return false;
	if (!write_verts_bin(mesh, f, need_swap, write_norm, true,
	                     float_color, true))
		return false;
	if (write_grid) {
		return write_grid_bin(mesh, f, need_swap);
	}
	return true;
}

// Convert colors float -> uchar
static unsigned char color2uchar(float p)
{
	return min(max(int(255.0f * p + 0.5f), 0), 255);
}

// Write a bunch of vertices to an ASCII file
static bool write_verts_asc(TriMesh *mesh, FILE *f,
                            const char *before_vert,
                            const char *before_norm,
                            const char *before_color,
                            bool float_color,
                            const char *before_conf,
                            const char *after_line)
{
    for (size_t i = 0; i < mesh->vertices.size(); i++) {
		FPRINTF(f, "%s%.7g %.7g %.7g", before_vert,
				mesh->vertices[i][0],
				mesh->vertices[i][1],
				mesh->vertices[i][2]);
		if (!mesh->normals.empty() && before_norm)
			FPRINTF(f, "%s%.7g %.7g %.7g", before_norm,
				mesh->normals[i][0],
				mesh->normals[i][1],
				mesh->normals[i][2]);
		if (!mesh->colors.empty() && before_color && float_color)
			FPRINTF(f, "%s%.7g %.7g %.7g", before_color,
				mesh->colors[i][0],
				mesh->colors[i][1],
				mesh->colors[i][2]);
		if (!mesh->colors.empty() && before_color && !float_color)
			FPRINTF(f, "%s%d %d %d", before_color,
				color2uchar(mesh->colors[i][0]),
				color2uchar(mesh->colors[i][1]),
				color2uchar(mesh->colors[i][2]));
		if (!mesh->confidences.empty() && before_conf)
			FPRINTF(f, "%s%.7g", before_conf, mesh->confidences[i]);
		FPRINTF(f, "%s\n", after_line);
	}
	return true;
}


// Byte-swap vertex properties
static void swap_vert_props(TriMesh *mesh, bool swap_color)
{
	for (size_t i = 0; i < mesh->vertices.size(); i++) {
		swap_float(mesh->vertices[i][0]);
		swap_float(mesh->vertices[i][1]);
		swap_float(mesh->vertices[i][2]);
	}
	if (!mesh->normals.empty()) {
		for (size_t i = 0; i < mesh->normals.size(); i++) {
			swap_float(mesh->normals[i][0]);
			swap_float(mesh->normals[i][1]);
			swap_float(mesh->normals[i][2]);
		}
	}
	if (!mesh->colors.empty() && swap_color) {
		for (size_t i = 0; i < mesh->normals.size(); i++) {
			swap_float(mesh->colors[i][0]);
			swap_float(mesh->colors[i][1]);
			swap_float(mesh->colors[i][2]);
		}
	}
	if (!mesh->confidences.empty()) {
		for (size_t i = 0; i < mesh->confidences.size(); i++)
			swap_float(mesh->confidences[i]);
	}
}


// Helper for write_verts_bin: actually does the writing.
static bool write_verts_bin_helper(TriMesh *mesh, FILE *f,
                                   bool write_norm, bool write_color,
                                   bool float_color, bool write_conf)
{
	if ((mesh->normals.empty() || !write_norm) &&
	    (mesh->colors.empty() || !write_color) &&
	    (mesh->confidences.empty() || !write_conf)) {
		// Optimized vertex-only code
		FWRITE(&(mesh->vertices[0][0]), 12*mesh->vertices.size(), 1, f);
	} else {
		// Generic code
		for (size_t i = 0; i < mesh->vertices.size(); i++) {
			FWRITE(&(mesh->vertices[i][0]), 12, 1, f);
			if (!mesh->normals.empty() && write_norm)
				FWRITE(&(mesh->normals[i][0]), 12, 1, f);
			if (!mesh->colors.empty() && write_color && float_color)
				FWRITE(&(mesh->colors[i][0]), 12, 1, f);
			if (!mesh->colors.empty() && write_color && !float_color) {
				unsigned char c[3] = {
					color2uchar(mesh->colors[i][0]),
					color2uchar(mesh->colors[i][1]),
					color2uchar(mesh->colors[i][2]) };
				FWRITE(&c, 3, 1, f);
			}
			if (!mesh->confidences.empty() && write_conf)
				FWRITE(&(mesh->confidences[i]), 4, 1, f);
		}
	}
	return true;
}


// Write a bunch of vertices to a binary file
static bool write_verts_bin(TriMesh *mesh, FILE *f, bool need_swap,
                            bool write_norm, bool write_color,
                            bool float_color, bool write_conf)
{
	if (need_swap)
		swap_vert_props(mesh, float_color);
	bool ok = write_verts_bin_helper(mesh, f, write_norm, write_color,
		float_color, write_conf);
	if (need_swap)
		swap_vert_props(mesh, float_color);
	return ok;
}

// Write tstrips to an ASCII file
static bool write_strips_asc(TriMesh *mesh, FILE *f)
{
	for (size_t i = 0; i < mesh->tstrips.size(); i++) {
		FPRINTF(f, "%d ", mesh->tstrips[i]);
	}
	FPRINTF(f, "\n");
	return true;
}


// Write tstrips to a binary file
static bool write_strips_bin(TriMesh *mesh, FILE *f, bool need_swap)
{
	if (need_swap) {
		for (size_t i = 0; i < mesh->tstrips.size(); i++)
			swap_int(mesh->tstrips[i]);
	}
	bool ok = (fwrite(&(mesh->tstrips[0]), 4*mesh->tstrips.size(), 1, f) == 1);
	if (need_swap) {
		for (size_t i = 0; i < mesh->tstrips.size(); i++)
			swap_int(mesh->tstrips[i]);
	}
	return ok;
}


// Write range grid to an ASCII file
static bool write_grid_asc(TriMesh *mesh, FILE *f)
{
	for (size_t i = 0; i < mesh->grid.size(); i++) {
		if (mesh->grid[i] < 0) {
			FPRINTF(f, "0\n");
		} else {
			FPRINTF(f, "1 %d\n", mesh->grid[i]);
		}
	}
	return true;
}


// Write range grid to a binary file
static bool write_grid_bin(TriMesh *mesh, FILE *f, bool need_swap)
{
	unsigned char zero = 0;
	unsigned char one = 1;
	for (size_t i = 0; i < mesh->grid.size(); i++) {
		if (mesh->grid[i] < 0) {
			FWRITE(&zero, 1, 1, f);
		} else {
			FWRITE(&one, 1, 1, f);
			int g = mesh->grid[i];
			if (need_swap)
				swap_int(g);
			FWRITE(&g, 4, 1, f);
		}
	}
	return true;
}


// Debugging printout, controllable by a "verbose"ness parameter, and
// hookable for GUIs
#undef dprintf

int TriMesh::verbose = 1;

void TriMesh::set_verbose(int verbose_)
{
	verbose = verbose_;
}

void (*TriMesh::dprintf_hook)(const char *) = NULL;

void TriMesh::set_dprintf_hook(void (*hook)(const char *))
{
	dprintf_hook = hook;
}

void TriMesh::dprintf(const char *format, ...)
{
	if (!verbose)
		return;

	va_list ap;
	va_start(ap, format);
	size_t required = 1 + vsnprintf(NULL, 0, format, ap);
	va_end(ap);
	char *buf = new char[required];
	va_start(ap, format);
	vsnprintf(buf, required, format, ap);
	va_end(ap);

	if (dprintf_hook) {
		dprintf_hook(buf);
	} else {
		fprintf(stderr, "%s", buf);
		fflush(stderr);
	}
	delete [] buf;
}


// Same as above, but fatal-error printout
#undef eprintf

void (*TriMesh::eprintf_hook)(const char *) = NULL;

void TriMesh::set_eprintf_hook(void (*hook)(const char *))
{
	eprintf_hook = hook;
}

void TriMesh::eprintf(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	size_t required = 1 + vsnprintf(NULL, 0, format, ap);
	va_end(ap);
	char *buf = new char[required];
	va_start(ap, format);
	vsnprintf(buf, required, format, ap);
	va_end(ap);

	if (eprintf_hook) {
		eprintf_hook(buf);
	} else {
		fprintf(stderr, "%s", buf);
		fflush(stderr);
	}
	delete [] buf;
}

} // namespace trimesh
