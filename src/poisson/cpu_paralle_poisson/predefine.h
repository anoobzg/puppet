#pragma once
#include <vector>

struct vec3
{
	float x;
	float y;
	float z;
};

inline float dot(const vec3& v, const vec3& u)
{
	return v.x * u.x + v.y * u.y + v.z * u.z;
}

inline void cross(const vec3& v1, const vec3& v2, vec3& v3)
{
	v3.x = v1.y * v2.z - v1.z * v2.y;
	v3.y = v1.z * v2.x - v1.x * v2.z;
	v3.z = v1.x * v2.y - v1.y * v2.x;
}

struct point
{
	vec3 position;
	vec3 normal;
};

typedef unsigned key;

struct index_key
{
	key k;
	unsigned index;
};

struct core_node
{
	key k;
	unsigned start_index;
	unsigned point_number;

	int parent;
	int children[8];

	unsigned n_start_index;
	unsigned n_number;
	core_node()
	{
		k = 0;
		start_index = 0;
		point_number = 0;
		parent = -1;
		children[0] = -1;
		children[1] = -1;
		children[2] = -1;
		children[3] = -1;
		children[4] = -1;
		children[5] = -1;
		children[6] = -1;
		children[7] = -1;

		n_start_index = 0;
		n_number = 0;
	}
};

struct depth_info
{
	unsigned start_index;
	unsigned node_number;
};

struct offset
{
	unsigned x;
	unsigned y;
	unsigned z;
};

struct node
{
	core_node n;
	int neighbors[27];

	depth_info vertices;
	depth_info edges;

	float w;
	vec3 c;
	node()
	{
		memset(neighbors, -1, 27 * sizeof(int));
	}
};

struct node_vertex
{
	key vkey[8];
	int nodes[8][8];
	int nindex;
	node_vertex()
	{
		memset(vkey, -1, 8 * sizeof(int));
		memset(nodes, -1, 8 * 8 * sizeof(int));
		nindex = -1;
	}
};

struct vertex
{
	int nodes[8];
	vec3 c;
	vertex()
	{
		memset(nodes, -1, 8 * sizeof(int));
	}
};

struct node_edge
{
	key ekey[12];
	int nodes[12][4];
	int nindex;
	node_edge()
	{
		memset(ekey, -1, 4 * sizeof(int));
		memset(nodes, -1, 12 * 4 * sizeof(int));
		nindex = -1;
	}
};

struct edge
{
	int nodes[4];
	edge()
	{
		memset(nodes, -1, 4 * sizeof(int));
	}
};

struct node_face
{
	key fkey[6];
	int nodes[6][2];
	int nindex;
	node_face()
	{
		memset(fkey, -1, 6 * sizeof(int));
		memset(nodes, -1, 6 * 2 * sizeof(int));
		nindex = -1;
	}
};

struct face
{
	int nodes[2];
	face()
	{
		memset(nodes, -1, 2 * sizeof(int));
	}
};