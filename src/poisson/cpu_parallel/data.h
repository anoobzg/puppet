#pragma once
#include <Vec.h>
#include <Box.h>
#include <TriMesh_algo.h>

#include <vector>

typedef unsigned octree_key;

struct index_key
{
	octree_key key;
	unsigned index;
};

struct index_key_compare
{
	bool operator()(const index_key& a, const index_key& b) const
	{
		return a.key < b.key;
	}
};

struct unique_node
{
	octree_key key;
	int start_index;
	int point_number;
};

struct octree_node
{

};

struct octree_vertex
{

};

struct octree_edge
{

};

struct octree_face
{

};


