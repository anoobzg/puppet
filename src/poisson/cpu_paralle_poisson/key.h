#pragma once
#include "predefine.h"

static unsigned shiff_bite[11][3] =
{
	0, 0, 0,
	31, 30, 29,
	28, 27, 26,
	25, 24, 23,
	22, 21, 20,
	19, 18, 17,
	16, 15, 14,
	13, 12, 11,
	10, 9, 8,
	7, 6, 5,
	4, 3, 2
};

void key_write_right(key& k, unsigned depth, unsigned dir);
void key_write_left(key& k, unsigned depth, unsigned dir);

struct index_key_compare
{
	bool operator()(const index_key& a, const index_key& b) const
	{
		return a.k < b.k;
	}
};

inline unsigned depth_key(key k, unsigned depth)
{
	return (k >> shiff_bite[depth][2])%8;
}

inline unsigned parent_key(key k, unsigned depth)
{
	if (depth == 1) return 0;
	unsigned d = (1 << (35 - 3 * depth)) - 1;
	return k &= ~d;
}

inline unsigned child_key(key pk, unsigned depth, unsigned c)
{
	return pk |= (c << shiff_bite[depth][2]);
}

inline bool same_parent(key k1, key k2, unsigned depth)
{
	unsigned p1 = parent_key(k1, depth);
	unsigned p2 = parent_key(k2, depth);
	return p1 == p2;
}

void key2index(key k, unsigned& xi, unsigned& yi, unsigned& zi, unsigned depth);

static unsigned parent_lu_table[8][27] = 
{
	0,	1,	1,	3,	4,	4,	3,	4,	4,	9,	10,	10,	12,	13,	13,	12,	13,	13,	9,	10,	10,	12,	13,	13,	12,	13,	13,
	1,	1,	2,	4,	4,	5,	4,	4,	5,	10,	10,	11,	13,	13,	14,	13,	13,	14,	10,	10,	11,	13,	13,	14,	13,	13,	14,
	3,	4,	4,	3,	4,	4,	6,	7,	7,	12,	13,	13,	12,	13,	13,	15,	16,	16,	12,	13,	13,	12,	13,	13,	15,	16,	16,
	4,	4,	5,	4,	4,	5,	7,	7,	8,	13,	13,	14,	13,	13,	14,	16,	16,	17,	13,	13,	14,	13,	13,	14,	16,	16,	17,
	9,	10,	10,	12,	13,	13,	12,	13,	13,	9,	10,	10,	12,	13,	13,	12,	13,	13,	18,	19,	19,	21,	22,	22,	21,	22,	22,
	10,	10,	11,	13,	13,	14,	13,	13,	14,	10,	10,	11,	13,	13,	14,	13,	13,	14,	19,	19,	20,	22,	22,	23,	22,	22,	23,
	12,	13,	13,	12,	13,	13,	15,	16,	16,	12,	13,	13,	12,	13,	13,	15,	16,	16,	21,	22,	22,	21,	22,	22,	24,	25,	25,
	13,	13,	14,	13,	13,	14,	16,	16,	17,	13,	13,	14,	13,	13,	14,	16,	16,	17,	22,	22,	23,	22,	22,	23,	25,	25,	26
};

static unsigned child_lu_table[8][27] = 
{
	7,	6,	7,	5,	4,	5,	7,	6,	7,	3,	2,	3,	1,	0,	1,	3,	2,	3,	7,	6,	7,	5,	4,	5,	7,	6,	7,
	6,	7,	6,	4,	5,	4,	6,	7,	6,	2,	3,	2,	0,	1,	0,	2,	3,	2,	6,	7,	6,	4,	5,	4,	6,	7,	6,
	5,	4,	5,	7,	6,	7,	5,	4,	5,	1,	0,	1,	3,	2,	3,	1,	0,	1,	5,	4,	5,	7,	6,	7,	5,	4,	5,
	4,	5,	4,	6,	7,	6,	4,	5,	4,	0,	1,	0,	2,	3,	2,	0,	1,	0,	4,	5,	4,	6,	7,	6,	4,	5,	4,
	3,	2,	3,	1,	0,	1,	3,	2,	3,	7,	6,	7,	5,	4,	5,	7,	6,	7,	3,	2,	3,	1,	0,	1,	3,	2,	3,
	2,	3,	2,	0,	1,	0,	2,	3,	2,	6,	7,	6,	4,	5,	4,	6,	7,	6,	2,	3,	2,	0,	1,	0,	2,	3,	2,
	1,	0,	1,	3,	2,	3,	1,	0,	1,	5,	4,	5,	7,	6,	7,	5,	4,	5,	1,	0,	1,	3,	2,	3,	1,	0,	1,
	0,	1,	0,	2,	3,	2,	0,	1,	0,	4,	5,	4,	6,	7,	6,	4,	5,	4,	0,	1,	0,	2,	3,	2,	0,	1,	0
};

static unsigned vertex_neighbors_table[8][8] = 
{
	0, 1, 3, 4, 9, 10, 12, 13,
	1, 2, 4, 5, 10, 11, 13, 14,
	3, 4, 6, 7, 12, 13, 15, 16,
	4, 5, 7, 8, 13, 14, 16, 17,
	9, 10, 12, 13, 18, 19, 21, 22,
	10, 11, 13, 14, 19, 20, 22, 23,
	12, 13, 15, 16, 21, 22, 24, 25,
	13, 14, 16, 17, 22, 23, 25, 26
};

static unsigned edge_neighbors_table[12][4] = 
{
	1, 4, 10, 13,
	3, 4, 12, 13,
	4, 5, 13, 14,
	4, 7, 13, 16,
	10, 13, 19, 22,
	12, 13, 21, 22,
	13, 14, 22, 23,
	13, 16, 22, 25,
	9, 10, 12, 13,
	10, 11, 13, 14,
	12, 13, 15, 16,
	13, 14, 16, 17
};

static unsigned face_neighbors_table[6][2] = 
{
	4, 13,
	13, 22,
	10, 13,
	12, 13,
	14, 13,
	13, 16
};