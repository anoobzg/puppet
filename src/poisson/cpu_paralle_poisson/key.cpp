#include "key.h"

void key_write_right(key& k, unsigned depth, unsigned dir)
{
	k |= (1 << shiff_bite[depth][dir]);
}

void key_write_left(key& k, unsigned depth, unsigned dir)
{
	k &= ~(1 << shiff_bite[depth][dir]);
}

void key2index(key k, unsigned& xi, unsigned& yi, unsigned& zi, unsigned depth)
{
	unsigned start = (1 << depth) - 1;
	unsigned dx = 0;
	unsigned dy = 0;
	unsigned dz = 0;
	for (unsigned d = depth; d >= 1; --d)
	{
		unsigned dk = depth_key(k, d);
		unsigned dkx = dk / 4;
		unsigned dky = (dk % 4) / 2;
		unsigned dkz = dk % 2;

		dx += dkx * (1 << (depth - d));
		dy += dky * (1 << (depth - d));
		dz += dkz * (1 << (depth - d));
	}
	xi = start + dx;
	yi = start + dy;
	zi = start + dz;
}
