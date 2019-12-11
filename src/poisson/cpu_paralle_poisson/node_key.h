#pragma once

inline void depthandoffset(unsigned index, unsigned& depth, unsigned& offset)
{
	int i = index + 1;
	depth = -1;
	while (i) {
		i >>= 1;
		depth++;
	}
	offset = (index + 1) - (1 << depth);
}

inline void centerandwidth(unsigned depth, unsigned offset, double& center, double& width)
{
	width = double(1.0 / (1 << depth));
	center = double((0.5 + offset)*width);
}

inline void index2centerandwidth(unsigned index, double& c, double& w)
{
	unsigned depth, offset;
	depthandoffset(index, depth, offset);
	centerandwidth(depth, offset, c, w);
}
