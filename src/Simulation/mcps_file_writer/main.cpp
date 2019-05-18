#include <iostream>
#include <fstream>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>
struct Point
{
	float x;
	float y;
	float nx;
	float ny;
};

void generate(std::vector<Point>& points, float x, float y, float r, unsigned n)
{
	points.resize(n);
	double delta_theta = 2.0 * M_PI / (double)n;
	for (unsigned i = 0; i < n; ++i)
	{
		double theta = (double)i * delta_theta;
		Point& p = points[i];
		p.nx = (float)cos(theta);
		p.ny = (float)sin(theta);
		p.x = x + p.nx * r;
		p.y = y + p.ny * r;
	}
}

int main(int argc, char* argv[])
{
	if (argc < 2)
		return EXIT_FAILURE;

	float x = 300.0f;
	if (argc >= 3) x = (float)atof(argv[2]);
	float y = 300.0f;
	if (argc >= 4) y = (float)atof(argv[3]);
	float r = 200.0f;
	if (argc >= 5) r = (float)atof(argv[4]);

	unsigned n = 200;
	if (argc >= 6) n = (unsigned)atoi(argv[5]);

	std::vector<Point> points;
	generate(points, x, y, r, n);

	std::fstream out;
	out.open(argv[1], std::ios::out | std::ios::binary);
	if (out.is_open())
	{
		unsigned size = (unsigned)points.size();
		out.write((const char*)&size, sizeof(unsigned));
		for (unsigned i = 0; i < size; ++i)
		{
			Point& p = points[i];
			out.write((const char*)&p, 4 * sizeof(float));
		}
	}
	
	out.close();
	return EXIT_SUCCESS;
}