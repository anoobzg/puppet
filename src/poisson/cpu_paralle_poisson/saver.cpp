#include "saver.h"

#include <fstream>
void save(unsigned m, std::vector<Eigen::Triplet<float>>& triplets, Eigen::VectorXf& B, Eigen::VectorXf& V, unsigned d)
{
	char file[128];
	sprintf(file, "E:\\Sim\\gpu_poisson\\matrix_depth_%d", d);

	std::fstream out;
	out.open(file, std::ios::out | std::ios::binary);
	if (out.is_open())
	{
		out.write((const char*)&m, sizeof(unsigned));
		unsigned size = (unsigned)triplets.size();
		out.write((const char*)&size, sizeof(unsigned));
		for (unsigned i = 0; i < size; ++i)
		{
			Eigen::Triplet<float>& t = triplets.at(i);
			out.write((const char*)&t.row(), sizeof(unsigned));
			out.write((const char*)&t.col(), sizeof(unsigned));
			out.write((const char*)&t.value(), sizeof(float));
		}
		for (unsigned i = 0; i < m; ++i)
		{
			float b = B(i);
			out.write((const char*)&b, sizeof(float));
		}
		for (unsigned i = 0; i < m; ++i)
		{
			float v = V(i);
			out.write((const char*)&v, sizeof(float));
		}
	}
	out.close();
}

void save_table(unsigned res, double* data, const char* file)
{
	std::fstream out;
	out.open(file, std::ios::out | std::ios::binary);
	if (out.is_open())
	{
		out.write((const char*)&res, sizeof(unsigned));
		out.write((const char*)data, res * res * sizeof(double));
	}
	out.close();
}