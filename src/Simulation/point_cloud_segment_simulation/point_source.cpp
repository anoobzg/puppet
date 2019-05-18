#include "point_source.h"
#include <fstream>

PointSource::PointSource()
{
	m_point_cloud.reset(new PointCloud());
	m_normal_cloud.reset(new NormalCloud());
	m_kdtree.reset(new KdTree());
}

PointSource::~PointSource()
{

}

bool PointSource::Load(const char* file_name)
{
	std::ifstream stream;
	stream.open(file_name, std::ios::in | std::ios::binary);
	if (!stream.is_open())
	{
		stream.close();
		return false;
	}

	unsigned frame_size = 0;
	stream.read((char*)&frame_size, sizeof(unsigned));
	if (frame_size <= 0)
	{
		stream.close();
		return false;
	}

	for (unsigned i = 0; i < frame_size; ++i)
	{
		float v[3];
		float m[9];
		unsigned id;
		float d;
		stream.read((char*)&v[0], 3 * sizeof(float));
		stream.read((char*)&id, sizeof(unsigned));
		stream.read((char*)&v[0], 3 * sizeof(float));
		stream.read((char*)&d, sizeof(float));
		stream.read((char*)&m[0], 9 * sizeof(float));
	}

	unsigned point_size = 0;
	stream.read((char*)&point_size, sizeof(unsigned));

	std::vector<short> indices;
	if (point_size > 0)
	{
		m_point_cloud->resize(point_size);
		m_normal_cloud->resize(point_size);

		indices.resize(point_size);

		for (unsigned i = 0; i < point_size; ++i)
		{//frame_indice
			stream.read((char*)&indices[i], sizeof(short));
		}
		for (unsigned i = 0; i < point_size; ++i)
		{//point
			stream.read((char*)&(m_point_cloud->at(i)), 3 * sizeof(float));
		}
		for (unsigned i = 0; i < point_size; ++i)
		{//point weight
			short sweight = 0;
			stream.read((char*)&sweight, sizeof(short));
		}
		for (unsigned i = 0; i < point_size; ++i)
		{//normal
			stream.read((char*)&(m_normal_cloud->at(i)), 3 * sizeof(float));
		}
		for (unsigned i = 0; i < point_size; ++i)
		{//point weight
			short sweight = 0;
			stream.read((char*)&sweight, sizeof(short));
		}
	}
	stream.close();

	m_kdtree->setInputCloud(m_point_cloud);
	return true;
}