#pragma once
#include <string>
#include <vector>
#include "Vec.h"

namespace esslam
{
	struct HandleScanImageData
	{
		int width;
		int height;
		int imageNum;
		std::vector<unsigned char> data;
	};

	struct BuildModelData
	{
		std::vector<trimesh::vec3> points;
		std::vector<trimesh::vec3> normals;
		std::vector<trimesh::vec3> colors;
		int width;
		int height;
		int num_effective;
		std::vector<int> grid;
	};

	struct PatchRenderData
	{
		std::vector<int> indices;
		std::vector<float> points;
		std::vector<float> normals;
		float xf[16];
		bool lost;
		int step;
	};

	struct FrameData
	{
		float xf[16];
		bool lost;
		std::vector<trimesh::vec3> position;
		std::vector<trimesh::vec3> normals;
		std::vector<float> distances;
	};

	struct NewAppendData
	{
		std::vector<trimesh::vec3> position;
		std::vector<trimesh::vec3> normals;
		std::vector < trimesh::Vec<3, unsigned char>> colors;
	};

	struct HHScanData
	{
		unsigned char* buffer;
		int buffer_size;
	};
}