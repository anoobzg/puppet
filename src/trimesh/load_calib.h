#pragma once
#include <string>

namespace trimesh
{
	struct CameraData
	{
		float m_fx;
		float m_fy;
		float m_cx;
		float m_cy;
	};

	extern bool load_camera_data_from_file(const std::string& file, CameraData& data);
}