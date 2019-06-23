#include "load_calib.h"
#include "xform_io.h"
#include <iostream>

namespace trimesh
{
	bool load_camera_data_from_file(const std::string& file, CameraData& data)
	{
		XForm<double> KK_left;
		if (!read_xform(file.c_str(), KK_left))
		{
			std::cout << "读取标定数据KK_left_pr失败！！！" << std::endl;
			return false;
		}

		data.m_fx = float(KK_left(0, 0));
		data.m_fy = float(KK_left(1, 1));
		data.m_cx = float(KK_left(0, 2));
		data.m_cy = float(KK_left(1, 2));
		return true;
	}
}