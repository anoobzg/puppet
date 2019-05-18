#include "TransformLoader.h"
#include <fstream>
#include <sstream>

namespace LauncaGeometry
{

bool TransformLoader::LoadFromFileName(GemTransform& transform, const wchar_t* file_name)
{
	std::wstring file = file_name;
	size_t pos = std::wstring::npos;
	if ((pos = file.rfind('.')) == std::string::npos)
		return false;

	std::string extension = std::string(file.begin() + pos, file.end());
	if(!strcmp(extension.c_str(), ".bin") || !strcmp(extension.c_str(), ".trs"))
	{
		std::ifstream in;
		in.open(file, std::ios::in);
		if(!in.is_open())
			return false;

		char rot_str[512];
		char pan_str[512];
		in.getline(rot_str, 512);
		in.getline(pan_str, 512);


		std::stringstream rot_stream(rot_str, std::stringstream::in);
		rot_stream >> transform.m_matrix[0];
		rot_stream >> transform.m_matrix[4];
		rot_stream >> transform.m_matrix[8];
		rot_stream >> transform.m_matrix[1];
		rot_stream >> transform.m_matrix[5];
		rot_stream >> transform.m_matrix[9];
		rot_stream >> transform.m_matrix[2];
		rot_stream >> transform.m_matrix[6];
		rot_stream >> transform.m_matrix[10];

		std::stringstream strmTranslation(pan_str, std::stringstream::in);

		strmTranslation >> transform.m_matrix[12];
		strmTranslation >> transform.m_matrix[13];
		strmTranslation >> transform.m_matrix[14];

		return true;
	}

	return false;
}

}