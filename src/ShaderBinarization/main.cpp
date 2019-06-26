#include <iostream>
#include <vector>
#include <boost\filesystem.hpp>
#include <set>
#include <algorithm>
#include <fstream>

#include "convert.h"

void parse_names(const boost::filesystem::path& input_path,
	std::vector<std::wstring>& names)
{
	std::set<std::wstring> names_set;
	boost::filesystem::recursive_directory_iterator beg_iter(input_path);
	boost::filesystem::recursive_directory_iterator end_iter;
	for (; beg_iter != end_iter; ++beg_iter)
	{
		if (boost::filesystem::is_directory(*beg_iter))
			continue;
		else
		{
			const boost::filesystem::path& file_path = beg_iter->path();
			if (boost::filesystem::is_regular_file(file_path))
			{
				boost::filesystem::path p = file_path.filename();
				std::wstring wname = p.wstring();
				size_t pos = wname.find(L'.');
				if (pos != std::wstring::npos)
				{
					std::wstring n = std::wstring(wname.begin(), wname.begin() + pos);
					names_set.insert(n);
				}
			}
		}
	}

	names.insert(names.end(), names_set.begin(), names_set.end());
}

void write_program(const boost::filesystem::path& output_path, const boost::filesystem::path& input_path,
	std::vector<std::wstring>& names)
{
	std::wstring program_def_file = output_path.wstring() + L"\\ProgramDef.h";
	std::wstring shader_string_file = output_path.wstring() + L"\\ShaderString.h";
	std::fstream program_def(program_def_file.c_str(), std::ios::out);
	std::fstream shader_string(shader_string_file.c_str(), std::ios::out);
	if (!program_def.is_open() || !shader_string.is_open())
	{
		std::cout << "ProgramDef.h or ShaderString.h open failed." << std::endl;

		program_def.close();
		shader_string.close();
		return;
	}

	program_def << "//auto generate!" << std::endl;
	shader_string << "//auto generate!" << std::endl;
	std::wstring ext[3] = { L".vert", L".geom", L".frag" };
	std::string span[3] = { "_vert", "_geom", "_frag" };
	for (size_t i = 0; i < names.size(); ++i)
	{
		const std::wstring& name = names.at(i);
		std::string utf_name;
		if (!unicode_2_utf8(name, utf_name))
			continue;

		program_def << "\t{\""<<utf_name.c_str()<<"\"";
		std::wstring base_path = input_path.wstring() + L"\\" + name;

		char temp[1024];
		for (int j = 0; j < 3; ++j)
		{
			std::string string_seg = "0";
			boost::filesystem::path p(base_path + ext[j]);
			if (boost::filesystem::exists(p))
			{
				string_seg = utf_name + span[j];
				shader_string << "static const char* " << string_seg << "=\n";
				std::fstream in(p.wstring().c_str(), std::ios::in);
				if (in.is_open())
				{
					while (!in.eof())
					{
						in.getline(temp, 1024);
						shader_string << "\"" << temp << "\\n\"\n";
					}
				}
				else
				{
					shader_string << "\"\"";
				}
				in.close();
				shader_string << ";\n";
			}
			program_def << "," << string_seg.c_str();
		}
		program_def << "}," << std::endl;
	}

	program_def.close();
	shader_string.close();
}

int main(int argc, char* argv[])
{
	if (argc < 3)
	{
		std::cout << "no input dir, no output dir." << std::endl;
		return EXIT_FAILURE;
	}

	boost::filesystem::path input_path(argv[1]);
	boost::filesystem::path output_path(argv[2]);

	std::cout << "input path " << input_path << std::endl;
	std::cout << "output path " << output_path << std::endl;

	if (!boost::filesystem::is_directory(input_path) || !boost::filesystem::is_directory(output_path))
	{
		std::cout << input_path << " not exist" << std::endl;
		std::cout << output_path << " not exist" << std::endl;
		return EXIT_FAILURE;
	}
	std::vector<std::wstring> names;
	parse_names(input_path, names);
	if (names.size() == 0)
	{
		std::cout << "there is no shader in " << input_path << std::endl;
		return EXIT_FAILURE;
	}

	write_program(output_path, input_path, names);

	return EXIT_SUCCESS;
}