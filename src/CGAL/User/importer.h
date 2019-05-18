#pragma once
#ifndef CGAL_IMPORTER_H
#define CGAL_IMPORTER_H

#include <fstream>
#include "importer_stl.h"
#include "importer_memory.h"

#include "mesh_checker.h"
template <class TPoly>
class Mesh_Importer
{
public:
	static void Import(std::string& filename, TPoly& P)
	{
		if (FileExtension(filename, "ply"))
		{
		}
		else if (FileExtension(filename, "stl"))
		{
			std::ifstream in(filename, std::ios::in | std::ios::binary);
			Importer_stl<TPoly>::Load(filename, in, P);
		}
		else if (FileExtension(filename, "off"))
		{
			//std::ifstream in(filename);
			//in >> P;
		}
		else if (FileExtension(filename, "obj"))
		{
		}
		else if (FileExtension(filename, "vmi"))
		{
		}
		else
		{

		}

		Mesh_Checker<TPoly>::Check(P);
	}

	static void Import(unsigned ver_num, float* vertices, unsigned fac_num, unsigned* index, TPoly& p)
	{
		Memory_Importer<TPoly>::Build_From_Memory(ver_num, vertices, fac_num, index, p);
		Mesh_Checker<TPoly>::Check(p);
	}

private:
	static bool FileExtension(std::string filename, std::string extension)
	{
		std::transform(filename.begin(), filename.end(), filename.begin(), ::tolower);
		std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
		std::string end = filename.substr(filename.length() - extension.length(), extension.length());
		return end == extension;
	}
};
#endif // CGAL_IMPORTER_H