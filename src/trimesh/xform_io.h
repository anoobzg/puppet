#pragma once
#include "Xform.h"
#include <fstream>

namespace trimesh
{
	template<class T>
	bool read_xform(const char* file, XForm<T>& matrix)
	{
		std::ifstream ifs(file, std::ios::binary);
		if (!ifs) 
		{
			ifs.close();
			return false;
		}

		int crow, ccol;
		int i, j;
		int RealSize = sizeof(T);
		ifs.read((char*)(&crow), sizeof(crow));
		ifs.read((char*)(&ccol), sizeof(ccol));

		for (i = 0; i < crow; i++)
			for (j = 0; j < ccol; j++)
				ifs.read((char*)(&(matrix(i, j))), RealSize);

		ifs.close();
		return true;
	}
}