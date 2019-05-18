#pragma once
#ifndef _POLYHEDRON_IOSTREAM_IO_STL
#define _POLYHEDRON_IOSTREAM_IO_STL
#include "stl_scanner.h"
#include <iostream>

template<class TPoly>
class Importer_stl
{
public:
	static void Load(std::string name, std::istream& in, TPoly& P)
	{
		typedef typename TPoly::HalfedgeDS HalfedgeDS;
		typedef STL_Scanner<HalfedgeDS> Scanner;
		Scanner scanner(name, in);
		P.delegate(scanner);
	}
};

#endif // _POLYHEDRON_IOSTREAM_IO_STL