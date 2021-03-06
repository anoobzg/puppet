#include "load_obj.h"
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>
#include <fstream>
#include <iostream>

#include<vcg/complex/complex.h>
#include<wrap/io_trimesh/import_obj.h>

class MyVertex; class MyEdge; class MyFace;
struct MyUsedTypes : public vcg::UsedTypes<vcg::Use<MyVertex>   ::AsVertexType,
	vcg::Use<MyEdge>     ::AsEdgeType,
	vcg::Use<MyFace>     ::AsFaceType> {};

class MyVertex : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags  > {};
class MyFace : public vcg::Face<   MyUsedTypes, vcg::face::FFAdj, vcg::face::VertexRef, vcg::face::BitFlags > {};
class MyEdge : public vcg::Edge<   MyUsedTypes> {};

class MyMesh : public vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace>, std::vector<MyEdge>  > {};

class MyVertex0 : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::BitFlags  > {};
class MyVertex1 : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags  > {};
class MyVertex2 : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Color4b, vcg::vertex::CurvatureDirf,
	vcg::vertex::Qualityf, vcg::vertex::Normal3f, vcg::vertex::BitFlags  > {};

osg::Geometry* LoadFromVCG(const std::string& file)
{
	MyMesh m;

	int load_mask = 0;
	if (vcg::tri::io::ImporterOBJ<MyMesh>::Open(m, file.c_str(), load_mask, NULL) >= vcg::tri::io::ImporterOBJ<MyMesh>::E_CANTOPEN)
	{
		printf("Error reading file  %s\n", file.c_str());
		return 0;
	}

	return 0;
}

osg::Geometry* LoadObj(const std::string& file)
{
	return LoadFromVCG(file);

	std::fstream in(file.c_str(), std::ios::in);
	if (!in.good()) return 0;

	char buffer[256];
	std::vector<osg::Vec3f> temp_coord;
	std::vector<osg::Vec3f> temp_normal;
	std::vector<osg::Vec2f> temp_texcoord;
	std::vector<osg::Vec3i> temp_primitive;
	std::vector<osg::Vec3i> temp_primitive_texcoord;
	float f1, f2, f3;
	int v1[3];
	int v2[3];
	int v3[3];

	while (!in.getline(buffer, 255).eof())
	{
		if (buffer[0] == 'v' && (buffer[1] == ' ' || buffer[1] == 32))
		{
			sscanf_s(buffer, "v %f %f %f", &f1, &f2, &f3);
			temp_coord.push_back(osg::Vec3f(f1, f2, f3));
		}
		else if (buffer[0] == 'v' && buffer[1] == 'n' && (buffer[2] == ' ' || buffer[2] == 32))
		{
			sscanf_s(buffer, "vn %f %f %f", &f1, &f2, &f3);
			temp_normal.push_back(osg::Vec3f(f1, f2, f3));
		}
		else if (buffer[0] == 'v'  && buffer[1] == 't' && (buffer[2] == ' ' || buffer[2] == 32))
		{
			sscanf_s(buffer, "vt %f %f", &f1, &f2);
			temp_texcoord.push_back(osg::Vec2f(f1, f2));
		}
		else if (buffer[0] == 'f'  && (buffer[1] == ' ' || buffer[1] == 32))
		{
			sscanf_s(buffer, "f %d/%d/%d %d/%d/%d %d/%d/%d", &v1[0], &v1[1], &v1[2], &v2[0], &v2[1], &v2[2], & v3[0], &v3[1], &v3[2]);
			
			if (v1[0] != v1[2] || v2[0] != v2[2] || v3[0] != v3[2])
				std::cout << "error face." << std::endl;

			temp_primitive.push_back(osg::Vec3i(v1[0], v2[0], v3[0]));
			temp_primitive_texcoord.push_back(osg::Vec3i(v1[1], v2[1], v3[1]));
		}
	}

	size_t tsize = temp_coord.size();
	if (temp_coord.size() == 0) return 0;
	if (temp_texcoord.size() == 0) return 0;

	size_t size = temp_texcoord.size();

	osg::Vec3Array* coord_array = new osg::Vec3Array();
	osg::Vec3Array* normal_array = new osg::Vec3Array();
	osg::Vec2Array* texcoord_array = new osg::Vec2Array();
	coord_array->resize(tsize);
	normal_array->resize(tsize);
	texcoord_array->resize(tsize);
	osg::DrawElementsUInt* primitive_set = new osg::DrawElementsUInt(GL_POINTS);

	for (size_t i = 0; i < tsize; ++i)
	{
		coord_array->at(i) = temp_coord.at(i);
		normal_array->at(i) = temp_normal.at(i);
	}

	//std::vector<bool> setted(size, false);
	primitive_set->reserve(3 * temp_primitive.size());
	for (size_t i = 0; i < tsize; ++i)
	{
		//const osg::Vec3i& vec = temp_primitive.at(i);
		//if (vec.x() < tsize && vec.y() < tsize && vec.z() < tsize)
		//{
		//	primitive_set->push_back(vec.x());
		//	primitive_set->push_back(vec.y());
		//	primitive_set->push_back(vec.z());
		//}
		primitive_set->push_back((int)i);
	}
	
	//size_t psize = temp_primitive.size();
	//for (size_t i = 0; i < size; ++i)
	//	texcoord_array->at(i) = temp_texcoord.at(i);
	//
	//for (size_t i = 0; i < psize; ++i)
	//{
	//	const osg::Vec3i& tcprimitive = temp_primitive_texcoord.at(i);
	//	const osg::Vec3i& tprimitive = temp_primitive.at(i);
	//
	//	primitive_set->push_back(tcprimitive.x());
	//	primitive_set->push_back(tcprimitive.y());
	//	primitive_set->push_back(tcprimitive.z());
	//
	//	for (int j = 0; j < 3; ++j)
	//	{
	//		int index1 = tcprimitive[j];
	//		int index2 = tprimitive[j];
	//
	//		if (!setted[index1] && index2 < temp_coord.size())
	//		{
	//			coord_array->at(index1) = temp_coord.at(index2);
	//			normal_array->at(index1) = temp_normal.at(index2);
	//			setted[index1] = true;
	//		}
	//	}
	//}

	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(primitive_set, coord_array, normal_array, texcoord_array);
	return geometry;
}