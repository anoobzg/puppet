#include "MeshLoader.h"
#include <fstream>
#include "Mesh.h"
#include <map>
#include <vector>

#include <Windows.h>
#include <atlbase.h>
#include <atlconv.h>

//#include "plylib.h"
//#include <vcg/complex/complex.h>
//#include <wrap/io_trimesh/import.h>
//#include <wrap/io_trimesh/io_mask.h>
//
//class A2Vertex;
//class A2Face;
//class A2UsedTypes : public vcg::UsedTypes < vcg::Use<A2Vertex>::AsVertexType,
//	vcg::Use<A2Face  >::AsFaceType > {};
//
//class A2Vertex : public vcg::Vertex<A2UsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::Color4b, vcg::vertex::BitFlags> {};
//class A2Face : public vcg::Face< A2UsedTypes, vcg::face::VertexRef, vcg::face::Normal3f, vcg::face::Mark, vcg::face::BitFlags> {};
//class A2Mesh : public vcg::tri::TriMesh< std::vector<A2Vertex>, std::vector<A2Face> > {};

namespace LauncaGeometry
{

MeshLoader::MeshLoader()
{
}

MeshLoader::~MeshLoader()
{
}

Mesh* MeshLoader::LoadFromFileName(const wchar_t* file_name)
{
	std::wstring file = file_name;
	size_t pos = std::string::npos;
	if ((pos = file.rfind('.')) == std::wstring::npos)
		return 0;

	std::string extension = std::string(file.begin() + pos, file.end());
	if(!strcmp(extension.c_str(), ".bin"))
		return LoadFromBin(file);
	else if(!strcmp(extension.c_str(), ".stl"))
		return LoadFromSTL(file);
	else if (!strcmp(extension.c_str(), ".ply"))
		return LoadFromPLY(file);

	return 0;
}

Mesh* MeshLoader::LoadFromFileName(const char* file_name)
{
	std::string file = file_name;
	size_t pos = std::string::npos;
	if ((pos = file.rfind('.')) == std::wstring::npos)
		return 0;

	std::string extension = std::string(file.begin() + pos, file.end());
	if (!strcmp(extension.c_str(), ".bin"))
		return LoadFromBin(file);
	else if (!strcmp(extension.c_str(), ".stl"))
		return LoadFromSTL(file);
	else if (!strcmp(extension.c_str(), ".ply"))
		return LoadFromPLY(file);

	return 0;
}

Mesh* MeshLoader::LoadFromPLY(const std::wstring& mesh_file)
{
	std::ifstream in;
	in.open(mesh_file, std::ios::in | std::ios::binary);
	if (!in.is_open())
		return 0;

	std::string f/* = ::CW2A(mesh_file.c_str(), CP_UTF8)*/;
	return LoadFromPLY(f);
}

Mesh* MeshLoader::LoadFromPLY(const std::string& mesh_file)
{
	/*vcg::ply::PlyFile pf;

	A2Mesh vcg_mesh;
	int dummymask = 0;
	bool result = (0 == vcg::tri::io::ImporterPLY<A2Mesh>::Open(vcg_mesh, mesh_file.c_str(), dummymask));
	if (result && vcg_mesh.vn > 0 && vcg_mesh.fn > 0)
	{
		Mesh* mesh = new Mesh();
		mesh->AllocateVertex(vcg_mesh.vn);
		mesh->AllocateTriangle(vcg_mesh.fn);

		float* position = mesh->vertex_position;
		float* normal = mesh->vertex_normal;
		unsigned char* color = mesh->vertex_color;
		unsigned* triangle_index = mesh->triangle_index;

		vcg::SimpleTempData<typename A2Mesh::VertContainer, int> indices(vcg_mesh.vert);

		int j;
		std::vector<int> FlagV;
		A2Mesh::VertexPointer  vp;
		A2Mesh::VertexIterator vi;
		for (j = 0, vi = vcg_mesh.vert.begin(); vi != vcg_mesh.vert.end(); ++vi)
		{
			vp = &(*vi);
			indices[vi] = j;

			float t;

			*position++ = float(vp->P()[0]);
			*position++ = float(vp->P()[1]);
			*position++ = float(vp->P()[2]);

			*normal++ = float(vp->N()[0]);
			*normal++ = float(vp->N()[1]);
			*normal++ = float(vp->N()[2]);

			*color++ = (unsigned char)(vp->C()[0]);
			*color++ = (unsigned char)(vp->C()[1]);
			*color++ = (unsigned char)(vp->C()[2]);

			++j;
		}

		A2Mesh::FacePointer fp;
		A2Mesh::FaceIterator fi;
		for (j = 0, fi = vcg_mesh.face.begin(); fi != vcg_mesh.face.end(); ++fi)
		{
			fp = &(*fi);
			*triangle_index++ = indices[fp->cV(0)];
			*triangle_index++ = indices[fp->cV(1)];
			*triangle_index++ = indices[fp->cV(2)];
		}

		return mesh;
	}*/

	return 0;
}

Mesh* MeshLoader::LoadFromBin(const std::string& mesh_file)
{
	std::ifstream in;
	in.open(mesh_file, std::ios::in | std::ios::binary);
	if (!in.is_open())
		return 0;

	return LoadFromBin(in);
}

Mesh* MeshLoader::LoadFromBin(const std::wstring& mesh_file)
{
	std::ifstream in;
	in.open(mesh_file, std::ios::in | std::ios::binary);
	if (!in.is_open())
		return 0;

	return LoadFromBin(in);
}

Mesh* MeshLoader::LoadFromSTL(const std::string& mesh_file)
{
	std::ifstream in;
	in.open(mesh_file, std::ios::in | std::ios::binary);
	if (!in.is_open())
		return 0;

	return LoadFromSTL(in);
}

Mesh* MeshLoader::LoadFromSTL(std::ifstream& in)
{
	char header[80];
	in.read((char*)header, 80 * sizeof(char));
	unsigned face_number;
	in.read((char*)&face_number, sizeof(unsigned));

	if (face_number == 0)
		return 0;

	Mesh* mesh = new Mesh();
	mesh->triangle_number = face_number;
	mesh->triangle_index = new unsigned[3 * face_number];
	mesh->triangle_char = new char[2 * face_number];
	struct point
	{
		float x;
		float y;
		float z;

		void operator+=(point& value)
		{
			x += value.x;
			y += value.y;
			z += value.z;
		}
	};

	class point_cmp
	{
	public:
		point_cmp(float e = FLT_MIN) :eps(e) {}

		bool operator()(const point& v0, const point& v1) const
		{
			if (fabs(v0.x - v1.x) <= eps)
			{
				if (fabs(v0.y - v1.y) <= eps)
				{
					return (v0.z < v1.z - eps);
				}
				else return (v0.y < v1.y - eps);
			}
			else return (v0.x < v1.x - eps);
		}
	private:
		float eps;
	};

	typedef std::map<point, unsigned, point_cmp> unique_point;
	typedef unique_point::iterator point_iterator;

	std::vector<point> coord;
	std::vector<point> normal;

	unique_point points;
	for (unsigned i = 0; i < face_number; ++i)
	{
		point n;
		in.read((char*)&n, 3 * sizeof(float));
		unsigned* triangle = mesh->triangle_index + 3 * i;
		char* triangle_char = mesh->triangle_char + 2 * i;
		for (unsigned j = 0; j < 3; ++j)
		{
			point p;
			in.read((char*)&p, 3 * sizeof(float));
			point_iterator it = points.find(p);
			if (it != points.end())
			{
				unsigned index = (*it).second;
				triangle[j] = index;
				normal[index] += n;
			}
			else
			{
				unsigned index = (unsigned)points.size();
				points.insert(unique_point::value_type(p, index));
				triangle[j] = index;
				coord.push_back(p);
				normal.push_back(n);
			}
		}

		in.read(triangle_char, 2);
	}


	mesh->vertex_number = (unsigned)coord.size();
	if (mesh->vertex_number)
	{
		mesh->vertex_position = new float[3 * mesh->vertex_number];
		memcpy(mesh->vertex_position, &coord[0], mesh->vertex_number * 3 * sizeof(float));
		mesh->vertex_normal = new float[3 * mesh->vertex_number];
		memcpy(mesh->vertex_normal, &normal[0], mesh->vertex_number * 3 * sizeof(float));
		mesh->vertex_color = new unsigned char[3 * mesh->vertex_number];
		memset(mesh->vertex_color, 0xff, 3 * mesh->vertex_number * sizeof(unsigned char));
		mesh->m_repair_triangle_append_index = mesh->triangle_number;
		mesh->m_repair_vertex_append_index = mesh->vertex_number;
	}

	for (unsigned i = 0; i < mesh->vertex_number; ++i)
	{
		float* np = mesh->vertex_normal + 3 * i;
		float d = np[0] * np[0] + np[1] * np[1] + np[2] * np[2];
		if (d > 0.0f)
		{
			float sd = sqrtf(d);
			np[0] /= sd; np[1] /= sd; np[2] /= sd;
		}
	}

	return mesh;
}

Mesh* MeshLoader::LoadFromSTL(const std::wstring& mesh_file)
{
	std::ifstream in;
	in.open(mesh_file, std::ios::in | std::ios::binary);
	if(!in.is_open())
		return 0;

	return LoadFromSTL(in);
}

Mesh* MeshLoader::LoadFromBin(std::ifstream& in)
{	
	{
		unsigned first = 0;
		in.read((char*)&first, sizeof(unsigned));
		char second[7];
		in.read(second, 7);
		if (first == 1988226 && !strcmp(second, "Launca"))
			return LoadFromMBin(in);
	}

	in.seekg(0, std::ios_base::end);
	unsigned buffer_size = (unsigned)in.tellg();
	in.seekg(0, std::ios_base::beg);
	
	unsigned char* buffer = new unsigned char[buffer_size];
	unsigned char* current = buffer;
	memset(buffer, 0, buffer_size);
	in.read((char*)buffer, buffer_size);
	
	in.close();
	
	__int32 type = *(__int32*)current;
	current += sizeof(__int32);
	
	ALG_ID algorithm_id = CALG_AES_128;
	std::string password = "F73D0B1F-789C-43AD-BDBD-232A497C4AC9";
	LPCTSTR providerName = MS_ENH_RSA_AES_PROV;
	DWORD providerType = PROV_RSA_AES;
	HCRYPTPROV hcr_provider = NULL;
	HCRYPTKEY key = NULL;
	HCRYPTHASH hash = NULL;
	
	BOOL result = FALSE;
	result = ::CryptAcquireContext(
		&hcr_provider,
		NULL,
		providerName,
		providerType,
		CRYPT_VERIFYCONTEXT);
	if(!result)
	{
		delete [] buffer;
		0;
	}
	
	//create the password hash
	result = ::CryptCreateHash(
		hcr_provider,
		CALG_MD5,
		0,
		0,
		&hash
		);
	if(!result)
	{
		delete [] buffer;
		return 0;
	}
	
	//hash the password
	result = ::CryptHashData(
		hash,
		(BYTE*)password.c_str(),
		(DWORD)password.size(),
		0
		);
	if(!result)
	{
		delete [] buffer;
		return 0;
	}
	
	//create the encryption key
	if(!::CryptDeriveKey(
		hcr_provider, 
        algorithm_id, 
        hash, 
        0x00800000, 
        &key))
	{
		delete [] buffer;
		return 0;
	}
	
	DWORD dwDataSize = (DWORD)(buffer_size - sizeof(__int32));
	if(!::CryptDecrypt(key, NULL, TRUE, 0, current, &dwDataSize))
	{
		delete [] buffer;
		return 0;
	}
	
	Mesh* mesh = new Mesh();
	buffer_size = sizeof(__int32) + (unsigned)dwDataSize;
	
	__int32 version = *(__int32*)current;
	current += sizeof(__int32);
	current += sizeof(__int32);
	
	DWORD format = *(DWORD*)current;
	current += sizeof(DWORD);
	//vertices
	size_t numVertices = *(size_t*)current;
	current += sizeof(size_t);
	
	mesh->vertex_number = (unsigned)numVertices;
	mesh->vertex_position = new float[3 * mesh->vertex_number];
	mesh->vertex_normal = new float[3 * mesh->vertex_number];
	mesh->vertex_color = new unsigned char[3 * mesh->vertex_number];
	memset(mesh->vertex_color, 0xff, 3 * mesh->vertex_number * sizeof(unsigned char));
	mesh->m_repair_vertex_append_index = mesh->vertex_number;

	for(unsigned i = 0; i < mesh->vertex_number; ++i)
	{
		mesh->vertex_position[3 * i] = *(float*)current;
		current += sizeof(float);
		mesh->vertex_position[3 * i + 1] = *(float*)current;
		current += sizeof(float);
		mesh->vertex_position[3 * i + 2] = *(float*)current;
		current += sizeof(float);
		mesh->vertex_normal[3 * i] = *(float*)current;
		current += sizeof(float);
		mesh->vertex_normal[3 * i + 1] = *(float*)current;
		current += sizeof(float);
		mesh->vertex_normal[3 * i + 2] = *(float*)current;
		current += sizeof(float);
	
		size_t neigbours = *(size_t*)current;
		current += sizeof(size_t);
		for(size_t j = 0; j < neigbours; ++j)
		{
			unsigned index = *(unsigned*)current;
			current += sizeof(unsigned);
		}
	}
	
	size_t triangle_number = *(size_t*)current;
	current += sizeof(size_t);
	mesh->triangle_number = (unsigned)triangle_number;
	mesh->triangle_index = new unsigned[3 * mesh->triangle_number];
	mesh->m_repair_triangle_append_index = mesh->triangle_number;

	for(unsigned i = 0; i < mesh->triangle_number; ++i)
	{
		mesh->triangle_index[3 * i] = *(unsigned*)current;
		current += sizeof(unsigned);
		mesh->triangle_index[3 * i + 1] = *(unsigned*)current;
		current += sizeof(unsigned);
		mesh->triangle_index[3 * i + 2] = *(unsigned*)current;
		current += sizeof(unsigned);
	}

	if(format == 2)
	{
		int triangle_append = *(int*)current;
		current += sizeof(int);
		int vertex_append = *(int*)current;
		current += sizeof(int);

		mesh->m_repair_vertex_append_index = (unsigned)vertex_append;
		mesh->m_repair_triangle_append_index = (unsigned)triangle_append;
	}

	size_t color_size = *(size_t*)current;
	current += sizeof(size_t);
	unsigned char* t_color = mesh->vertex_color;
	if(color_size == numVertices)
	{
		for (size_t i = 0; i < color_size; ++i)
		{
			*t_color++ = *current;
			current += sizeof(unsigned char);
			*t_color++ = *current;
			current += sizeof(unsigned char);
			*t_color++ = *current;
			current += sizeof(unsigned char);
		}
	}
	delete [] buffer;
	return mesh;
}

Mesh* MeshLoader::LoadFromMBin(std::ifstream& in)
{
	unsigned vertex_number = 0;
	unsigned triangle_number = 0;
	in.read((char*)&vertex_number, sizeof(unsigned));
	in.read((char*)&triangle_number, sizeof(unsigned));

	Mesh* mesh = 0;
	if (vertex_number > 0 && triangle_number > 0)
	{
		mesh = new Mesh();
		mesh->AllocateVertex(vertex_number);
		mesh->AllocateTriangle(triangle_number);

		unsigned repair_vertex = vertex_number;
		unsigned repair_triangle = triangle_number;
		in.read((char*)&repair_vertex, sizeof(unsigned));
		in.read((char*)&repair_triangle, sizeof(unsigned));
		mesh->m_repair_vertex_append_index = repair_vertex;
		mesh->m_repair_triangle_append_index = repair_triangle;

		in.read((char*)mesh->vertex_position, 3 * sizeof(float) * mesh->vertex_number);
		in.read((char*)mesh->vertex_normal, 3 * sizeof(float) * mesh->vertex_number);
		in.read((char*)mesh->vertex_color, 3 * sizeof(unsigned char) * mesh->vertex_number);
		in.read((char*)mesh->triangle_index, 3 * sizeof(unsigned) * mesh->triangle_number);
	}
	in.close();

	return mesh;
}

}