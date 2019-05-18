#include "MeshSaver.h"
#include <fstream>
#include <Windows.h>

#include "Mesh.h"
namespace LauncaGeometry
{

bool MeshSaver::Save(const wchar_t* mesh_file, Mesh& mesh)
{
	std::wstring file = mesh_file;
	size_t pos = std::string::npos;
	if ((pos = file.rfind('.')) == std::string::npos)
	{
		return false;
	}

	std::string extension = std::string(file.begin() + pos, file.end());
	if (!strcmp(extension.c_str(), ".stl"))
		return SaveToSTL(file, mesh);
	else if (!strcmp(extension.c_str(), ".bin"))
		return SaveToBin(file, mesh);
	else if (!strcmp(extension.c_str(), ".ply"))
		return SaveToPLY(file, mesh);
	return false;
}

bool MeshSaver::Save(const char* mesh_file, Mesh& mesh)
{
	std::string file = mesh_file;
	size_t pos = std::string::npos;
	if ((pos = file.rfind('.')) == std::string::npos)
	{
		return false;
	}

	std::string extension = std::string(file.begin() + pos, file.end());
	if (!strcmp(extension.c_str(), ".stl"))
		return SaveToSTL(file, mesh);
	else if (!strcmp(extension.c_str(), ".bin"))
		return SaveToBin(file, mesh);
	else if (!strcmp(extension.c_str(), ".ply"))
		return SaveToPLY(file, mesh);
	return false;
}

bool MeshSaver::SaveToBin(const std::string& mesh_file, Mesh& mesh)
{
	return false;
}

bool MeshSaver::SaveToBin(const std::wstring& mesh_file, Mesh& mesh)
{
	//DWORD encryptionType = rand();
	//int version = 4;
	//
	//ALG_ID algorithm_id = CALG_AES_128;
	//std::string password = "F73D0B1F-789C-43AD-BDBD-232A497C4AC9";
	//LPCTSTR providerName = MS_ENH_RSA_AES_PROV;
	//DWORD providerType = PROV_RSA_AES;
	//HCRYPTPROV hcr_provider = NULL;
	//HCRYPTKEY key = NULL;
	//HCRYPTHASH hash = NULL;
	//
	//BOOL result = FALSE;
	//result = ::CryptAcquireContext(
	//	&hcr_provider,
	//	NULL,
	//	providerName,
	//	providerType,
	//	CRYPT_VERIFYCONTEXT);
	//if (!result)
	//	return false;
	//
	////create the password hash
	//result = ::CryptCreateHash(
	//	hcr_provider,
	//	CALG_MD5,
	//	0,
	//	0,
	//	&hash
	//);
	//if (!result)
	//	return false;
	//
	////hash the password
	//result = ::CryptHashData(
	//	hash,
	//	(BYTE*)password.c_str(),
	//	(DWORD)password.size(),
	//	0
	//);
	//if (!result)
	//	return false;
	//
	////create the encryption key
	//if (!::CryptDeriveKey(
	//	hcr_provider,
	//	algorithm_id,
	//	hash,
	//	0x00800000,
	//	&key))
	//	return false;
	//
	//size_t start_pos = sizeof(DWORD);
	//size_t data_size = 10;
	//if (start_pos >= data_size)
	//	return false;
	//
	//size_t sizeOfDataToEncrypt = data_size - start_pos;
	//
	//size_t encryptedDataSize = 0;
	//DWORD tmp_encrypted_data_size = (DWORD)sizeOfDataToEncrypt;
	//if (!::CryptEncrypt(
	//	key,
	//	NULL,
	//	TRUE,
	//	0,
	//	NULL,
	//	&tmp_encrypted_data_size,
	//	sizeOfDataToEncrypt
	//))
	//	return false;
	//
	//encryptedDataSize = (size_t)tmp_encrypted_data_size;
	//
	//size_t requiredDataSize = start_pos + encryptedDataSize;
	//
	//size_t buffer_size = sizeof(DWORD)
	//unsigned char* buffer = 0;
	//
	//DWORD dwDataSize = sizeOfDataToEncrypt;
	//DWORD buffer_size = (DWORD)(buffer_size - start_pos);
	//if (!::CryptEncrypt(
	//	key,
	//	NULL,
	//	TRUE,
	//	0,
	//	buffer,
	//	&dwDataSize,
	//	buffer_size
	//))
	//	return false;
	//

	std::fstream out;
	out.open(mesh_file.c_str(), std::ios::out | std::ios::binary);
	if (out.good())
	{
		unsigned first_magic_number = 1988226;
		char second_magic_string[7] = "Launca";

		out.write((const char*)&first_magic_number, sizeof(unsigned));
		out.write((const char*)second_magic_string, 7);

		out.write((const char*)&mesh.vertex_number, sizeof(unsigned));
		out.write((const char*)&mesh.triangle_number, sizeof(unsigned));
		out.write((const char*)&mesh.m_repair_vertex_append_index, sizeof(unsigned));
		out.write((const char*)&mesh.m_repair_triangle_append_index, sizeof(unsigned));
		out.write((const char*)mesh.vertex_position, 3 * sizeof(float) * mesh.vertex_number);
		out.write((const char*)mesh.vertex_normal, 3 * sizeof(float) * mesh.vertex_number);
		out.write((const char*)mesh.vertex_color, 3 * sizeof(unsigned char) * mesh.vertex_number);
		out.write((const char*)mesh.triangle_index, 3 * sizeof(unsigned) * mesh.triangle_number);
	}
	return true;
}

bool MeshSaver::SaveToSTL(const std::string& mesh_file, Mesh& mesh)
{
	std::fstream out;

	out.open(mesh_file.c_str(), std::ios::out | std::ios::binary);
	if (out.good())
	{
		return SaveToSTL(out, mesh);
	}

	return false;
}

bool MeshSaver::SaveToSTL(std::fstream& out, Mesh& mesh)
{
	char header[80];
	memset(header, 0, 80);
	out.write(header, 80);

	unsigned tri_size = mesh.triangle_number;
	out.write((const char*)&tri_size, sizeof(unsigned));

	for (unsigned i = 0; i < tri_size; ++i)
	{
		unsigned index1 = mesh.triangle_index[3 * i];
		unsigned index2 = mesh.triangle_index[3 * i + 1];
		unsigned index3 = mesh.triangle_index[3 * i + 2];

		float normal1[3];
		float normal2[3];
		float normal3[3];
		float vertex1[3];
		float vertex2[3];
		float vertex3[3];

		auto get = [&mesh](float* normal, float* vertex, unsigned index) {
			float* n = mesh.vertex_normal;
			float* v = mesh.vertex_position;
			memcpy(normal, n + 3 * index, 3 * sizeof(float));
			memcpy(vertex, v + 3 * index, 3 * sizeof(float));
		};

		get(normal1, vertex1, index1);
		get(normal2, vertex2, index2);
		get(normal3, vertex3, index3);

		float nf[3];
		nf[0] = (normal1[0] + normal2[0] + normal3[0]) / 3.0f;
		nf[1] = (normal1[1] + normal2[1] + normal3[1]) / 3.0f;
		nf[2] = (normal1[2] + normal2[2] + normal3[2]) / 3.0f;
		out.write((const char*)&nf[0], 3 * sizeof(float));

		out.write((const char*)&vertex1[0], 3 * sizeof(float));
		out.write((const char*)&vertex2[0], 3 * sizeof(float));
		out.write((const char*)&vertex3[0], 3 * sizeof(float));

		unsigned short attributes = 0;
		out.write((const char*)&attributes, sizeof(unsigned short));
	}

	return true;
}

bool MeshSaver::SaveToPLY(const std::string& mesh_file, Mesh& mesh)
{
	std::fstream exportFile(mesh_file, std::ios::out | std::ios::binary);

	if (!exportFile.is_open())
	{
		return false;
	}

	exportFile.clear();

	return SaveToPLY(exportFile, mesh);
}

bool MeshSaver::SaveToPLY(const std::wstring & mesh_file, Mesh & mesh)
{
	std::fstream exportFile(mesh_file, std::ios::out | std::ios::binary);

	if (!exportFile.is_open())
	{
		return false;
	}

	exportFile.clear();

	return SaveToPLY(exportFile, mesh);
}

bool MeshSaver::SaveToPLY(std::fstream & exportFile, Mesh & mesh)
{
	// Write Header
	exportFile << "ply\n";
	exportFile << "format	binary_little_endian	1.0\n";
	exportFile << "element	vertex	" << mesh.vertex_number << "\n";
	exportFile << "property	float32	x\n";
	exportFile << "property	float32	y\n";
	exportFile << "property	float32	z\n";

	if (mesh.vertex_color)
	{
		exportFile << "property uint8	red\n";
		exportFile << "property uint8	green\n";
		exportFile << "property uint8	blue\n";
	}

	if (mesh.vertex_normal)
	{
		exportFile << "property	int32	nx\n";
		exportFile << "property	int32	ny\n";
		exportFile << "property	int32	nz\n";
	}

	exportFile << "element	face	" << mesh.triangle_number << "\n";
	exportFile << "property   list   uint8   int32   vertex_indices \n";

	exportFile << "end_header\n";

	// Fill data
	for (size_t i = 0; i < mesh.vertex_number; i++)
	{
		auto vertexPositionData = mesh.vertex_position + 3 * i;
		exportFile.write(reinterpret_cast<char*>(vertexPositionData), 3 * sizeof(float));

		if (mesh.vertex_color)
		{
			auto vertexColorData = mesh.vertex_color + 3 * i;
			exportFile.write(reinterpret_cast<char*>(vertexColorData), 3 * sizeof(char));
		}

		if (mesh.vertex_normal)
		{
			auto vertexNormalData = mesh.vertex_normal + 3 * i;
			exportFile.write(reinterpret_cast<char*>(vertexNormalData), 3 * sizeof(float));
		}
	}

	unsigned char vertex_count = 3;
	for (size_t i = 0; i < mesh.triangle_number; i++)
	{
		unsigned* indices = mesh.triangle_index + 3 * i;
		exportFile.write(reinterpret_cast<char*> (&vertex_count), sizeof(unsigned char));
		exportFile.write(reinterpret_cast<char*>(indices), 3 * sizeof(unsigned));
	}

	exportFile.flush();
	exportFile.close();

	return true;
}

bool MeshSaver::SaveToSTL(const std::wstring& mesh_file, Mesh& mesh)
{
	std::fstream out;

	out.open(mesh_file.c_str(), std::ios::out | std::ios::binary);
	if (out.good())
	{
		return SaveToSTL(out, mesh);
	}

	return false;
}

}