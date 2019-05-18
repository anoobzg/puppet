#include "Mesh.h"

namespace LauncaGeometry
{
	Mesh::Mesh()
		:vertex_number(0), triangle_number(0)
		,vertex_position(0), vertex_normal(0), vertex_color(0), triangle_index(0), triangle_char(0)
	{
		m_repair_vertex_append_index = 0; m_repair_triangle_append_index = 0;
	}

	Mesh::~Mesh()
	{
		ReleaseVertex();
		ReleaseTriangles();
	}

	void Mesh::AllocateVertex(unsigned number)
	{
		ReleaseVertex();

		vertex_number = number;
		m_repair_vertex_append_index = vertex_number;
		vertex_position = new float[3 * vertex_number];
		vertex_normal = new float[3 * vertex_number];
		vertex_color = new unsigned char[3 * vertex_number];
	}

	void Mesh::AllocateTriangle(unsigned number)
	{
		ReleaseTriangles();

		triangle_number = number;
		m_repair_triangle_append_index = triangle_number;
		triangle_index = new unsigned[3 * triangle_number];
		triangle_char = new char[2 * triangle_number];
	}

	void Mesh::ReleaseVertex()
	{
		vertex_number = 0;
		if(vertex_color) { delete [] vertex_color; vertex_color = 0; }
		if(vertex_position) { delete [] vertex_position; vertex_position = 0; }
		if(vertex_normal) { delete [] vertex_normal; vertex_normal = 0; }
		m_repair_vertex_append_index = vertex_number;
	}

	void Mesh::ReleaseTriangles()
	{
		triangle_number = 0;
		if(triangle_index) { delete [] triangle_index; triangle_index = 0; }
		if (triangle_char) { delete[] triangle_char; triangle_char = 0; }
		m_repair_triangle_append_index = triangle_number;
	}
}