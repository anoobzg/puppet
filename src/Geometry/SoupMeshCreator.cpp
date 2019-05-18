#include "SoupMeshCreator.h"
#include "Mesh.h"

namespace LauncaGeometry
{
	Mesh* SoupMeshCreator::Create(Mesh& mesh)
	{
		if(mesh.vertex_number == 0 || mesh.triangle_number == 0)
			return NULL;

		Mesh* result = new Mesh();
		result->AllocateTriangle(mesh.triangle_number);
		result->AllocateVertex(3 * mesh.triangle_number);

		for(unsigned i = 0; i < 3 * result->triangle_number; ++i)
			*(result->triangle_index+i) = i;

		float* tposition = result->vertex_position;
		float* tnormal = result->vertex_normal;
		unsigned char* tcolor = result->vertex_color;
		for(unsigned i = 0; i < result->triangle_number; ++i)
		{
			unsigned* triangle = mesh.triangle_index + 3 * i;
			unsigned index1 = *triangle;
			unsigned index2 = *(triangle+1);
			unsigned index3 = *(triangle+2);

			float* vertex1 = mesh.vertex_position + 3 * index1;
			float* vertex2 = mesh.vertex_position + 3 * index2;
			float* vertex3 = mesh.vertex_position + 3 * index3;
			unsigned char* color1 = mesh.vertex_color + 3 * index1;
			unsigned char* color2 = mesh.vertex_color + 3 * index2;
			unsigned char* color3 = mesh.vertex_color + 3 * index3;

			float x12 = vertex2[0] - vertex1[0];
			float y12 = vertex2[1] - vertex1[1];
			float z12 = vertex2[2] - vertex1[2];
			float x13 = vertex3[0] - vertex1[0];
			float y13 = vertex3[1] - vertex1[1];
			float z13 = vertex3[2] - vertex1[2];

			float nf[3];
			nf[0] = y12 * z13 - y13 * z12;
			nf[1] = z12 * x13 - x12 * z13;
			nf[2] = x12 * y13 - y12 * x13;

			*tnormal++ = nf[0]; *tnormal++ = nf[1]; *tnormal++ = nf[2];
			*tnormal++ = nf[0]; *tnormal++ = nf[1]; *tnormal++ = nf[2];
			*tnormal++ = nf[0]; *tnormal++ = nf[1]; *tnormal++ = nf[2];

			*tposition++ = *vertex1++; *tposition++ = *vertex1++; *tposition++ = *vertex1++;
			*tposition++ = *vertex2++; *tposition++ = *vertex2++; *tposition++ = *vertex2++;
			*tposition++ = *vertex3++; *tposition++ = *vertex3++; *tposition++ = *vertex3++;
			*tcolor++ = *color1++; *tcolor++ = *color1++; *tcolor++ = *color1++;
			*tcolor++ = *color2++; *tcolor++ = *color2++; *tcolor++ = *color2++;
			*tcolor++ = *color3++; *tcolor++ = *color3++; *tcolor++ = *color3++;
		}

		return result;
	}
}