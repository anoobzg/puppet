#include "NormalCaculator.h"
#include "Mesh.h"

#include <math.h>
#include <memory>

namespace LauncaGeometry
{
void NormalCaculator::CaculateNormalFromTopo(Mesh& mesh)
{
	memset(mesh.vertex_normal, 0x00, 3 * sizeof(float) * mesh.vertex_number);
	for(unsigned i = 0; i < mesh.triangle_number; ++i)
	{
		unsigned* triangle = mesh.triangle_index + 3 * i;
		unsigned index1 = *triangle;
		unsigned index2 = *(triangle+1);
		unsigned index3 = *(triangle+2);

		float* vertex1 = mesh.vertex_position + 3 * index1;
		float* vertex2 = mesh.vertex_position + 3 * index2;
		float* vertex3 = mesh.vertex_position + 3 * index3;
	
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

		float l = sqrt(nf[0] * nf[0] + nf[1] * nf[1] + nf[2] * nf[2]);
		if(l > 0.0f)
		{
			nf[0] /= l;
			nf[1] /= l;
			nf[2] /= l;
		}

		auto add_normal = [&mesh](unsigned index, float x, float y, float z){
			float* normal = mesh.vertex_normal + 3 * index;
			*normal++ += x;
			*normal++ += y;
			*normal++ += z;
		};
		add_normal(index1, nf[0], nf[1], nf[2]);
		add_normal(index2, nf[0], nf[1], nf[2]);
		add_normal(index3, nf[0], nf[1], nf[2]);
	}
}

void NormalCaculator::CaculateFaceNormal(Mesh& mesh, unsigned index, float& nx, float& ny, float& nz)
{
	unsigned* triangle = mesh.triangle_index + 3 * index;
	unsigned index1 = *triangle;
	unsigned index2 = *(triangle + 1);
	unsigned index3 = *(triangle + 2);

	float* vertex1 = mesh.vertex_position + 3 * index1;
	float* vertex2 = mesh.vertex_position + 3 * index2;
	float* vertex3 = mesh.vertex_position + 3 * index3;

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

	float l = sqrt(nf[0] * nf[0] + nf[1] * nf[1] + nf[2] * nf[2]);
	if (l > 0.0f)
	{
		nf[0] /= l;
		nf[1] /= l;
		nf[2] /= l;
	}

	nx = nf[0];
	ny = nf[1];
	nz = nf[2];
}
}