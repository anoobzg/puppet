#pragma once

class ControlPoint
{
public:
	ControlPoint();
	~ControlPoint();

public:
	unsigned m_handle;
	float m_radius;

	unsigned m_vertex_index;
};

#include "Mesh.h"
using namespace LauncaGeometry;

struct MeshSegmentorCallback
{
	virtual void ShowMesh(Mesh& mesh) = 0;
	virtual void UpdateHarmonic(unsigned vertex_number, float* harmonic) = 0;
};