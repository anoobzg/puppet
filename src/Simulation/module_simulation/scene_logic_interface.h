#pragma once
#include <vector>

struct XYZ
{
	float x;
	float y;
	float z;
};

class ControlPoint
{
	friend class Surface;
public:
	ControlPoint();
	~ControlPoint();

public:
	unsigned m_handle;
	float m_radius;

	XYZ m_xyz;

	unsigned m_vertex_index;
};

class Path
{
	friend class Surface;
public:
	Path();
	~Path();

public:
	unsigned m_handle;

	std::vector<XYZ> m_path;
};

#include "Mesh.h"
using namespace LauncaGeometry;

struct SurfaceTopoCallback
{
	virtual void ShowSurface(Mesh& mesh) = 0;
	virtual void SurfaceCurvatureChanged(unsigned verrtex_number, float* curvature) = 0;
	virtual void ControlPointAdded(ControlPoint& control_point) = 0;
	virtual void ControlPointDeleted(ControlPoint& control_point) = 0;
	virtual void ControlPointModified(ControlPoint& control_point) = 0;
	virtual void PathAdded(Path& path) = 0;
	virtual void PathRemoved(Path& path) = 0;
};