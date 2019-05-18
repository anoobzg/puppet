#pragma once
#include "Mesh.h"
#include "point_group.h"
#include "ShortestPath.h"

#include <map>
using namespace LauncaGeometry;
class Surface
{
	friend class Collider;
public:
	Surface(Mesh& mesh);
	~Surface();

	void SetSurfaceTopoCallback(SurfaceTopoCallback* callback);
	void ShowGuassCurvature();
	void ShowMeanCurvature();

	bool TryAddControlPoint(unsigned vertex_handle, float* position);
	void ModifyControlPoint(unsigned control_point_handle, unsigned vertex_handle, float* position);
	void DeleteControlPoint(unsigned control_point_handle);

	float GetProperRadius();
protected:
	void CalculateCurvature();
	ControlPoint* CreateControlPoint(unsigned vertex_handle, float* position);
private:
	Mesh& m_mesh;

	SurfaceTopoCallback* m_callback;
	float* m_curvature_buffer;

	ShortestPath m_path_algrithm;

	PointGroup m_points;
};