#pragma once
#include "Mesh.h"
#include <map>
#include <list>
#include <osg\Matrixf>

#include "ShortestPath.h"

using namespace LauncaGeometry;
class ControlPoint
{
public:
	ControlPoint();
	~ControlPoint();

	unsigned GetHandle();

	float x;
	float y;
	float z;

	unsigned m_vertex_index;
private:
	unsigned m_handle;
};

class Path
{
public:
	Path(unsigned vertex_index[2]);
	~Path();

	unsigned GetHandle();

	struct Point3D
	{
		unsigned vertex_handle;

		float x;
		float y;
		float z;
	};

	Path* m_pPrevPath;
	Path* m_pNextPath;

	std::list<Point3D> vertexPathList;
	unsigned m_control_point_handle[2];
private:
	unsigned m_handle;
};

struct SurfaceTopoCallback
{
	virtual void ControlPointAdded(ControlPoint& control_point) = 0;
	virtual void ControlPointDeleted(ControlPoint& control_point) = 0;
	virtual void ControlPointModified(ControlPoint& control_point) = 0;
	virtual void PathAdded(Path& path) = 0;
	virtual void PathRemoved(Path& path) = 0;
};

class Surface
{
public:
	bool m_bAutoFinishCircle;
	Mesh& m_mesh;

	Surface(Mesh& mesh);
	~Surface();

	void SetSurfaceTopoCallback(SurfaceTopoCallback* callback);
	bool AddControlPoint(unsigned vertex_handle, float clickCoord[3]);
	bool AddControlPointPath();
	void DeleteControlPoint(unsigned control_point_handle);
	void DeleteControlPointPath(unsigned control_point_path_handle);
	void UpdateControlPointPath(unsigned control_point_handle, unsigned vertex_handle);
	void InsertControlPoint(unsigned vertex_handle, float clickCoord[3]);

	bool CheckCollideControlPoint(const osg::Matrixf& matrix, const osg::Vec3f& eye, const osg::Vec3f& center, unsigned& control_point_handle);
	bool ModifyControlPoint(unsigned control_point_handle, unsigned vertex_handle, float clickCoord[3]);
	void SetWeightFactor(float m, float n);
	void SetCurvatureType(int nUseGuassCurvature);
	void SetupCurvatureArray(float* curve);
	bool ReCreatePath();
	bool IsCirclePath();

	unsigned GetProperStartIndex(unsigned index);
	std::list<unsigned> GetCirclePath();
	bool ClipMesh(unsigned clipCircleVertexSize, unsigned* clipCircleVertexData, unsigned userClickTriangleVertexData[3]);
	unsigned getClipMeshPrimitiveSize();
	void getClipMeshPrimitiveData(unsigned* pPrimitiveData);
	void SetCurvatureData(float* curvatureData);

private:
	ControlPoint* _createControlPoint(unsigned vertex_handle, float clickCoord[3]);
	bool _createControlPath(unsigned start_vertex_handle, unsigned end_vertex_handle);
	bool _doPathFinding(int nStartVertexIndex, int nEndVertexIndex, std::vector<unsigned>& result);
	

	SurfaceTopoCallback* m_topo_callback;

	typedef std::map<unsigned, ControlPoint*> ControlPoints;
	typedef std::map<unsigned, Path*> ControlPointPaths;
	typedef ControlPoints::iterator ControlPointsIter;

	std::list<ControlPoint*> m_point_order_list;
	ControlPoints m_control_points;
	ControlPointPaths m_control_point_paths;

	ShortestPath m_alg;
};