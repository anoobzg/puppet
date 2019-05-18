#pragma once
#include "Mesh.h"
#include <memory>

#include <osg\Geode>

#include "ShortestPath.h"

#include <list>
using namespace LauncaGeometry;
class Surface
{
public:
	Surface(const char* file);
	~Surface();

	Mesh* GetMesh();
	osg::Geode* GetGeode();
	osg::Geode* GetPath();
	void GetCurvatureData(char* pCurvatureData);

	void AddControlPoint(unsigned triangle_index, bool bCorrect);
	bool _doPathFinding(int nStartVertexIndex, int nEndVertexIndex, std::vector<unsigned>& result);
	unsigned GetCorrectEndIndex();
	unsigned GetCorrectStartIndex();
protected:
	void CreateOnePath(std::vector<unsigned>& path);
private:
	std::auto_ptr<Mesh> m_mesh;

	osg::ref_ptr<osg::Geode> m_geode;
	osg::ref_ptr<osg::Geode> m_path;

	ShortestPath m_algrithm;

	std::list<unsigned> m_control_points;
	float* pCurvatureData;
};