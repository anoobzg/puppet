#pragma once
#include <vector>
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeodeCreator.h>

#include "cloud.h"
class Vertex
{
public:
	Vertex();
	~Vertex();

	void Update(float x, float y, float nx, float ny);
	void Clear();

	bool m_valid;
	float m_iso_value; //µÈÖµ
	float m_weight;
	unsigned m_id;

	float _x;
	float _y;
};

class Cell
{
public:
	Cell();
	~Cell();

	Vertex* lu;
	Vertex* ld;
	Vertex* ru;
	Vertex* rd;
};

class Base
{
public:
	Base();
	~Base();

	osg::Geode* GetGridGeode();
	osg::Geode* GetPanelGeode();
	osg::Geode* GetMCGeode();
	osg::Geode* GetPointsGeode();

	void AddPoint(const osg::Vec3f& p1, const osg::Vec3f& p2);
	void AddPoint(float x, float y, float nx, float ny);
	void Clear();
protected:
	Vertex* GetVertex(unsigned i, unsigned j);
	Cell* GetCell(unsigned i, unsigned j);
	Cell* GetCell(float x, float y);

	void SetupTopo();
	void SetupGLObject();
	void UpdatePanelColor(Vertex& vertex);
	void UpdateMC(unsigned i);
private:
	std::vector<Vertex> m_vertexes;
	std::vector<Cell> m_cells;

	osg::ref_ptr<osg::Vec3Array> m_coord_array;
	osg::ref_ptr<osg::Vec4Array> m_color_array;
	osg::ref_ptr<osg::Geode> m_grid_geode;
	osg::ref_ptr<osg::Geode> m_panel_geode;

	osg::ref_ptr<osg::Geode> m_mc_geode;
	osg::ref_ptr<Cloud> m_points_geode;
};