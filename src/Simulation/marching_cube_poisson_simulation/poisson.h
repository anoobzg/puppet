#pragma once
#include <vector>
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeodeCreator.h>
#include "cloud.h"

class OcCell
{
public:
	OcCell(float x, float y, float cell_len);
	~OcCell();

	void Expand(float x, float y);

	OcCell* lu;
	OcCell* ld;
	OcCell* ru;
	OcCell* rd;
	
	const float m_x;
	const float m_y;
	const float m_cell_len;
	static float min_cell_len;
};

class Poisson
{
public:
	Poisson();
	~Poisson();

	osg::Geode* GetGridGeode();
	osg::Geode* GetPanelGeode();
	osg::Geode* GetPoissonGeode();
	osg::Geode* GetPointsGeode();

	void AddPoint(float x, float y, float nx, float ny);
	void Generate();
protected:
	void Generate(osg::Vec3Array* panel_coord, osg::Vec4Array* panel_color, osg::Vec3Array* grid_coord, OcCell& cell);
private:
	osg::ref_ptr<osg::Vec3Array> m_coord_array;
	osg::ref_ptr<osg::Vec4Array> m_color_array;
	osg::ref_ptr<osg::Geode> m_grid_geode;
	osg::ref_ptr<osg::Geode> m_panel_geode;

	osg::ref_ptr<osg::Geode> m_poisson_geode;
	osg::ref_ptr<Cloud> m_points_geode;

	OcCell m_root;
};