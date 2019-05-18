#include "poisson.h"
#include <osgWrapper\StateDeclare.h>
#include <osgWrapper\GeodeCreator.h>

using namespace OSGWrapper;

float OcCell::min_cell_len = 4.0f;
OcCell::OcCell(float x, float y, float cell_len)
	:m_x(x), m_y(y), m_cell_len(cell_len)
	,lu(0), ld(0), ru(0), rd(0)
{

}

OcCell::~OcCell()
{
	if (lu) delete lu;
	if (ld) delete ld;
	if (ru) delete ru;
	if (rd) delete rd;
}

void OcCell::Expand(float x, float y)
{
	if (m_cell_len <= min_cell_len)
		return;

	float half_cell_len = m_cell_len / 2.0f;
	if (!lu) lu = new OcCell(m_x, m_y + half_cell_len, half_cell_len);
	if (!ld) ld = new OcCell(m_x, m_y, half_cell_len);
	if (!ru) ru = new OcCell(m_x + half_cell_len, m_y + half_cell_len, half_cell_len);
	if (!rd) rd = new OcCell(m_x + half_cell_len, m_y, half_cell_len);

	float x1 = m_x; float x2 = m_x + half_cell_len; float x3 = m_x + m_cell_len;
	float y1 = m_y; float y2 = m_y + half_cell_len; float y3 = m_y + m_cell_len;
	if (x >= x1 && x <= x2)
	{
		if (y >= y1 && y <= y2)
		{
			ld->Expand(x, y);
		}
		else if (y >= y2 && y <= y3)
		{
			lu->Expand(x, y);
		}
	}
	else if (x >= x2 && x <= x3)
	{
		if (y >= y1 && y <= y2)
		{
			rd->Expand(x, y);
		}
		else if (y >= y2 && y <= y3)
		{
			ru->Expand(x, y);
		}
	}
}

Poisson::Poisson()
	:m_root(0.0f, 0.0f, 1024.0f)
{
	m_points_geode = new Cloud();
	m_points_geode->getOrCreateStateSet()->addUniform(new osg::Uniform("color", osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)), state_on);
}

Poisson::~Poisson()
{

}

osg::Geode* Poisson::GetGridGeode()
{
	return m_grid_geode;
}

osg::Geode* Poisson::GetPanelGeode()
{
	return m_panel_geode;
}

osg::Geode* Poisson::GetPoissonGeode()
{
	return m_poisson_geode;
}

osg::Geode* Poisson::GetPointsGeode()
{
	return m_points_geode;
}

void Poisson::AddPoint(float x, float y, float nx, float ny)
{
	m_points_geode->AddPoint(x, y, nx, ny);

	m_root.Expand(x, y);
}

void Poisson::Generate(osg::Vec3Array* panel_coord, osg::Vec4Array* panel_color, osg::Vec3Array* grid_coord, OcCell& cell)
{
	float x1 = cell.m_x; float x2 = cell.m_x + cell.m_cell_len;
	float y1 = cell.m_y; float y2 = cell.m_y + cell.m_cell_len;
	osg::Vec3 p1(x1, y2, 0.0f); osg::Vec3 p2(x1, y1, 0.0f); osg::Vec3 p3(x2, y1, 0.0f); osg::Vec3 p4(x2, y2, 0.0f);
	osg::Vec4 color(1.0f, 1.0f, 1.0f, 1.0f);
	panel_coord->push_back(p1); panel_coord->push_back(p2); panel_coord->push_back(p3); panel_coord->push_back(p4);
	panel_color->push_back(color); panel_color->push_back(color); panel_color->push_back(color); panel_color->push_back(color);
	grid_coord->push_back(p1); grid_coord->push_back(p2);
	grid_coord->push_back(p2); grid_coord->push_back(p3);
	grid_coord->push_back(p3); grid_coord->push_back(p4);
	grid_coord->push_back(p4); grid_coord->push_back(p1);
	OcCell* lu = cell.lu;
	OcCell* ld = cell.ld;
	OcCell* ru = cell.ru;
	OcCell* rd = cell.rd;
	if (lu) Generate(panel_coord, panel_color, grid_coord, *lu);
	if (ld) Generate(panel_coord, panel_color, grid_coord, *ld);
	if (ru) Generate(panel_coord, panel_color, grid_coord, *ru);
	if (rd) Generate(panel_coord, panel_color, grid_coord, *rd);
}

void Poisson::Generate()
{
	m_coord_array = new osg::Vec3Array();
	m_color_array = new osg::Vec4Array();
	osg::Vec3Array* grid_coord_array = new osg::Vec3Array();
	Generate(m_coord_array, m_color_array, grid_coord_array, m_root);
	
	osg::DrawArrays* grid_primitive_set = new osg::DrawArrays(GL_LINES, 0, grid_coord_array->size());
	osg::DrawArrays* panel_primitive_set = new osg::DrawArrays(GL_QUADS, 0, m_color_array->size());

	m_grid_geode = GeodeCreator::CreateIndexAttributeGeode(grid_primitive_set, grid_coord_array);
	m_panel_geode = GeodeCreator::CreateIndexAttributeGeode(panel_primitive_set, m_coord_array, m_color_array);
}