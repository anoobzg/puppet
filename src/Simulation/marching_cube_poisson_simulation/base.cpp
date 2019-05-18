#include "base.h"
#include <osgWrapper\StateDeclare.h>
#include "config.h"

using namespace OSGWrapper;
Vertex::Vertex()
	:m_valid(false),m_iso_value(0.0f),m_id(0),_x(0.0f), _y(0.0f), m_weight(0.0f)
{

}

Vertex::~Vertex()
{

}

void Vertex::Update(float x, float y, float nx, float ny)
{
	float dx = _x - x;
	float dy = _y - y;
	float d = sqrt(dx * dx + dy * dy) / base_grid_size;

	float value = dx * nx + dy * ny;
	value /= d;

	float w1 = m_weight;
	float w2 = 1.0f;
	m_weight += 1.0f;
	float iweight = 1.0f / m_weight;
	w1 *= iweight;
	w2 *= iweight;

	m_iso_value = w1 * m_iso_value + w2 * value;

	m_valid = true;
}

void Vertex::Clear()
{
	m_valid = false;
	m_weight = 0.0f;
	m_iso_value = 0.0f;
}

Cell::Cell()
	:lu(0), ld(0), ru(0), rd(0)
{

}

Cell::~Cell()
{

}

Base::Base()
{
	m_vertexes.resize(base_grid_vertex_number * base_grid_vertex_number);
	m_cells.resize(base_cell_number * base_cell_number);

	SetupTopo();
	SetupGLObject();
}

Base::~Base()
{

}

Vertex* Base::GetVertex(unsigned i, unsigned j)
{
	if (i >= base_grid_vertex_number || j >= base_grid_vertex_number)
		return 0;
	return &m_vertexes[i * base_grid_vertex_number + j];
}

Cell* Base::GetCell(unsigned i, unsigned j)
{
	if (i >= base_cell_number || j >= base_cell_number)
		return 0;
	return &m_cells[i * base_cell_number + j];
}

Cell* Base::GetCell(float x, float y)
{
	unsigned j = (unsigned)((x - base_x_offset)/base_grid_size);
	unsigned i = (unsigned)((y - base_y_offset) / base_grid_size);
	return GetCell(i, j);
}

void Base::SetupTopo()
{
	for (unsigned i = 0; i < base_grid_vertex_number; ++i)
	{
		for (unsigned j = 0; j < base_grid_vertex_number; ++j)
		{
			unsigned id = i * base_grid_vertex_number + j;
			m_vertexes[id].m_id = id;
			m_vertexes[id]._x = (float)j * base_grid_size + base_x_offset;
			m_vertexes[id]._y = (float)i * base_grid_size + base_y_offset;
		}
	}

	for (unsigned i = 0; i < base_cell_number; ++i)
	{
		for (unsigned j = 0; j < base_cell_number; ++j)
		{
			unsigned index = i * base_cell_number + j;
			Cell& cell = m_cells[index];
			cell.lu = GetVertex(i + 1, j);
			cell.ld = GetVertex(i, j);
			cell.ru = GetVertex(i + 1, j + 1);
			cell.rd = GetVertex(i, j + 1);
		}
	}
}

void Base::SetupGLObject()
{
	unsigned vertex_number = base_grid_vertex_number * base_grid_vertex_number;
	m_coord_array = new osg::Vec3Array();
	m_color_array = new osg::Vec4Array();
	m_coord_array->reserve(vertex_number);
	m_color_array->reserve(vertex_number);

	for (unsigned i = 0; i < base_grid_vertex_number; ++i)
	{
		float y = (float)i * base_grid_size + base_y_offset;
		for (unsigned j = 0; j < base_grid_vertex_number; ++j)
		{
			float x = (float)j * base_grid_size + base_x_offset;
			m_coord_array->push_back(osg::Vec3(x, y, 0.0f));
			m_color_array->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
		}
	}

	unsigned cell_number = base_cell_number * base_cell_number;
	osg::DrawElementsUInt* grid_primitive_set = new osg::DrawElementsUInt(GL_LINES);
	osg::DrawElementsUInt* panel_primitive_set = new osg::DrawElementsUInt(GL_QUADS);
	grid_primitive_set->reserve(8 * cell_number);
	panel_primitive_set->reserve(4 * cell_number);
	for (unsigned i = 0; i < base_cell_number; ++i)
	{
		for (unsigned j = 0; j < base_cell_number; ++j)
		{
			unsigned ld = i * base_grid_vertex_number + j;
			unsigned lu = ld + base_grid_vertex_number;
			unsigned rd = ld + 1;
			unsigned ru = lu + 1;
			grid_primitive_set->push_back(lu); grid_primitive_set->push_back(ld);
			grid_primitive_set->push_back(ld); grid_primitive_set->push_back(rd);
			grid_primitive_set->push_back(ru); grid_primitive_set->push_back(lu);
			grid_primitive_set->push_back(rd); grid_primitive_set->push_back(ru);
			panel_primitive_set->push_back(lu);
			panel_primitive_set->push_back(ld);
			panel_primitive_set->push_back(rd);
			panel_primitive_set->push_back(ru);
		}
	}

	m_grid_geode = GeodeCreator::CreateIndexAttributeGeode(grid_primitive_set, m_coord_array);
	m_panel_geode = GeodeCreator::CreateIndexAttributeGeode(panel_primitive_set, m_coord_array, m_color_array);

	m_mc_geode = GeodeCreator::CreateGeode();
	m_mc_geode->getOrCreateStateSet()->addUniform(new osg::Uniform("color", osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f)), state_on);
	for (unsigned i = 0; i < base_cell_number; ++i)
	{
		osg::Geometry* geom = new osg::Geometry();
		geom->setUseVertexBufferObjects(true);
		geom->setUseDisplayList(false);
		geom->setCullingActive(false);
		m_mc_geode->addDrawable(geom);
	}

	m_points_geode = new Cloud();
	m_points_geode->getOrCreateStateSet()->addUniform(new osg::Uniform("color", osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)), state_on);
}

osg::Geode* Base::GetGridGeode()
{
	return m_grid_geode;
}

osg::Geode* Base::GetPanelGeode()
{
	return m_panel_geode;
}

osg::Geode* Base::GetMCGeode()
{
	return m_mc_geode;
}

osg::Geode* Base::GetPointsGeode()
{
	return m_points_geode;
}

void Base::AddPoint(float x, float y, float nx, float ny)
{
	float d = std::sqrtf(nx * nx + ny * ny);
	if (d == 0.0f) return;

	nx /= d; ny /= d;

	Cell* cell = GetCell(x, y);
	if (!cell) return;

	//update vertex
	if (cell->ld)
	{
		cell->ld->Update(x, y, nx, ny);
		UpdatePanelColor(*cell->ld);
	}
	if (cell->lu)
	{
		cell->lu->Update(x, y, nx, ny);
		UpdatePanelColor(*cell->lu);
	}
	if (cell->rd)
	{
		cell->rd->Update(x, y, nx, ny);
		UpdatePanelColor(*cell->rd);
	}
	if (cell->ru)
	{
		cell->ru->Update(x, y, nx, ny);
		UpdatePanelColor(*cell->ru);
	}
	//update cloud
	m_points_geode->AddPoint(x, y, nx, ny);
	//update cell

	unsigned i = (unsigned)((y - base_y_offset) / base_grid_size);
	UpdateMC(i);
	UpdateMC(i + 1);
	UpdateMC(i - 1);
}

void Base::AddPoint(const osg::Vec3f& p1, const osg::Vec3f& p2)
{
	float x = p1.x(); float y = p1.y();
	float nx = p2.x() - p1.x(); float ny = p2.y() - p1.y();

	AddPoint(x, y, nx, ny);
}

void Base::UpdatePanelColor(Vertex& vertex)
{
	auto f = [](float iso)->osg::Vec4f {
		static osg::Vec4 blue = osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f);
		static osg::Vec4 green = osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
		static osg::Vec4 red = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
		static osg::Vec4 n = osg::Vec4(0.576f, 1.0f, 0.788f, 1.0f);
		static osg::Vec4 p = osg::Vec4(1.0f, 0.73f, 0.73f, 1.0f);

		static float clip = 10.0f;
		float lambda = abs(iso) / clip;
		if (lambda >= 1.0f) lambda = 1.0f;

		//if (abs(iso) <= 1.0f)
		//	return blue;
		//return osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
		if (iso >= 0.0f)
			return p;
		else if (iso <= -0.0f)
			return n;
		return osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
		//if (iso >= 0.0f)
		//{
		//	return red * lambda + green * (1 - lambda);
		//}
		//else
		//{
		//	return blue * lambda + green * (1 - lambda);
		//}
	};

	unsigned id = vertex.m_id;
	float iso = vertex.m_iso_value;

	if (vertex.m_valid)
	{
		m_color_array->at(id) = f(iso);
		m_color_array->dirty();
	}
}

void Base::Clear()
{
	unsigned vertex_number = base_grid_vertex_number * base_grid_vertex_number;
	for (unsigned i = 0; i < vertex_number; ++i)
	{
		Vertex& v = m_vertexes[i];
		v.Clear();
		m_color_array->at(i) = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
	}
	m_color_array->dirty();

	m_points_geode->Clear();

	for (unsigned i = 0; i < base_cell_number; ++i)
	{
		osg::Geometry* geom = dynamic_cast<osg::Geometry*>(m_mc_geode->getDrawable(i));
		if (geom)
		{
			geom->removePrimitiveSet(0, geom->getPrimitiveSetList().size());
			geom->setVertexAttribArray(0, 0);
		}
	}
}

void Base::UpdateMC(unsigned i)
{
	static unsigned mc_count[16] = {
		0, 1, 1, 1,
		1, 2, 1, 1,
		1, 1, 2, 1,
		1, 1, 1, 0
	};

	static unsigned v_map[16][8] = 
	{
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 3, 0, 0, 0, 0,
		0, 1, 1, 2, 0, 0, 0, 0,
		0, 3, 1, 2, 0, 0, 0, 0,
		1, 2, 2, 3, 0, 0, 0, 0,
		0, 1, 1, 2, 2, 3, 0, 3,
		0, 1, 2, 3, 0, 0, 0, 0,
		2, 3, 3, 0, 0, 0, 0, 0,
		0, 3, 2, 3, 0, 0, 0, 0,
		0, 1, 2, 3, 0, 0, 0, 0,
		0, 3, 0, 1, 2, 1, 2, 3,
		1, 2, 2, 3, 0, 0, 0, 0,
		1, 2, 0, 3, 0, 0, 0, 0,
		0, 1, 1, 2, 0, 0, 0, 0,
		0, 1, 0, 3, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0
	};

	if (i >= base_cell_number)
		return;

	osg::Geometry* geom = dynamic_cast<osg::Geometry*>(m_mc_geode->getDrawable(i));
	if (geom)
	{
		osg::Vec3Array* coord_array = new osg::Vec3Array();
		for (unsigned j = 0; j < base_cell_number; ++j)
		{
			Cell* cell = GetCell(i, j);
			Vertex* v[4];
			v[0] = cell->lu;
			v[1] = cell->ld;
			v[2] = cell->rd;
			v[3] = cell->ru;

			if (!v[0]->m_valid || !v[1]->m_valid || !v[2]->m_valid || !v[3]->m_valid)
				continue;
			
			unsigned index = 0;
			if (v[0]->m_iso_value >= 0.0f) index |= 1;
			if (v[1]->m_iso_value >= 0.0f) index |= 2;
			if (v[2]->m_iso_value >= 0.0f) index |= 4;
			if (v[3]->m_iso_value >= 0.0f) index |= 8;

			unsigned count = mc_count[index];
			for (unsigned i = 0; i < count; ++i)
			{
				unsigned idx1 = v_map[index][4 * i];
				unsigned idx2 = v_map[index][4 * i + 1];
				unsigned idx3 = v_map[index][4 * i + 2];
				unsigned idx4 = v_map[index][4 * i + 3];

				float x1 = (v[idx1]->_x + v[idx2]->_x) / 2.0f;
				float y1 = (v[idx1]->_y + v[idx2]->_y) / 2.0f;
				float x2 = (v[idx3]->_x + v[idx4]->_x) / 2.0f;
				float y2 = (v[idx3]->_y + v[idx4]->_y) / 2.0f;
				coord_array->push_back(osg::Vec3(x1, y1, 0.0f));
				coord_array->push_back(osg::Vec3(x2, y2, 0.0f));
			}
		}

		osg::DrawArrays* primitive_set = new osg::DrawArrays(GL_LINES, 0, coord_array->size());
		geom->setVertexAttribArray(0, coord_array, osg::Array::BIND_PER_VERTEX);

		if (geom->getPrimitiveSetList().size() == 0)
			geom->addPrimitiveSet(primitive_set);
		else
			geom->setPrimitiveSet(0, primitive_set);
	}
}