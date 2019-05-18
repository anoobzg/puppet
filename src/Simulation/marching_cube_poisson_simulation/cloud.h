#pragma once
#include <osg\Geode>

struct Point
{
	float x;
	float y;
	float nx;
	float ny;
};

class Cloud : public osg::Geode
{
public:
	Cloud();
	~Cloud();

	void AddPoint(float x, float y, float nx, float ny);
	void Clear();
	std::vector<Point>& GetPoints();
private:
	osg::ref_ptr<osg::Vec3Array> m_coord_array;
	osg::ref_ptr<osg::DrawArrays> m_draw_array;

	std::vector<Point> m_clouds;
};
