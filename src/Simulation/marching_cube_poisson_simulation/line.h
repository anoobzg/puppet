#pragma once
#include <osg\Geode>

class Line : public osg::Geode
{
public:
	Line();
	~Line();

	void SetP1(const osg::Vec3f& p1);
	void SetP2(const osg::Vec3f& p2);
	void Set(const osg::Vec3f& p1, const osg::Vec3f& p2);
	void Get(osg::Vec3f& p1, osg::Vec3f& p2);
private:
	osg::ref_ptr<osg::Vec3Array> m_coord_array;
};