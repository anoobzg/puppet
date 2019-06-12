#pragma once
#include <osg\Geometry>

class FeedGeometry : public osg::Geometry
{
public:
	FeedGeometry(osg::Vec3Array* coord_array, osg::FloatArray* flag_array);
	~FeedGeometry();

	void Exchange();
protected:
	osg::BoundingBox computeBoundingBox() const;
	void drawImplementation(osg::RenderInfo& renderInfo) const;
private:
	osg::ref_ptr<osg::Vec3Array> m_coord_array;
	osg::ref_ptr<osg::FloatArray> m_in_array;
};