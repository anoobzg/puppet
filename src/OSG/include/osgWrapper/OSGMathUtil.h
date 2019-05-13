#ifndef OSG_MATH_UTIL
#define OSG_MATH_UTIL
#include <osg\Vec3>
#include <osg\Quat>

#define _USE_MATH_DEFINES
#include <math.h>

namespace OSGWrapper
{

class OSG_EXPORT OSGMathUtil
{
public:
	static float Angle(const osg::Vec3f& v1, const osg::Vec3f& v2);
	static osg::Vec3f Get3DPoint(const osg::Vec2f& point, const osg::Vec2f& center, float width, float height, bool skipz);

	static osg::Vec3d Lerp(osg::Vec3d& p1, osg::Vec3d& p2, double lambda);
	static osg::Vec3f Lerp(osg::Vec3f& p1, osg::Vec3f& p2, double lambda);
	static osg::Quat Lerp(osg::Quat& q1, osg::Quat& q2, double lambda);
};

}
#endif // OSG_MATH_UTIL