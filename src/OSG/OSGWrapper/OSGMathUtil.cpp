#include <osgWrapper\OSGMathUtil.h>

#include <math.h>
#include <stdlib.h>

namespace OSGWrapper
{

float OSGMathUtil::Angle(const osg::Vec3f& v1, const osg::Vec3f& v2)
{
	double denominator = sqrt((double)v1.length2() * (double)v2.length2());
	double cosinus = 0.0;

	if (denominator < DBL_MIN)
		cosinus = 1.0; // cos (1)  = 0 degrees
	
	cosinus = (double)(v1 * v2) / denominator;
	cosinus = cosinus > 1.0 ? 1.0 : ( cosinus < -1.0 ? -1.0 : cosinus);
	double angle = acos(cosinus);

	if (!_finite(angle) || angle > M_PI)
		angle = 0.0;

	return (float)angle;
}

osg::Vec3f OSGMathUtil::Get3DPoint(const osg::Vec2f& point, const osg::Vec2f& center, float width, float height, bool skipz)
{
	osg::Vec3f pt3D;

	float dx = point.x() - center.x();
	float dy = point.y() - center.y();

	float width2 = __max(fabs(dx), __min(fabs(width - center.x()), fabs(center.x())));
	float height2 = __max(fabs(dy), __min(fabs(height - center.y()), fabs(center.y())));

	float maxDim = __max(width2, height2);

	pt3D.x() = dx / maxDim;
	pt3D.y() = dy / maxDim;

	if (skipz)
	{
		pt3D.z() = 0;
		pt3D.normalize();
	}
	else
	{
		/// Sphere equation : (x-x_o)^2 + (y-y_o)^2 + (z-z_o)^2 = R^2
		/// we take Center O(x_o=0, y_o=0, z_o=0) and R = 1

		float x2_plus_y2 = (pt3D.x() * pt3D.x() + pt3D.y() * pt3D.y());
		if (x2_plus_y2  > 1.0f)
		{
			pt3D.z() = 0.0f;

			// fast normalization
			float length = sqrt(x2_plus_y2);
			pt3D.x() /= length;
			pt3D.y() /= length;
		}
		else
		{
			pt3D.z() = sqrt(1.0f - x2_plus_y2);
		}
	}

	return pt3D;
}

osg::Vec3d OSGMathUtil::Lerp(osg::Vec3d& p1, osg::Vec3d& p2, double lambda)
{
	return p1*(1.0 - lambda) + p2*lambda;
}

osg::Vec3f OSGMathUtil::Lerp(osg::Vec3f& p1, osg::Vec3f& p2, double lambda)
{
	return p1*(1.0 - lambda) + p2*lambda;
}

void lerp(float q1x, float q1y, float q1z, float q1w, float q2x, float q2y, float q2z, float q2w, float t, float* dstx, float* dsty, float* dstz, float* dstw)
{
	// Fast slerp implementation by kwhatmough:  
	// It contains no division operations, no trig, no inverse trig  
	// and no sqrt. Not only does this code tolerate small constraint  
	// errors in the input quaternions, it actually corrects for them.    
	if (t == 0.0f)
	{
		*dstx = q1x;
		*dsty = q1y;
		*dstz = q1z;
		*dstw = q1w;
		return;
	}
	else if(t == 1.0f)
	{
		*dstx = q2x;
		*dsty = q2y;
		*dstz = q2z;
		*dstw = q2w;
		return;
	}

	if (q1x == q2x && q1y == q2y && q1z == q2z && q1w == q2w)
	{
		*dstx = q1x; 
		*dsty = q1y; 
		*dstz = q1z; 
		*dstw = q1w; 
		return;
	}

	float halfY, alpha, beta;
	float u, f1, f2a, f2b;
	float ratio1, ratio2;
	float halfSecHalfTheta, versHalfTheta;
	float sqNotU, sqU;
		  
	float cosTheta = q1w * q2w + q1x * q2x + q1y * q2y + q1z * q2z;
	
	// As usual in all slerp implementations, we fold theta.  
	alpha = cosTheta >= 0 ? 1.0f : -1.0f;
	halfY = 1.0f + alpha * cosTheta;

	// Here we bisect the interval, so we need to fold t as well.  
	f2b = t - 0.5f;
	u = f2b >= 0 ? f2b : -f2b;
	f2a = u - f2b;
	f2b += u;
	u += u;
	f1 = 1.0f - u;
	
	// One iteration of Newton to get 1-cos(theta / 2) to good accuracy.  
	halfSecHalfTheta = 1.09f - (0.476537f - 0.0903321f * halfY) * halfY;
	halfSecHalfTheta *= 1.5f - halfY * halfSecHalfTheta * halfSecHalfTheta;
	versHalfTheta = 1.0f - halfY * halfSecHalfTheta;

	// Evaluate series expansions of the coefficients.  
	sqNotU = f1 * f1;
	ratio2 = 0.0000440917108f * versHalfTheta;
	ratio1 = -0.00158730159f + (sqNotU - 16.0f) * ratio2;
	ratio1 = 0.0333333333f + ratio1 * (sqNotU - 9.0f) * versHalfTheta;
	ratio1 = -0.333333333f + ratio1 * (sqNotU - 4.0f) * versHalfTheta;
	ratio1 = 1.0f + ratio1 * (sqNotU - 1.0f) * versHalfTheta;
	
	sqU = u * u;
	ratio2 = -0.00158730159f + (sqU - 16.0f) * ratio2;
	ratio2 = 0.0333333333f + ratio2 * (sqU - 9.0f) * versHalfTheta;
	ratio2 = -0.333333333f + ratio2 * (sqU - 4.0f) * versHalfTheta;
	ratio2 = 1.0f + ratio2 * (sqU - 1.0f) * versHalfTheta;
	
	// Perform the bisection and resolve the folding done earlier.  
	f1 *= ratio1 * halfSecHalfTheta;
	f2a *= ratio2;
	f2b *= ratio2;
	alpha *= f1 + f2a;
	beta = f1 + f2b;
	
	// Apply final coefficients to a and b as usual.  
	float w = alpha * q1w + beta * q2w;
	float x = alpha * q1x + beta * q2x;
	float y = alpha * q1y + beta * q2y;
	float z = alpha * q1z + beta * q2z;
	
	// This final adjustment to the quaternion's length corrects for  
	// any small constraint error in the inputs q1 and q2 But as you  
	// can see, it comes at the cost of 9 additional multiplication  
	// operations. If this error-correcting feature is not required,  
	// the following code may be removed.  
	f1 = 1.5f - 0.5f * (w * w + x * x + y * y + z * z);
	*dstw = w * f1;
	*dstx = x * f1;
	*dsty = y * f1;
	*dstz = z * f1;
}

osg::Quat OSGMathUtil::Lerp(osg::Quat& q1, osg::Quat& q2, double lambda)
{
	//return q1*(1.0 - lambda) + q2*lambda;
	float x, y, z, w;
	lerp(q1.x(), q1.y(), q1.z(), q1.w(), q2.x(), q2.y(), q2.z(), q2.w(), lambda, &x, &y, &z, &w);
	return osg::Quat(x, y, z, w);
}

}