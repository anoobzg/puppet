#include "EsslamDLL.h"
#include "Esslam.h"

esslam::IESSlam* CreateSlam()
{
	return new esslam::Esslam();
}

void DestroySlam(esslam::IESSlam* slam)
{
	if (slam) delete slam;
}