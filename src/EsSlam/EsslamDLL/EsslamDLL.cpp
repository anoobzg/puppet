#include "EsslamDLL.h"
#include "Esslam.h"
#include "TEsslam.h"

esslam::IESSlam* CreateSlam(const char* name)
{
	if (name && !strcmp(name, "T"))
		return new esslam::TEsslam();
	return new esslam::Esslam();
}

void DestroySlam(esslam::IESSlam* slam)
{
	if (slam) delete slam;
}