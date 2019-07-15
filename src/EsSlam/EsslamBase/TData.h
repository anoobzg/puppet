#pragma once
#include <memory>
#include "Xform.h"

namespace esslam
{
	class DFrame;
	typedef std::shared_ptr<DFrame> DFramePtr;

	class LocateData
	{
	public:
		LocateData();
		~LocateData();

		trimesh::xform xf;
		bool lost;
	};
}