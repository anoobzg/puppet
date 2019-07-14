#pragma once
#include "DFrame.h"
#include <base/synchronization/lock.h>
#include <list>

namespace esslam
{
	class ESSLAM_API DFramePool
	{
	public:
		DFramePool();
		~DFramePool();

		void Melloc(int size, int width, int height);
		void Free();

		DFrame* Get();
		void Release(DFrame* frame);
	private:
		std::list<DFrame*> m_datas;
		base::Lock m_lock;

		int m_width;
		int m_height;
	};
}