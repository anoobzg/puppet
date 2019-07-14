#include "DFramePool.h"

namespace esslam
{

	DFramePool::DFramePool()
		:m_width(1920), m_height(1080)
	{
	}

	DFramePool::~DFramePool()
	{
	}

	void DFramePool::Melloc(int size, int width, int height)
	{
		m_width = width;
		m_height = height;
		int alloc_num = 5;
		if (size > 0) alloc_num = size;
		for (int i = 0; i < alloc_num; ++i)
			m_datas.push_back(new DFrame(m_width, m_height));
	}

	void DFramePool::Free()
	{
		for (std::list<DFrame*>::iterator it = m_datas.begin(); it != m_datas.end(); ++it)
			delete *it;
		m_datas.clear();
	}

	DFrame* DFramePool::Get()
	{
		DFrame* frame = 0;
		m_lock.Acquire();
		if (m_datas.size() > 0)
		{
			frame = *m_datas.begin();
			m_datas.pop_front();
		}
		m_lock.Release();

		if (!frame)
		{
			std::cout << "Warning DFrame New Alloc." << std::endl;
			frame = new DFrame(m_width, m_height);
		}
		return frame;
	}

	void DFramePool::Release(DFrame* frame)
	{
		if (frame)
		{
			m_lock.Acquire();
			m_datas.push_back(frame);
			m_lock.Release();
		}
	}
}