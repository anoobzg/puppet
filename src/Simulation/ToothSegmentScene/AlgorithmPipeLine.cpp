#include "AlgorithmPipeLine.h"
#include <windows.h>
#include <iostream>

void AlgorithmPipeLine::AddTask(Task task)
{
	m_taskQueue.push_back(task);
}

void AlgorithmPipeLine::RunAllTask()
{
	ULONGLONG uLastTime = GetTickCount64();
	ULONGLONG uBeginTime = uLastTime;
	unsigned uLoopCount = 0;

	while (RunFirstTask())
	{
		std::cout << "[Performance Info] TaskIndex: " << uLoopCount++ << ", Cost time: " << GetTickCount64() - uLastTime << " ms" << std::endl;
		uLastTime = GetTickCount64();
	}

	std::cout << "[Performance Info] Task total cost time: " << GetTickCount64() - uBeginTime << " ms." << std::endl;
}

bool AlgorithmPipeLine::RunFirstTask()
{
	if (m_taskQueue.begin() == m_taskQueue.end())
	{
		return false;
	}

	(*m_taskQueue.begin()).m_actualTask();
	m_taskQueue.erase(m_taskQueue.begin());
	return true;
}

void AlgorithmPipeLine::DiscardAllTask()
{
	m_taskQueue.clear();
}
