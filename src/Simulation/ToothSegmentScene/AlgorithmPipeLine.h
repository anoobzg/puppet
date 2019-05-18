#pragma once
#include <list>
#include <functional>

class Task
{
public:
	Task(std::function<void()> task) : m_actualTask(task) {}
	
	std::function<void()> m_actualTask;
};

class AlgorithmPipeLine
{
public:
	AlgorithmPipeLine() {}
	
	void AddTask(Task task);
	void RunAllTask();
	bool RunFirstTask();
	void DiscardAllTask();

	std::list<Task> m_taskQueue;
};