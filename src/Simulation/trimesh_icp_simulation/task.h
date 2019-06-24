#pragma once
#include <osg\Object>

class Task : public osg::Referenced
{
public:
	Task();
	virtual ~Task();

	virtual bool Execute(); // false will release
};