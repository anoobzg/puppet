#pragma once
#include <vector>

class UseCase
{
public:
	virtual ~UseCase() {}
	virtual void Setup(std::vector<int>& datas, std::vector<int>& expected_result) = 0;
};

class Case1 : public UseCase
{
public:
	virtual void Setup(std::vector<int>& datas, std::vector<int>& expected_result);
};

class Case2 : public UseCase
{
public:
	virtual void Setup(std::vector<int>& datas, std::vector<int>& expected_result);
};

class Case3 : public UseCase
{
public:
	virtual void Setup(std::vector<int>& datas, std::vector<int>& expected_result);
};

class Case4 : public UseCase
{
public:
	virtual void Setup(std::vector<int>& datas, std::vector<int>& expected_result);
};