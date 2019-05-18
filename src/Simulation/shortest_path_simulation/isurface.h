#pragma once
#include <vector>

struct isurface
{
	virtual unsigned create_control_point(unsigned vertex_handle) = 0;
	virtual void remove_control_point(unsigned point_handle) = 0;
	virtual void modify_control_point(unsigned point_handle, unsigned vertex_handle) = 0;
};

struct ialgrithm
{
	virtual void get_shortest_path(unsigned vertex_handle1, unsigned vertex_handle2, std::vector<unsigned>& path) = 0;
	virtual void search_proper_path(unsigned vertex_handle, std::vector<unsigned>& path) = 0;
};

struct icontrolpoint
{

};