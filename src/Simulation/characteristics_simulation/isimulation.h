#pragma once

struct icontrolpoint
{
	virtual unsigned get_handle() = 0;
};

struct ipath
{
	virtual unsigned get_handle() = 0;
};

struct isurface_topo_callback
{
	virtual void control_point_added(icontrolpoint& control_point) = 0;
	virtual void control_point_deleted(icontrolpoint& control_point) = 0;
	virtual void path_added(ipath& path) = 0;
	virtual void path_removed(ipath& path) = 0;
};

struct isurface
{
	virtual void set_topo_callback(isurface_topo_callback* callback) = 0;
	virtual bool add_control_point(unsigned vertex_handle) = 0;
	virtual void delete_control_point(unsigned control_point_handle) = 0;
	virtual bool modify_control_point(unsigned vertex_handle) = 0;
	virtual bool query_collid_control_point(unsigned& control_point_handle) = 0;
	virtual bool query_collid_path(unsigned& path_handle) = 0;
};