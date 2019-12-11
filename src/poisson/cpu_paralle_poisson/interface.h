#pragma once
#include "predefine.h"
#include "PointCloud.h"

using namespace LauncaGeometry;

void build_raw_points(PointCloud& cloud, std::vector<point>& points);
void build_octree_parameters(const std::vector<point>& points, vec3& recon_max, vec3& recon_min, unsigned& depth, float& grid_size);
void build_points_key(const std::vector<point>& points, const vec3& recon_max, const vec3& recon_min, unsigned depth, std::vector<index_key>& ikeys);
void build_map_points(std::vector<point>& points, const vec3& recon_max, const vec3& recon_min);
void build_sorted_points(const std::vector<point>& points, std::vector<index_key>& sorted_keys, std::vector<point>& sorted_points);
void build_unique_nodes(const std::vector<index_key>& sorted_keys, unsigned max_depth, std::vector<core_node>& nodes);
void expand_nodes(const std::vector<core_node>& dense_nodes, std::vector<core_node>& child_full_nodes, unsigned depth, std::vector<core_node>& full_nodes);
void build_next_depth_nodes(std::vector<core_node>& full_nodes, unsigned depth, std::vector<core_node>& next_dense_nodes);
void fill_childrens(const std::vector<core_node>& parents_nodes, const depth_info& parent_info, const depth_info& children_info, std::vector<node>& nodes);
void fill_parents(const std::vector<core_node>& children_nodes, const depth_info& parent_info, const depth_info& children_info, std::vector<node>& nodes);
void build_node_vertices(const std::vector<node>& nodes, const depth_info& info, std::vector<node_vertex>& nvertices);
void build_full_vertices(const std::vector<std::vector<node_vertex>>& nvertices, std::vector<vertex>& vertices);
void build_node_edges(const std::vector<node>& nodes, const depth_info& info, std::vector<node_edge>& nedges);
void build_full_edges(const std::vector<std::vector<node_edge>>& nedges, std::vector<edge>& edges);
void build_node_faces(const std::vector<node>& nodes, const depth_info& info, std::vector<node_face>& nfaces);
void build_full_faces(const std::vector<std::vector<node_face>>& nfaces, std::vector<face>& faces);