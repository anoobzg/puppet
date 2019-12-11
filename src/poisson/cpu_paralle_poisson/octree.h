#pragma once
#include "predefine.h"

class Octree
{
	friend class Solver;
public:
	Octree(unsigned depth);
	~Octree();

	void Build(const std::vector<point>& points, const vec3& max, const vec3& min);
protected:
	void BuildSortedPoints(const std::vector<point>& points, const vec3& max, const vec3& min, unsigned depth,
		std::vector<index_key>& sorted_keys, std::vector<point>& sorted_points);
	void BuildUniqueNodes(const std::vector<index_key>& sorted_keys, const std::vector<point>& sorted_points, unsigned max_depth,
		std::vector<core_node>& nodes);
	void BuildAllDepthNodes(std::vector<core_node>& nodes, std::vector<std::vector<core_node>>& all_depth_nodes, unsigned max_depth);

	void BuildRecursive(std::vector<core_node>& children_full_nodes, unsigned depth, std::vector<core_node>& parent_full_nodes);
	void BuildDepthRelationRecursive(const std::vector<core_node>& parent_nodes, const std::vector<core_node>& child_nodes,
		const depth_info& parent_depth_info, const depth_info& child_depth_info, std::vector<node>& nodes);
	void BuildNeighbors(std::vector<node>& nodes, std::vector<depth_info>& depth_infos);

	void BuildVertices();
	void BuildEdges();
	void BuildFaces();
private:
	unsigned m_depth;
	std::vector<point> m_sorted_points;
	std::vector<node> m_nodes;
	std::vector<depth_info> m_depth_info;

	std::vector<depth_info> m_vertices_depth_info;
	std::vector<vertex> m_vertices;

	std::vector<depth_info> m_edges_depth_info;
	std::vector<edge> m_edges;

	std::vector<depth_info> m_faces_depth_info;
	std::vector<face> m_faces;

	std::vector<float> m_phi;
	std::vector<float> m_point_phi;
};