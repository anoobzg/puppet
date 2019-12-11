#pragma once
#include "predefine.h"
#include "key.h"

bool check_points_number(const std::vector<core_node>& nodes, unsigned total);
bool check_relation(const std::vector<node>& all_nodes, const std::vector<depth_info>& infos);
bool check_relation_ex(const std::vector<node>& all_nodes, const std::vector<depth_info>& infos);
bool check_neighbors(const node& n, unsigned index);