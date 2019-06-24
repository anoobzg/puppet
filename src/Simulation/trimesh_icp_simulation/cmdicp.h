#pragma once
#include "TriMesh.h"
#include "load_calib.h"

void cmd_test_icp(trimesh::TriMesh* source, trimesh::TriMesh* target, 
	const trimesh::CameraData& data);

void cmd_analysis_icp(trimesh::TriMesh* source, trimesh::TriMesh* target, const trimesh::CameraData& data,
	const std::string& error_csv_file, const std::string& time_csv_file);