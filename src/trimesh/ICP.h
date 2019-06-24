#ifndef ICP_H
#define ICP_H
/*
Szymon Rusinkiewicz
Princeton University

ICP.h
Iterative Closest Point alignment using symmetric point-to-plane minimization
and adaptive outlier rejection.
*/

#include "TriMesh.h"
#include "XForm.h"
#include "KDtree.h"
#include "tnt_matrix.h"
#include "projectionicpheader.h"
#include<sstream>
struct CalibData1
{
	float fx;
	float fy;
	float cx;
	float cy;
	float e_left1;
	float e_left2;
};
namespace trimesh {

class Grid;

// Make a grid for accelerating overlap computation
extern Grid *make_grid(TriMesh *mesh);

// Determine which points on s1 and s2 overlap the other, filling in o1 and o2
// Also fills in maxdist, if it is <= 0 on input
extern void compute_overlaps(TriMesh *mesh1, TriMesh *mesh2,
                             const xform &xf1, const xform &xf2,
                             const KDtree *kd1, const KDtree *kd2,
                             const Grid *g1, const Grid *g2,
                             ::std::vector<float> &o1, ::std::vector<float> &o2,
                             float &maxdist, int verbose);

//void select_and_match(TriMesh *cloud1, TriMesh *cloud2, const float fx,
//	const float fy, const float cx, const float cy,
//	const xform &xf1, const xform &xf2, vector<PtPair> &pairs);

// Do ICP.  Aligns mesh2 to mesh1, updating xf2 with the new transform.
// Returns alignment error, or -1 on failure.
// Pass in 0 for maxdist to figure it out...
// Pass in empty vector for weights to figure it out...
extern float ICP(TriMesh *cloud1, TriMesh *cloud2,
	const xform &xf1, xform &xf2, const float fx,
	const float fy, const float cx, const float cy,
	float maxdist /* = 0.0f */, int verbose /* = 0 */, TNT::Matrix<float>& e_left);

void point_align_fusion(TriMesh *cloud1, TriMesh *cloud2, const float fx,
	const float fy, const float cx, const float cy,
	const xform &xf1, const xform &xf2);

// 对point_align_fusion进行代码优化，并且线程数改成只用一半 --- 2019.5.20 by HeJinying
void point_align_fusion_optim(TriMesh *cloud1, TriMesh *cloud2, const float fx,
	const float fy, const float cx, const float cy,
	const xform &xf1, const xform &xf2);

// 点云带置信度的融合，记录每个点云最高置信度时所在的帧序号 --- 2019.5.28 by HeJinying
// 暂时没记录每个点被融合的次数
void point_align_fusion_with_confidence(TriMesh *cloud1, TriMesh *cloud2, const float fx,
	const float fy, const float cx, const float cy,
	const xform &xf1, const xform &xf2, const int frame_idx);

// 融合时截取出当前帧视角包围到的点云 --- 2019.5.17 by HeJinying
void point_align_fusion_last(TriMesh *cloud1, TriMesh *cloud2, TriMesh *cloud_last, const float fx,
	const float fy, const float cx, const float cy,
	const xform &xf1, const xform &xf2);

// 带颜色的点云融合 --- 2019.4.20 by HeJinying
void point_align_fusion_with_color(TriMesh *cloud1, TriMesh *cloud2, const float fx,
	const float fy, const float cx, const float cy,
	const xform &xf1, const xform &xf2);
}

#endif
