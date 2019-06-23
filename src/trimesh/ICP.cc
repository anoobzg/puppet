/*
Szymon Rusinkiewicz
Princeton University

ICP.cc
Iterative Closest Point alignment using covariance-weighted sampling,
adaptive outlier rejection, and symmetric point-to-plane minimization.
*/

#include <functional>   // std::greater
#include <algorithm>    // std::sort
#include <numeric>
#include "ICP.h"
#include "timestamp.h"
#include "lineqn.h"
#include <omp.h>
using namespace std;

#define INITIAL_ITERS 2
#define MAX_ITERS 80
#define TERMINATION_ITER_THRESH 20
#define FINAL_ITERS 2
#define MIN_PAIRS 10
#define DESIRED_PAIRS 500
#define DESIRED_PAIRS_FINAL 5000
#define CDF_UPDATE_INTERVAL 20
#define REJECT_BDY false
#define USE_NORMCOMPAT true
#define REGULARIZATION 0.005f
#define MEDIAN_TO_SIGMA 1.4826f
#define DIST_THRESH_MULT_START 5.0f
#define DIST_THRESH_MULT_FINAL 2.0f
#define NORMDOT_THRESH_START 0.6f
#define NORMDOT_THRESH_FINAL 0.9f
#define THRESH_RATE_OF_CHANGE 0.002f
#define dprintf TriMesh::dprintf


namespace trimesh {

	// A spatial grid datastructure for fast overlap computation
	class Grid {
	private:
		enum { GRID_SHIFT = 4, GRID_MAX = (1 << GRID_SHIFT) - 1 };
		float xmin, xmax, ymin, ymax, zmin, zmax, scale;
		vector<char> g;
	public:
		void getXYZLimit(float& xxmin, float& xxmax,
			float& yymin, float& yymax,
			float& zzmin, float& zzmax) {
			xxmin = xmin; xxmax = xmax;
			yymin = ymin; yymax = ymax;
			zzmin = zmin; zzmax = zmax;
		};
		inline float bbox_size() const
		{
			return dist(point(xmin, ymin, zmin), point(xmax, ymax, zmax));
		}
		inline bool valid(const point &p) const
		{
			return p[0] >= xmin && p[1] >= ymin && p[2] >= zmin &&
				p[0] <= xmax && p[1] <= ymax && p[2] <= zmax;
		}

		inline std::vector<point> point_bbox() const
		{
			std::vector<point> points;
			points.push_back(point(xmin, ymin, zmin));
			points.push_back(point(xmax, ymin, zmin));
			points.push_back(point(xmax, ymax, zmin));
			points.push_back(point(xmax, ymax, zmax));
			points.push_back(point(xmin, ymax, zmax));
			points.push_back(point(xmax, ymin, zmax));
			points.push_back(point(xmin, ymin, zmax));
			points.push_back(point(xmin, ymax, zmin));
			return points;
		}

		// return the index of the point by (xcell,ycell,zcell) ---- by tim 2018.12.10
		inline int ind(int xcell, int ycell, int zcell) const
		{
			xcell = clamp(xcell, 0, int(GRID_MAX));
			ycell = clamp(ycell, 0, int(GRID_MAX));
			zcell = clamp(zcell, 0, int(GRID_MAX));
			return (xcell << (2 * GRID_SHIFT)) +
				(ycell << GRID_SHIFT) +
				zcell;
		}
		inline int ind(const point &p) const
		{
			return ind(int(scale * (p[0] - xmin)),
				int(scale * (p[1] - ymin)),
				int(scale * (p[2] - zmin)));
		}
		inline bool overlaps(const point &p) const
		{
			return valid(p) && g[ind(p)];
		}
		Grid(const vector<point> &pts);
	};


	// Compute a Grid from a list of points
	Grid::Grid(const vector<point> &pts)
	{
		size_t gsize = (1 << (3 * GRID_SHIFT));
		g.resize(gsize);
		if (pts.empty()) {
			xmin = xmax = ymin = ymax = zmin = zmax = scale = 0.0f;
			return;
		}

		// Find bounding box of pts
		xmin = xmax = pts[0][0];
		ymin = ymax = pts[0][1];
		zmin = zmax = pts[0][2];
		size_t npts = pts.size();
		for (size_t i = 1; i < npts; i++) {
			if (pts[i][0] < xmin) xmin = pts[i][0];
			else if (pts[i][0] > xmax) xmax = pts[i][0];
			if (pts[i][1] < ymin) ymin = pts[i][1];
			else if (pts[i][1] > ymax) ymax = pts[i][1];
			if (pts[i][2] < zmin) zmin = pts[i][2];
			else if (pts[i][2] > zmax) zmax = pts[i][2];
		}
		scale = 1.0f / max(max(xmax - xmin, ymax - ymin), zmax - zmin);
		scale *= float(1 << GRID_SHIFT);

	
	}


	// Return Grid for a mesh
	Grid *make_grid(TriMesh *mesh)
	{
		return new Grid(mesh->vertices);
	}

	// Select a number of points and find correspondences
	// pts means ??
	static void select_and_match(TriMesh *cloud1, TriMesh *cloud2, std::vector<point> pts, const float fx,
		const float fy, const float cx, const float cy,
		const xform &xf1, const xform &xf2, vector<PtPair> &pairs,TNT::Matrix<float>& e_left)
	{
		clock_t t1 = clock();
		pairs.clear();

		//xform nxf1 = norm_xf(xf1);
		//xform nxf2 = norm_xf(xf2);
		//xform xf12 = inv(xf2) * xf1;
		//xform nxf12 = norm_xf(xf12);

		xform nxf1 = xf1;
		nxf1[12] = 0;
		nxf1[13] = 0;
		nxf1[14] = 0;
		xform nxf2 = xf2;
		nxf2[12] = 0;
		nxf2[13] = 0;
		nxf2[14] = 0;
		xform xf12 = inv(xf2) * xf1;
		xform nxf12 = xf12;
		nxf12[12] = 0;
		nxf12[13] = 0;
		nxf12[14] = 0;

		size_t npts = pts.size();
		for (auto i = 0; i < npts; i++)
		{
			pts[i] = xf2* pts[i];
		}
		float xmin, xmax, ymin, ymax, zmin, zmax;
		xmin = xmax = pts[0][0];
		ymin = ymax = pts[0][1];
		zmin = zmax = pts[0][2];
		
		for (size_t i = 1; i < npts; i++) {
			if (pts[i][0] < xmin) xmin = pts[i][0];
			else if (pts[i][0] > xmax) xmax = pts[i][0];
			if (pts[i][1] < ymin) ymin = pts[i][1];
			else if (pts[i][1] > ymax) ymax = pts[i][1];
			if (pts[i][2] < zmin) zmin = pts[i][2];
			else if (pts[i][2] > zmax) zmax = pts[i][2];
		}
		//TriMesh cloud;
		//for (auto i  = 0; i < cloud2->vertices.size(); i++)
		//{
		//	cloud.vertices.push_back(xf2* cloud2->vertices[i]);
		//	cloud.normals.push_back(nxf2* cloud2->normals[i]);
		//}
		//cloud.write("norm:ply_ascii:1.ply");
		//std::cout << xmin << " " << xmax << " " << ymin << " " << ymax << " " << zmin << " " << zmax << std::endl;
		const int sz = floor(cloud2->vertices.size()/1000);
		int nv = (int)cloud1->vertices.size() / sz;
		vector<PtPair> 	pairs1(nv);
		vector<int> idx(nv, 0);
		int num_omp = omp_get_num_procs();
//#pragma omp parallel for num_threads(num_omp)
		{
#pragma omp parallel for num_threads(num_omp)
			for (int i = 0; i < nv; i++)
			{
				if (cloud1->vertices[i*sz].x>xmin&&cloud1->vertices[i*sz].x<xmax&&
					cloud1->vertices[i*sz].y>ymin&&cloud1->vertices[i*sz].y<ymax&&
					cloud1->vertices[i*sz].z>zmin&&cloud1->vertices[i*sz].z < zmax)
				{
					point p = xf12 *cloud1->vertices[i*sz];

					// Project source vertex into the destination's image plane.
					int u = (int)roundf((fx * p[0] + cx * p[2]) / p[2]);
					int v = (int)roundf((fy * p[1] + cy * p[2]) / p[2]);

					// Check corresponding vertex
					if ((u >= 1 && u < cloud2->grid_width-1) && (v >= 1 && v < cloud2->grid_height-1)) {
						int j = u + v * cloud2->grid_width;
						if (cloud2->grid[j] >= 0 && cloud2->grid[j] < cloud2->vertices.size())
						{
							// Project both points into world coords and save
							point p1 = xf1 * cloud1->vertices[i*sz];
							vec n1 = nxf1 * cloud1->normals[i*sz];
							point p2 = xf2 * cloud2->vertices[cloud2->grid[j]];
							vec n2 = nxf2 * cloud2->normals[cloud2->grid[j]];
							if (n1.dot(n2) > 0.6)
							{
								pairs1[i] = PtPair(p1, n1, p2, n2);
								idx[i] = 1;
							}
						}
					}
				}
			}
		}
		int num = std::accumulate(idx.begin(), idx.end(), 0);
		int k = num;
		int n = 0;
		int m = 0;
		int num_erase = 0;

		pairs.resize(k);
		for (int i = 0; i < nv; i++)
		{
			if (idx[i] == 1)
			{
				pairs[n] = pairs1[i];
				n++;
			}
		}
		//}
		//double t2 = double(clock() - t1) * 1000 / CLOCKS_PER_SEC;
		//if (0)
		//{
		//	cout << " ********************* cloud last size :" << cloud1->vertices.size() << endl;
		//	cout << " ********************* nv :" << nv << endl;
		//	cout << " ********************* select and match pairs:" << pairs.size() << std::endl;
		//	cout << " ********************* select and match time:" << t2 << endl;
		//}
	}

	void point_align_fusion(TriMesh *cloud1, TriMesh *cloud2,
		const float fx, const float fy, const float cx,
		const float cy, const xform &xf1, const xform &xf2)
	{
		clock_t t1 = clock();
		xform nxf1 = norm_xf(xf1);
		xform nxf2 = norm_xf(xf2);
		xform xf12 = inv(xf2) * xf1;
		xform nxf12 = norm_xf(xf12);
		xform ixf1 = inv(xf1);
		xform nixf1 = norm_xf(ixf1);
		//xform ixf2 = inv(xf2);
		//xform nixf2 = norm_xf(ixf2);
		//newPnt.clear();
		//newNrm.clear();
		const int nv2 = cloud2->vertices.size();

		std::vector<point> vertices_g2(nv2);
		std::vector<vec> normals_g2(nv2);
#pragma omp parallel for
		//将cloud2转换到全局坐标系下
		for (int i = 0; i < nv2; i++)
		{
			vertices_g2[i] = xf2*cloud2->vertices[i];
			normals_g2[i] = nxf2*cloud2->normals[i];
		}

		const int nv1 = cloud1->vertices.size();
		std::vector<int> idx(nv1, 0); //用于记录cloud1中和cloud2对应的点

		//std::vector<std::vector<int>>  temp_con(nv2, std::vector<int>(50,0)); //用于记录cloud2中每个点和cloud1对应的点
		std::vector<std::vector<int>>  temp_con(nv2); //用于记录cloud2中每个点和cloud1对应的点
		//std::vector<int> temp_idx(nv2, 0); //用于记录cloud2中每个点和cloud1对应的点的个数

		clock_t t2 = clock();
		for (int i = 0; i < nv1; i++)
		{
			//将cloud1转化到cloud2相机坐标系下
			point p = xf12 * cloud1->vertices[i];
			vec n = nxf12 * cloud1->normals[i];

			//// 加包围盒约束
			//if (p.x<cloud2->bbox.min.x || p.x>cloud2->bbox.max.x
			//	|| p.y<cloud2->bbox.min.y || p.y>cloud2->bbox.max.y
			//	|| p.z<cloud2->bbox.min.z || p.z>cloud2->bbox.max.z)
			//	continue;

			// Project source vertex into the destination's image plane.
			//将cloud1投影到图像平面，计算图像平面的像素位置
			int u = (int)roundf((fx * p[0] + cx * p[2]) / p[2]);
			int v = (int)roundf((fy * p[1] + cy * p[2]) / p[2]);
			
			// Check corresponding vertex
			if ((u >= 1 && u < cloud2->grid_width-1) && (v >= 1 && v < cloud2->grid_height-1)) //确保像素位置在cloud2的图像平面中
			{
				int j = u + v * cloud2->grid_width;	//计算cloud2图像平面位置的对应点位置
				if (cloud2->grid[j] >= 0 && n.dot(cloud2->normals[cloud2->grid[j]]) > 0.90&&dist2(p, cloud2->vertices[cloud2->grid[j]])<1.0)
				{	//记录cloud2与cloud1的对应点
					//temp_con[cloud2->grid[j]][temp_idx[cloud2->grid[j]]] = i;
					temp_con[cloud2->grid[j]].push_back(i);
					//temp_idx[cloud2->grid[j]]++;
					//std::cout << temp_idx[cloud2->grid[j]] << std::endl;
					idx[i] = 1;
				}
			}
		}
		cout << "fusion reproject time:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		int num_erase = std::accumulate(idx.begin(), idx.end(), 0);	//cloud1中找到对应点的个数
#pragma omp parallel for
		for (int k = 0; k < nv2; k++)
		{
			//将cloud2转化到cloud1的相机坐标系下
			vertices_g2[k] = ixf1*vertices_g2[k];
			normals_g2[k] = nixf1*normals_g2[k];
			int temp_num = temp_con[k].size();
			if (/*temp_idx[k]*/temp_num > 0)
			{
				//将对应的cloud1对应点相加求平均作为cloud1改点的新值
				for (int i = 0; i < /*temp_idx[k]*/temp_num; i++)
				{
					vertices_g2[k] += cloud1->vertices[temp_con[k][i]];
					//normals_g2[k] += cloud1->normals[temp_con[k][i]];
				}
				vertices_g2[k] = (vertices_g2[k] / (/*temp_idx[k]*/temp_num + 1));
				normals_g2[k] = normalized(normals_g2[k]);
			}
			else {
				//newPnt.push_back(vertices_g2[k]);
				//newNrm.push_back(normals_g2[k]);
			}
		}
		
		std::vector<point> vertices_1(nv1 - num_erase);
		std::vector<vec> normals_1(nv1 - num_erase);

		int k = 0;
		//去除cloud1中的对应点，将剩余的点记录在vertices_1中
		for (int i = 0; i < nv1; i++)
		{
			if (idx[i] != 1 && k < vertices_1.size())
			{
				vertices_1[k] = cloud1->vertices[i];
				normals_1[k] = cloud1->normals[i];
				k++;
			}
		}

		cloud1->vertices.clear();
		cloud1->normals.clear();
		vertices_1.swap(cloud1->vertices);
		normals_1.swap(cloud1->normals);
		for (int i = 0; i < nv2; i++)
		{
			if (normals_g2[i].x == 0 && normals_g2[i].y == 0 && normals_g2[i].z == 0)
				continue;
			cloud1->vertices.push_back(vertices_g2[i]);
			cloud1->normals.push_back(normals_g2[i]);
		}

		cout << "fusion time:" << double(clock() - t1) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
	}

	void point_align_fusion_optim(TriMesh *cloud1, TriMesh *cloud2,
		const float fx, const float fy, const float cx,
		const float cy, const xform &xf1, const xform &xf2)
	{
		clock_t t1 = clock();
		xform nxf1 = norm_xf(xf1);
		xform nxf2 = norm_xf(xf2);
		xform xf12 = inv(xf2) * xf1;
		xform nxf12 = norm_xf(xf12);
		xform ixf1 = inv(xf1);
		xform nixf1 = norm_xf(ixf1);

		const int nv2 = cloud2->vertices.size();
		const int nv1 = cloud1->vertices.size();

		t1 = clock();
		clock_t t2 = clock();
		static trimesh::TriMesh cloud_tmp;
		static vector<int> u_tmp;
		static vector<int> v_tmp;
		cloud_tmp.vertices.resize(nv1);
		cloud_tmp.normals.resize(nv1);

		u_tmp.resize(nv1);
		v_tmp.resize(nv1);
		//cout << "vertices size():" << cloud1->vertices.size() << endl;
		//cout << "data copy time:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
		int num_mp = omp_get_num_procs();
		//cout << "num openmp:" << num_mp << endl;
#pragma omp parallel for num_threads(num_mp/2)
		for (int i = 0; i < cloud_tmp.vertices.size(); i++)
		{
			point p = xf12 * cloud1->vertices[i];
			vec n = nxf12 * cloud1->normals[i];
			cloud_tmp.vertices[i] = p;
			cloud_tmp.normals[i] = n;

			u_tmp[i] = (int)roundf((fx * p[0] + cx * p[2]) / p[2]);
			v_tmp[i] = (int)roundf((fy * p[1] + cy * p[2]) / p[2]);
		}
		//cout << "fusion reproject parallel test time:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		std::vector<int> idx_tmp(nv1, 0);
		std::vector<std::vector<int>>  temp_con1(nv2);
		for (int i = 0; i < nv1; i++)
		{
			point p = cloud_tmp.vertices[i];
			vec n = cloud_tmp.normals[i];

			// 当前帧看不到的点
			if (n.z >= 0)
				continue;

			// 加包围盒约束
			if (p.x<cloud2->bbox.min.x || p.x>cloud2->bbox.max.x
				|| p.y<cloud2->bbox.min.y || p.y>cloud2->bbox.max.y
				|| p.z<cloud2->bbox.min.z || p.z>cloud2->bbox.max.z)
				continue;

			int u = u_tmp[i];
			int v = v_tmp[i];
			// Check corresponding vertex
			if ((u >= 1 && u < cloud2->grid_width - 1) && (v >= 1 && v < cloud2->grid_height - 1)) //确保像素位置在cloud2的图像平面中
			{
				int j = u + v * cloud2->grid_width;	//计算cloud2图像平面位置的对应点位置
				if (cloud2->grid[j] >= 0 && n.dot(cloud2->normals[cloud2->grid[j]]) > 0.90&&dist2(p, cloud2->vertices[cloud2->grid[j]])<1.0)
				{	//记录cloud2与cloud1的对应点
					//temp_con[cloud2->grid[j]][temp_idx[cloud2->grid[j]]] = i;
					temp_con1[cloud2->grid[j]].push_back(i);
					//temp_idx[cloud2->grid[j]]++;
					//std::cout << temp_idx[cloud2->grid[j]] << std::endl;
					idx_tmp[i] = 1;
				}
			}
		}
		//cout << "全部点云RT，重投影，找对应点时间:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		t2 = clock();
		std::vector<point> vertices_g2(nv2);
		std::vector<vec> normals_g2(nv2);
		//将cloud2转换到全局坐标系下
#pragma omp parallel for num_threads(num_mp/2)
		for (int i = 0; i < nv2; i++)
		{
			vertices_g2[i] = xf2*cloud2->vertices[i];
			normals_g2[i] = nxf2*cloud2->normals[i];
		}

#pragma omp parallel for num_threads(num_mp/2)
		for (int k = 0; k < nv2; k++)
		{
			//将cloud2转化到cloud1的相机坐标系下
			vertices_g2[k] = ixf1*vertices_g2[k];
			normals_g2[k] = nixf1*normals_g2[k];
			int temp_num = temp_con1[k].size();
			if (/*temp_idx[k]*/temp_num > 0)
			{
				//将对应的cloud1对应点相加求平均作为cloud1改点的新值
				for (int i = 0; i < /*temp_idx[k]*/temp_num; i++)
				{
					vertices_g2[k] += cloud1->vertices[temp_con1[k][i]];
					//normals_g2[k] += cloud1->normals[temp_con[k][i]];
				}
				vertices_g2[k] = (vertices_g2[k] / (/*temp_idx[k]*/temp_num + 1));
				normals_g2[k] = normalized(normals_g2[k]);
			}
			else {
				//newPnt.push_back(vertices_g2[k]);
				//newNrm.push_back(normals_g2[k]);
			}
		}
		//cout << "当前帧点云并行融合时间:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		int num_erase = std::accumulate(idx_tmp.begin(), idx_tmp.end(), 0);	//cloud1中找到对应点的个数
		std::vector<point> vertices_1(nv1 - num_erase);
		std::vector<vec> normals_1(nv1 - num_erase);

		int k = 0;
		//去除cloud1中的对应点，将剩余的点记录在vertices_1中
		for (int i = 0; i < nv1; i++)
		{
			if (idx_tmp[i] != 1 && k < vertices_1.size())
			{
				vertices_1[k] = cloud1->vertices[i];
				normals_1[k] = cloud1->normals[i];
				k++;
			}
		}

		cloud1->vertices.clear();
		cloud1->normals.clear();
		vertices_1.swap(cloud1->vertices);
		normals_1.swap(cloud1->normals);
		for (int i = 0; i < nv2; i++)
		{
			if (normals_g2[i].x == 0 && normals_g2[i].y == 0 && normals_g2[i].z == 0)
				continue;
			cloud1->vertices.push_back(vertices_g2[i]);
			cloud1->normals.push_back(normals_g2[i]);
		}

		cout << "cloud all size:" << cloud1->vertices.size() << endl;
		cout << "cloud now size:" << cloud2->vertices.size() << endl;
		cout << "fusion time:" << double(clock() - t1) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
	}

	void point_align_fusion_with_confidence(TriMesh *cloud1, TriMesh *cloud2, const float fx,
		const float fy, const float cx, const float cy,
		const xform &xf1, const xform &xf2, const int frame_idx)
	{
		clock_t t1 = clock();
		xform nxf1 = norm_xf(xf1);
		xform nxf2 = norm_xf(xf2);
		xform xf12 = inv(xf2) * xf1;
		xform nxf12 = norm_xf(xf12);
		xform ixf1 = inv(xf1);
		xform nixf1 = norm_xf(ixf1);

		const int nv2 = cloud2->vertices.size();
		const int nv1 = cloud1->vertices.size();

		t1 = clock();
		clock_t t2 = clock();
		static trimesh::TriMesh cloud_tmp;
		static vector<int> u_tmp;
		static vector<int> v_tmp;
		cloud_tmp.vertices.resize(nv1);
		cloud_tmp.normals.resize(nv1);

		u_tmp.resize(nv1);
		v_tmp.resize(nv1);
		//cout << "vertices size():" << cloud1->vertices.size() << endl;
		//cout << "data copy time:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
		int num_mp = omp_get_num_procs();
		//cout << "num openmp:" << num_mp << endl;
#pragma omp parallel for num_threads(num_mp/2)
		for (int i = 0; i < cloud_tmp.vertices.size(); i++)
		{
			point p = xf12 * cloud1->vertices[i];
			vec n = nxf12 * cloud1->normals[i];
			cloud_tmp.vertices[i] = p;
			cloud_tmp.normals[i] = n;

			u_tmp[i] = (int)roundf((fx * p[0] + cx * p[2]) / p[2]);
			v_tmp[i] = (int)roundf((fy * p[1] + cy * p[2]) / p[2]);
		}
		//cout << "fusion reproject parallel test time:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		std::vector<int> idx_tmp(nv1, 0);
		std::vector<std::vector<int>>  temp_con1(nv2);
		for (int i = 0; i < nv1; i++)
		{
			point p = cloud_tmp.vertices[i];
			vec n = cloud_tmp.normals[i];

			// 当前帧看不到的点
			if (n.z >= 0)
				continue;

			// 加包围盒约束
			if (p.x<cloud2->bbox.min.x || p.x>cloud2->bbox.max.x
				|| p.y<cloud2->bbox.min.y || p.y>cloud2->bbox.max.y
				|| p.z<cloud2->bbox.min.z || p.z>cloud2->bbox.max.z)
				continue;

			int u = u_tmp[i];
			int v = v_tmp[i];
			// Check corresponding vertex
			if ((u >= 1 && u < cloud2->grid_width - 1) && (v >= 1 && v < cloud2->grid_height - 1)) //确保像素位置在cloud2的图像平面中
			{
				int j = u + v * cloud2->grid_width;	//计算cloud2图像平面位置的对应点位置
				if (cloud2->grid[j] >= 0 && n.dot(cloud2->normals[cloud2->grid[j]]) > 0.90&&dist2(p, cloud2->vertices[cloud2->grid[j]])<1.0)
				{	//记录cloud2与cloud1的对应点
					//temp_con[cloud2->grid[j]][temp_idx[cloud2->grid[j]]] = i;
					temp_con1[cloud2->grid[j]].push_back(i);
					//temp_idx[cloud2->grid[j]]++;
					//std::cout << temp_idx[cloud2->grid[j]] << std::endl;
					idx_tmp[i] = 1;
				}
			}
		}
		//cout << "全部点云RT，重投影，找对应点时间:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		t2 = clock();
		std::vector<point> vertices_g2(nv2);
		std::vector<vec> normals_g2(nv2);
		std::vector<float> confidence_g2(nv2);
		std::vector<int> best_frame_idx_g2(nv2);
		//将cloud2转换到全局坐标系下
#pragma omp parallel for num_threads(num_mp/2)
		for (int i = 0; i < nv2; i++)
		{
			vertices_g2[i] = xf2*cloud2->vertices[i];
			normals_g2[i] = nxf2*cloud2->normals[i];
			confidence_g2[i] = cloud2->confidences[i];
			best_frame_idx_g2[i] = cloud2->best_frame_idx[i];
		}

#pragma omp parallel for num_threads(num_mp/2)
		for (int k = 0; k < nv2; k++)
		{
			//将cloud2转化到cloud1的相机坐标系下
			vertices_g2[k] = ixf1*vertices_g2[k];
			normals_g2[k] = nixf1*normals_g2[k];
			int temp_num = temp_con1[k].size();

			float confidence_max = confidence_g2[k];
			int frame_idx_final = frame_idx;
			if (/*temp_idx[k]*/temp_num > 0)
			{
				//将对应的cloud1对应点相加求平均作为cloud1改点的新值
				for (int i = 0; i < /*temp_idx[k]*/temp_num; i++)
				{
					vertices_g2[k] += cloud1->vertices[temp_con1[k][i]];
					if (cloud1->confidences[temp_con1[k][i]] > confidence_max)
					{
						confidence_max = cloud1->confidences[temp_con1[k][i]];
						frame_idx_final = cloud1->best_frame_idx[temp_con1[k][i]];
					}
				}
				vertices_g2[k] = (vertices_g2[k] / (/*temp_idx[k]*/temp_num + 1));
				normals_g2[k] = normalized(normals_g2[k]);	
			}
			confidence_g2[k] = confidence_max;
			best_frame_idx_g2[k] = frame_idx_final;
		}
		//cout << "当前帧点云并行融合时间:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		int num_erase = std::accumulate(idx_tmp.begin(), idx_tmp.end(), 0);	//cloud1中找到对应点的个数
		std::vector<point> vertices_1(nv1 - num_erase);
		std::vector<vec> normals_1(nv1 - num_erase);
		std::vector<float> confidence_1(nv1 - num_erase);
		std::vector<int> best_frame_idx_1(nv1 - num_erase);

		int k = 0;
		//去除cloud1中的对应点，将剩余的点记录在vertices_1中
		for (int i = 0; i < nv1; i++)
		{
			if (idx_tmp[i] != 1 && k < vertices_1.size())
			{
				vertices_1[k] = cloud1->vertices[i];
				normals_1[k] = cloud1->normals[i];
				confidence_1[k] = cloud1->confidences[i];
				best_frame_idx_1[k] = cloud1->best_frame_idx[i];
				k++;
			}
		}

		cloud1->vertices.clear();
		cloud1->normals.clear();
		cloud1->confidences.clear();
		cloud1->best_frame_idx.clear();

		vertices_1.swap(cloud1->vertices);
		normals_1.swap(cloud1->normals);
		confidence_1.swap(cloud1->confidences);
		best_frame_idx_1.swap(cloud1->best_frame_idx);

		for (int i = 0; i < nv2; i++)
		{
			if (normals_g2[i].x == 0 && normals_g2[i].y == 0 && normals_g2[i].z == 0)
				continue;
			cloud1->vertices.push_back(vertices_g2[i]);
			cloud1->normals.push_back(normals_g2[i]);
			cloud1->confidences.push_back(confidence_g2[i]);
			cloud1->best_frame_idx.push_back(best_frame_idx_g2[i]);
		}

		cout << "fusion time:" << double(clock() - t1) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
	}

	void point_align_fusion_last(TriMesh *cloud1, TriMesh *cloud2, TriMesh *cloud_last,
		const float fx, const float fy, const float cx,
		const float cy, const xform &xf1, const xform &xf2)
	{
		vector<trimesh::point> points_tmp;
		vector<trimesh::vec> norms_tmp;

		clock_t t1 = clock();
		cloud_last->clear();
		xform nxf1 = norm_xf(xf1);
		xform nxf2 = norm_xf(xf2);
		xform xf12 = inv(xf2) * xf1;
		xform nxf12 = norm_xf(xf12);
		xform ixf1 = inv(xf1);
		xform nixf1 = norm_xf(ixf1);
		//xform ixf2 = inv(xf2);
		//xform nixf2 = norm_xf(ixf2);
		//newPnt.clear();
		//newNrm.clear();
		const int nv2 = cloud2->vertices.size();
		const int nv1 = cloud1->vertices.size();
		std::vector<int> idx(nv1, 0); //用于记录cloud1中和cloud2对应的点

									  //std::vector<std::vector<int>>  temp_con(nv2, std::vector<int>(50,0)); //用于记录cloud2中每个点和cloud1对应的点
		std::vector<std::vector<int>>  temp_con(nv2); //用于记录cloud2中每个点和cloud1对应的点
													
													  //std::vector<int> temp_idx(nv2, 0); //用于记录cloud2中每个点和cloud1对应的点的个数
		clock_t t2 = clock();
		for (int i = 0; i < nv1; i++)
		{
			//将cloud1转化到cloud2相机坐标系下
			point p = xf12 * cloud1->vertices[i];
			vec n = nxf12 * cloud1->normals[i];

			// 加包围盒判断?

			// Project source vertex into the destination's image plane.
			//将cloud1投影到图像平面，计算图像平面的像素位置
			int u = (int)roundf((fx * p[0] + cx * p[2]) / p[2]);
			int v = (int)roundf((fy * p[1] + cy * p[2]) / p[2]);

			// Check corresponding vertex
			if ((u >= 1 && u < cloud2->grid_width - 1) && (v >= 1 && v < cloud2->grid_height - 1)) //确保像素位置在cloud2的图像平面中
			{
				int j = u + v * cloud2->grid_width;	//计算cloud2图像平面位置的对应点位置
				if (cloud2->grid[j] >= 0 && n.dot(cloud2->normals[cloud2->grid[j]]) > 0.90 && dist2(p, cloud2->vertices[cloud2->grid[j]])<1.0)
				{	//记录cloud2与cloud1的对应点
					//temp_con[cloud2->grid[j]][temp_idx[cloud2->grid[j]]] = i;
					temp_con[cloud2->grid[j]].push_back(i);
					//temp_idx[cloud2->grid[j]]++;
					//std::cout << temp_idx[cloud2->grid[j]] << std::endl;
					idx[i] = 1;
				}
				else if(n.z<0)
				{
					cloud_last->vertices.push_back(cloud1->vertices[i]);
					cloud_last->normals.push_back(cloud1->normals[i]);
				}
			}

		}
		cout << "fusion reproject time:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		t1 = clock();
		t2 = clock();
		static trimesh::TriMesh cloud_tmp;
		static vector<int> u_tmp;
		static vector<int> v_tmp;
		cloud_tmp.vertices = cloud1->vertices;
		cloud_tmp.normals = cloud1->normals;
		u_tmp.resize(nv1);
		v_tmp.resize(nv1);
		cout << "vertices size():" << cloud1->vertices.size() << endl;
		cout << "data copy time:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
#pragma omp parallel for
		for (int i = 0; i < cloud_tmp.vertices.size(); i++)
		{
			point p = xf12 * cloud1->vertices[i];
			vec n = nxf12 * cloud1->normals[i];
			cloud_tmp.vertices[i] = p;
			cloud_tmp.normals[i] = n;

			u_tmp[i] = (int)roundf((fx * p[0] + cx * p[2]) / p[2]);
			v_tmp[i] = (int)roundf((fy * p[1] + cy * p[2]) / p[2]);
		}
		cout << "fusion reproject parallel test time:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		std::vector<int> idx_tmp(nv1, 0); 
		std::vector<std::vector<int>>  temp_con1(nv2);
		for (int i = 0; i < nv1; i++)
		{
			point p = cloud_tmp.vertices[i];
			vec n = cloud_tmp.normals[i];

			// 当前帧看不到的点
			if (n.z >= 0)
				continue;

			// 加包围盒约束
			if (p.x<cloud2->bbox.min.x || p.x>cloud2->bbox.max.x
				|| p.y<cloud2->bbox.min.y || p.y>cloud2->bbox.max.y
				|| p.z<cloud2->bbox.min.z || p.z>cloud2->bbox.max.z)
				continue;

			int u = u_tmp[i];
			int v = v_tmp[i];
			// Check corresponding vertex
			if ((u >= 1 && u < cloud2->grid_width - 1) && (v >= 1 && v < cloud2->grid_height - 1)) //确保像素位置在cloud2的图像平面中
			{
				int j = u + v * cloud2->grid_width;	//计算cloud2图像平面位置的对应点位置
				if (cloud2->grid[j] >= 0 && n.dot(cloud2->normals[cloud2->grid[j]]) > 0.90&&dist2(p, cloud2->vertices[cloud2->grid[j]])<1.0)
				{	//记录cloud2与cloud1的对应点
					//temp_con[cloud2->grid[j]][temp_idx[cloud2->grid[j]]] = i;
					temp_con1[cloud2->grid[j]].push_back(i);
					//temp_idx[cloud2->grid[j]]++;
					//std::cout << temp_idx[cloud2->grid[j]] << std::endl;
					idx_tmp[i] = 1;
				}
			}
		}
		cout << "new fusion reproject time:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		t2 = clock();
		std::vector<point> vertices_g2(nv2);
		std::vector<vec> normals_g2(nv2);
		//将cloud2转换到全局坐标系下
#pragma omp parallel for
		for (int i = 0; i < nv2; i++)
		{
			vertices_g2[i] = xf2*cloud2->vertices[i];
			normals_g2[i] = nxf2*cloud2->normals[i];
		}
		
#pragma omp parallel for
		for (int k = 0; k < nv2; k++)
		{
			//将cloud2转化到cloud1的相机坐标系下
			vertices_g2[k] = ixf1*vertices_g2[k];
			normals_g2[k] = nixf1*normals_g2[k];
			int temp_num = temp_con1[k].size();
			if (/*temp_idx[k]*/temp_num > 0)
			{
				//将对应的cloud1对应点相加求平均作为cloud1改点的新值
				for (int i = 0; i < /*temp_idx[k]*/temp_num; i++)
				{
					vertices_g2[k] += cloud1->vertices[temp_con1[k][i]];
					//normals_g2[k] += cloud1->normals[temp_con[k][i]];
				}
				vertices_g2[k] = (vertices_g2[k] / (/*temp_idx[k]*/temp_num + 1));
				normals_g2[k] = normalized(normals_g2[k]);
			}
			else {
				//newPnt.push_back(vertices_g2[k]);
				//newNrm.push_back(normals_g2[k]);
			}
		}
		cout << "cloud now fusion parallel test time:" << double(clock() - t2) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		int num_erase = std::accumulate(idx_tmp.begin(), idx_tmp.end(), 0);	//cloud1中找到对应点的个数
		std::vector<point> vertices_1(nv1 - num_erase);
		std::vector<vec> normals_1(nv1 - num_erase);

		int k = 0;
		//去除cloud1中的对应点，将剩余的点记录在vertices_1中
		for (int i = 0; i < nv1; i++)
		{
			if (idx_tmp[i] != 1 && k < vertices_1.size())
			{
				vertices_1[k] = cloud1->vertices[i];
				normals_1[k] = cloud1->normals[i];
				k++;
			}
		}

		cloud1->vertices.clear();
		cloud1->normals.clear();
		vertices_1.swap(cloud1->vertices);
		normals_1.swap(cloud1->normals);
		for (int i = 0; i < nv2; i++)
		{
			if (normals_g2[i].x == 0 && normals_g2[i].y == 0 && normals_g2[i].z == 0)
				continue;
			cloud1->vertices.push_back(vertices_g2[i]);
			cloud1->normals.push_back(normals_g2[i]);
		}

		cout << "fusion time:" << double(clock() - t1) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

		// get cloud_last
		t1 = clock();
		for (int i = 0; i < cloud1->vertices.size(); i++)
		{
			point p = xf12 * cloud1->vertices[i];
			vec n = nxf12 * cloud1->normals[i];

			// 当前帧看不到的点
			if (n.z >= 0)
				continue;

			// Project source vertex into the destination's image plane.
			//将cloud1投影到图像平面，计算图像平面的像素位置
			int u = (int)roundf((fx * p[0] + cx * p[2]) / p[2]);
			int v = (int)roundf((fy * p[1] + cy * p[2]) / p[2]);
			// Check corresponding vertex
			if ((u >= 1 && u < cloud2->grid_width - 1) && (v >= 1 && v < cloud2->grid_height - 1)) //确保像素位置在cloud2的图像平面中
			{
				cloud_last->vertices.push_back(cloud1->vertices[i]);
				cloud_last->normals.push_back(cloud1->normals[i]);
			}
		}
		cout << "get cloud last time:" << double(clock() - t1) * 1000 / CLOCKS_PER_SEC << "ms" << endl;

	}

	void point_align_fusion_with_color(TriMesh *cloud1, TriMesh *cloud2,
		const float fx, const float fy, const float cx,
		const float cy, const xform &xf1, const xform &xf2)
	{
		clock_t t1 = clock();
		xform nxf1 = norm_xf(xf1);
		xform nxf2 = norm_xf(xf2);
		xform xf12 = inv(xf2) * xf1;
		xform nxf12 = norm_xf(xf12);
		xform ixf1 = inv(xf1);
		xform nixf1 = norm_xf(ixf1);

		const int nv2 = cloud2->vertices.size();

		std::vector<point> vertices_g2(nv2);
		std::vector<vec> normals_g2(nv2);
		std::vector<Color> colors_g2(nv2);;
#pragma omp parallel for
		//将cloud2转换到全局坐标系下
		for (int i = 0; i < nv2; i++)
		{
			vertices_g2[i] = xf2*cloud2->vertices[i];
			normals_g2[i] = nxf2*cloud2->normals[i];
			colors_g2[i] = cloud2->colors[i];
		}

		const int nv1 = cloud1->vertices.size();
		std::vector<int> idx(nv1, 0); //用于记录cloud1中和cloud2对应的点

									  //std::vector<std::vector<int>>  temp_con(nv2, std::vector<int>(50,0)); //用于记录cloud2中每个点和cloud1对应的点
		std::vector<std::vector<int>>  temp_con(nv2); //用于记录cloud2中每个点和cloud1对应的点
													  //std::vector<int> temp_idx(nv2, 0); //用于记录cloud2中每个点和cloud1对应的点的个数
													  // "F:\\扫描软件\\code\\EsScanSoft\\EsScanSoft\\data\\capture\\";// donglin
													  // string calib_dir = "F:\\扫描软件\\code\\EsScanSoft\\EsScanSoft\\data\\capture\\"; //"D:\\prejects\\08_s2_software\\x64\\Release\\Capture\\";//"F:\\project\\S2\\S2_Software_Project_V1.0\\EsScanSoft\\capture\\";//
													  // 读取标定数据
		for (int i = 0; i < nv1; i++)
		{
			//将cloud1转化到cloud2相机坐标系下
			point p = xf12 * cloud1->vertices[i];
			vec n = nxf12 * cloud1->normals[i];

			// Project source vertex into the destination's image plane.
			//将cloud1投影到图像平面，计算图像平面的像素位置
			int u = (int)roundf((fx * p[0] + cx * p[2]) / p[2]);
			int v = (int)roundf((fy * p[1] + cy * p[2]) / p[2]);

			// Check corresponding vertex
			if ((u >= 1 && u < cloud2->grid_width - 1) && (v >= 1 && v < cloud2->grid_height - 1)) //确保像素位置在cloud2的图像平面中
			{
				int j = u + v * cloud2->grid_width;	//计算cloud2图像平面位置的对应点位置
				if (cloud2->grid[j] >= 0 && n.dot(cloud2->normals[cloud2->grid[j]]) > 0.90&&dist2(p, cloud2->vertices[cloud2->grid[j]])<1.0)
				{	//记录cloud2与cloud1的对应点
					//temp_con[cloud2->grid[j]][temp_idx[cloud2->grid[j]]] = i;
					temp_con[cloud2->grid[j]].push_back(i);
					//temp_idx[cloud2->grid[j]]++;
					//std::cout << temp_idx[cloud2->grid[j]] << std::endl;
					idx[i] = 1;
				}
			}
		}

		int num_erase = std::accumulate(idx.begin(), idx.end(), 0);	//cloud1中找到对应点的个数
#pragma omp parallel for
		for (int k = 0; k < nv2; k++)
		{
			//将cloud2转化到cloud1的相机坐标系下
			vertices_g2[k] = ixf1*vertices_g2[k];
			normals_g2[k] = nixf1*normals_g2[k];
			int temp_num = temp_con[k].size();
			if (/*temp_idx[k]*/temp_num > 0)
			{
				//将对应的cloud1对应点相加求平均作为cloud1改点的新值
				for (int i = 0; i < /*temp_idx[k]*/temp_num; i++)
				{
					vertices_g2[k] += cloud1->vertices[temp_con[k][i]];
					//normals_g2[k] += cloud1->normals[temp_con[k][i]];
					colors_g2[k] += cloud1->colors[temp_con[k][i]];
				}
				vertices_g2[k] = (vertices_g2[k] / (/*temp_idx[k]*/temp_num + 1));
				colors_g2[k] = (colors_g2[k] / (temp_num + 1));
				normals_g2[k] = normalized(normals_g2[k]);
			}
		}

		std::vector<point> vertices_1(nv1 - num_erase);
		std::vector<vec> normals_1(nv1 - num_erase);
		std::vector<Color> colors_1(nv1 - num_erase);

		int k = 0;
		//去除cloud1中的对应点，将剩余的点记录在vertices_1中
		for (int i = 0; i < nv1; i++)
		{
			if (idx[i] != 1 && k < vertices_1.size())
			{
				vertices_1[k] = cloud1->vertices[i];
				normals_1[k] = cloud1->normals[i];
				colors_1[k] = cloud1->colors[i];
				k++;
			}
		}

		cloud1->vertices.clear();
		cloud1->normals.clear();
		cloud1->colors.clear();
		vertices_1.swap(cloud1->vertices);
		normals_1.swap(cloud1->normals);
		colors_1.swap(cloud1->colors);
		for (int i = 0; i < nv2; i++)
		{
			if (normals_g2[i].x == 0 && normals_g2[i].y == 0 && normals_g2[i].z == 0)
				continue;
			cloud1->vertices.push_back(vertices_g2[i]);
			cloud1->normals.push_back(normals_g2[i]);
			cloud1->colors.push_back(colors_g2[i]);
		}
		cout << "cloud all normal size:" << cloud1->normals.size() << endl;
		cout << "cloud now normal size:" << cloud2->normals.size() << endl;

		cout << "color fusion time:" << double(clock() - t1) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
	}


	// Determinant of a 3x3 matrix
	static float det(const float(&A)[3][3])
	{
		return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) +
			A[0][1] * (A[1][2] * A[2][0] - A[1][0] * A[2][2]) +
			A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
	}


	// Do rigid-body point-to-point alignment.  (This is done in the early stages
	// of registration to improve stability.)
	static void align_pt2pt(const vector<PtPair> &pairs,
		const point &centroid1, const point &centroid2,
		xform &alignxf)
	{
		size_t n = pairs.size();

		float A[3][3] = { { 0 } };
		for (size_t i = 0; i < n; i++) {
			vec v1 = pairs[i].p1 - centroid1;
			vec v2 = pairs[i].p2 - centroid2;
			for (int j = 0; j < 3; j++)
				for (int k = 0; k < 3; k++)
					A[j][k] += v1[j] * v2[k];
		}
		float s[3], V[3][3];
		svd<float, 3, 3>(A, s, V);
		if ((det(A) < 0.0f) ^ (det(V) < 0.0f)) {
			V[2][0] = -V[2][0];
			V[2][1] = -V[2][1];
			V[2][2] = -V[2][2];
		}
		alignxf = xform::trans(centroid1) *
			xform::fromarray(A) * transp(xform::fromarray(V)) *
			xform::trans(-centroid2);
	}


	// Do symmetric point-to-plane alignment, returning alignxf
	// as well as eigenvectors and inverse eigenvalues
	static void align_pt2pl(const vector<PtPair> &pairs, float scale,
		const point &centroid1, const point &centroid2,
		float(&evec)[6][6], float(&einv)[6],
		xform &alignxf)
	{
		int n = (int)pairs.size();

		float b[6] = { 0 };
		for (int i = 0; i < n; i++) {
			vec p1 = pairs[i].p1 - centroid1;
			//cout << "p1.x"<<p1.x << endl;
			vec p2 = pairs[i].p2 - centroid2;
			//cout << "p1:" << p1.x << endl;
			const vec &n1 = pairs[i].n1;
			const vec &n2 = pairs[i].n2;
			vec nn = n1 + n2;
			
			vec c = scale * (p1 CROSS n2 + p2 CROSS n1);			
			vec p12 = p1 - p2;
			
			float d = scale * (p12 DOT nn);

			float x[6] = { c[0], c[1], c[2], nn[0], nn[1], nn[2] };
			for (int j = 0; j < 6; j++) {
				b[j] += d * x[j];
				for (int k = j; k < 6; k++)
					evec[j][k] += x[j] * x[k];
			}
			//cout << "pairs[i].p1:" << pairs[i].p1.x << endl;
			//cout << "pairs[i].p2:" << pairs[i].p2.x << endl;
			//cout << "centroid1:" << centroid1 << endl;
			//cout << "centroid1:" << centroid2 << endl;
			//cout << "p1.x:" << p1.x << endl;
			//cout << "p2.x:" << p2.x << endl;
			//cout << "evec[0][2]:" << p12.x << endl;
			// Regularization for rotational component - point to point
			float reg = REGULARIZATION * sqr(scale);
			evec[0][0] += reg * (sqr(p2[1]) + sqr(p2[2]));			
			evec[0][1] -= reg * p2[0] * p2[1];
			evec[0][2] -= reg * p2[0] * p2[2];
			evec[1][1] += reg * (sqr(p2[0]) + sqr(p2[2]));
			evec[1][2] -= reg * p2[1] * p2[2];
			evec[2][2] += reg * (sqr(p2[0]) + sqr(p2[1]));
			
			/*vec c2 = p2 CROSS p12;
			x[0] += reg * c2[0];
			x[1] += reg * c2[1];
			x[2] += reg * c2[2];*/
		}
		
		// Regularization for translational component
		evec[3][3] += REGULARIZATION * n;
		evec[4][4] += REGULARIZATION * n;
		evec[5][5] += REGULARIZATION * n;
		/*cout << "einv0[1][2]:" << evec[0][0] << " " << evec[0][1] << " " << evec[0][2]<< endl;
		cout << "einv0[3][5]:" << evec[0][3] << " " << evec[0][4] << " " << evec[0][5] << endl;
		cout << "einv1[0][5]:" << evec[1][1] << " " << evec[1][2] << " " << evec[1][3] << endl;
		cout << "einv[0][5]:" << evec[1][4] << " " << evec[1][5] << " " << evec[2][2] << endl;
		cout << "einv[0][5]:" << evec[2][3] << " " << evec[2][4] << " " << evec[2][5] << endl;*/
		// Make matrix symmetric
		for (int j = 0; j < 6; j++)
			for (int k = 0; k < j; k++)
				evec[j][k] = evec[k][j];
		//cout << "einv[0][3],,,,,,,,,:" << evec[0][4] << endl;
		// Eigen-decomposition and inverse
		float eval[6];
		eigdc<float, 6>(evec, eval);
		for (int i = 0; i < 6; i++)
			einv[i] = 1.0f / eval[i];
		//cout << "einv[1][1]:" << evec[0][4] << endl;
	/*	cout << "einv[0]=" << einv[0] << "einv[1]= " << einv[1] << "einv[1]= " << einv[2] << endl;
		cout << "einv[3]:" << einv[3] << "einv[4]= " << einv[4] << "einv[5]= " << einv[5] << endl;
		cout << "einv0[1][2]:" << evec[0][0] << " " << evec[0][1] << " " << evec[0][2] << endl;
		cout << "einv0[3][5]:" << evec[0][3] << " " << evec[0][4] << " " << evec[0][5] << endl;
		cout << "einv1[0][5]:" << evec[1][1] << " " << evec[1][2] << " " << evec[1][3] << endl;
		cout << "einv[0][5]:" << evec[1][4] << " " << evec[1][5] << " " << evec[2][2] << endl;
		cout << "einv[0][5]:" << evec[2][3] << " " << evec[2][4] << " " << evec[2][5] << endl;*/
		// Solve system
		eigmult<float, 6>(evec, einv, b);
		//cout << "einv......:" << evec<< endl;
		// Extract rotation and translation
		//cout << b[0] << endl;
		//cout << b[1] << endl; cout << b[2] << endl; cout << b[3] << endl; cout << b[4] << endl; cout << b[5] << endl;
		vec rot(b[0], b[1], b[2]), trans(b[3], b[4], b[5]);
		float rotangle = atan(len(rot));
		//cout << "rotangle:" << rotangle << endl; 
		float cosangle = cos(rotangle);
		//cout << "cosangle:" << cosangle << endl;
		trans *= cosangle / ((1.0f + cosangle) * scale);
		//cout << "cosangle:" << trans << endl;
		alignxf = xform::trans(trans + centroid1) *
			xform::rot(rotangle, rot) *
			xform::trans(trans - centroid2);
		//cout << "R:"<<endl << xform::rot(rotangle, rot) << endl;
		//cout << "alignxf:"<<endl << alignxf << endl;
	}


	// Do symmetric point-to-plane translation-only alignment
	static void align_pt2pl_trans(const vector<PtPair> &pairs,
		const point &centroid1, const point &centroid2,
		xform &alignxf)
	{
		size_t n = pairs.size();

		float evec[3][3] = { { 0 } }, einv[3] = { 0 };
		vec b;
		for (size_t i = 0; i < n; i++) {
			vec p1 = pairs[i].p1 - centroid1;
			vec p2 = pairs[i].p2 - centroid2;
			const vec &n1 = pairs[i].n1;
			const vec &n2 = pairs[i].n2;
			vec nn = n1 + n2;
			float d = (p1 - p2) DOT nn;

			for (int j = 0; j < 3; j++) {
				b[j] += d * nn[j];
				for (int k = 0; k < 3; k++)
					evec[j][k] += nn[j] * nn[k];
			}
		}

		// Regularization
		evec[0][0] += REGULARIZATION * n;
		evec[1][1] += REGULARIZATION * n;
		evec[2][2] += REGULARIZATION * n;

		// Eigen-decomposition and inverse
		vec eval;
		eigdc<float, 3>(evec, eval);
		for (int i = 0; i < 3; i++)
			einv[i] = 1.0f / eval[i];

		// Solve system
		eigmult<float, 3>(evec, einv, b);
		b += centroid1 - centroid2;
		alignxf = xform::trans(b);
	}


	// Compute isotropic or anisotropic scale.  Assumes alignxf already contains
	// a rigid-body transformation to be applied to pairs[i].p2
	static void align_scale(const vector<PtPair> &pairs, xform &alignxf,
		const point &centroid1, const point &centroid2,
		bool do_affine)
	{
		size_t n = pairs.size();

		point centroid = 0.5f * (centroid1 + alignxf * centroid2);

		// Compute covariance matrices
		float cov1[3][3] = { { 0 } };
		float cov2[3][3] = { { 0 } };
		for (size_t i = 0; i < n; i++) {
			point p1 = pairs[i].p1 - centroid;
			point p2 = alignxf * pairs[i].p2 - centroid;
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					cov1[j][k] += p1[j] * p1[k];
					cov2[j][k] += p2[j] * p2[k];
				}
			}
		}

		// Compute eigenstuff of cov
		vec eval1, eval2;
		eigdc<float, 3>(cov1, eval1);
		eigdc<float, 3>(cov2, eval2);

		if (!do_affine) {
			// Just uniform scale
			alignxf = xform::trans(centroid) *
				xform::scale(sqrt(eval1.sum() / eval2.sum())) *
				xform::trans(-centroid) *
				alignxf;
			return;
		}

		// Compute sqrt of covariance
		float csqrt1[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
		float icsqrt2[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
		for (int i = 0; i < 3; i++) {
			eigmult<float, 3>(cov1, sqrt(eval1), csqrt1[i]);
			eigmult<float, 3>(cov2, sqrt(1.0f / eval2), icsqrt2[i]);
		}

		alignxf = xform::trans(centroid) *
			xform::fromarray(csqrt1) *
			xform::fromarray(icsqrt2) *
			xform::trans(-centroid) *
			alignxf;
	}


	// Compute point-to-point or point-to-plane squared distances
	static void compute_dist2(const vector<PtPair> &pairs,
		vector<float> &distances2, ICP_iter_type iter_type)
	{
		int np = (int)pairs.size();
		if (np == 0)
		{
			cout << " no correspondent point! " << endl;
			return;
		}
		distances2.clear();
		distances2.resize(np);
		if (iter_type == ICP_POINT_TO_POINT) {
			//#pragma omp parallel for 
			for (int i = 0; i < np; i++)
				distances2[i] = dist2(pairs[i].p1, pairs[i].p2);
		}
		else /* ICP_POINT_TO_PLANE */ {
			float p2pl_mult = 0.5f / (1.0f + REGULARIZATION);
			float p2pt_mult = REGULARIZATION / (1.0f + REGULARIZATION);
			//#pragma omp parallel for 
			for (int i = 0; i < np; i++) {
				vec v = pairs[i].p1 - pairs[i].p2;
				distances2[i] = p2pl_mult * (sqr(v DOT pairs[i].n1) +
					sqr(v DOT pairs[i].n2));
				distances2[i] += p2pt_mult * len2(v);
			}
		}
	}


	// Find the median of a list of numbers - makes a copy of vals (since it's
	// passed by value, not by reference) so that we can modify it
	static float median(vector<float> vals)
	{
		size_t n = vals.size();
		if (!n)
			return 0.0f;

		size_t mid = n / 2;
		nth_element(vals.begin(), vals.begin() + mid, vals.end());
		return vals[mid];
	}


	// Find the mean of a list of numbers
	static float mean(const vector<float> &vals)
	{
		size_t n = vals.size();
		if (!n)
			return 0.0f;

		float sum = 0.0f;
		for (size_t i = 0; i < n; i++)
			sum += vals[i];

		return sum / n;
	}


	// Do one iteration of ICP
	static float ICP_iter(TriMesh *cloud1, TriMesh *cloud2,
		const xform &xf1, xform &xf2, std::vector<point> pts, const float fx,
		const float fy, const float cx, const float cy,
		float dist_thresh_mult, float normdot_thresh,
		float &maxdist, int verbose, TNT::Matrix<float>& e_left,
		ICP_iter_type iter_type)
	{
		const size_t nv1 = cloud1->vertices.size(), nv2 = cloud2->vertices.size();
		// Compute pairs
		timestamp t1 = now();
		if (verbose > 1) {
			dprintf("Matching with maxdist = %g\n", maxdist);
		}

		vector<PtPair> pairs;
		select_and_match(cloud1, cloud2, pts, fx, fy, cx, cy, xf1, xf2, pairs, e_left);
		//cout << pairs[1].p1 << "      " << pairs[1].n1 << "      " << pairs[1].p2 << "      " << pairs[1].n2 << "      " << endl;
		timestamp t2 = now();
		int npairs = (int)pairs.size();
		if (verbose > 1) {
			cout << "Generated " << (unsigned long)npairs << "pairs in" <<
				(t2 - t1) * 1000.0f << "msec." << endl;
		}
		if (npairs == 0)
		{
			return -1.0f;
		}

		//std::cout << "npairs != 0" << std::endl;
		// Compute median point-to-point or point-to-plane distance.
		vector<float> distances2;
		compute_dist2(pairs, distances2, iter_type);
		float median_dist = sqrt(median(distances2));
		
		// Now compute threshold for rejection, as a multiple of sigma
		// (estimated robustly based on the median).
		float sigma = MEDIAN_TO_SIGMA * median_dist;
		float dist_thresh = dist_thresh_mult * sigma;
		float dist_thresh2 = sqr(dist_thresh);

		// We also compute the new maxdist, but that always has to be
		// based on point-to-point distances (since that's how the KDtree
		// interprets maxdist).

		/*if (iter_type == ICP_POINT_TO_POINT) {
			maxdist = dist_thresh_mult * sigma;
		}
		else {
			vector<float> distances2_pt2pt;
			compute_dist2(pairs, distances2_pt2pt, ICP_POINT_TO_POINT);
			float sigma_pt2pt = MEDIAN_TO_SIGMA * sqrt(median(distances2_pt2pt));
			maxdist = dist_thresh_mult * sigma_pt2pt;
		}*/

		// Reject
		if (verbose > 1)
			dprintf("Rejecting pairs with dist > %g or angle > %.1f\n",
			dist_thresh, degrees(acos(normdot_thresh)));
		float err = 0.0f;
		size_t next = 0;
		
		for (int i = 0; i < npairs; i++) {
			if (distances2[i] > dist_thresh2)
				continue;
			if ((pairs[i].n1 DOT pairs[i].n2) < normdot_thresh)
				continue;
			pairs[next++] = pairs[i];
			err += distances2[i];
		}
		
		pairs.erase(pairs.begin() + next, pairs.end());
		timestamp t3 = now();

		npairs = pairs.size();
		
		if (npairs < MIN_PAIRS) {
			if (verbose)
				dprintf("Too few point pairs.\n");
			return -1.0f;
		}

		err = sqrt(err / npairs);
		
		const char *dist_type = (iter_type == ICP_POINT_TO_POINT) ?
			"point-to-point" : "point-to-plane";
		if (verbose > 1)
			dprintf("RMS %s error before alignment = %g\n", dist_type, err);

		// Compute centroids and scale
		point centroid1, centroid2;
		for (int i = 0; i < npairs; i++) {
			centroid1 += pairs[i].p1;
			centroid2 += pairs[i].p2;
			//cout << "pairs["<<i<<"].p1" << pairs[i].p1 << endl;
			//cout << "pairs[" << i << "].p1" << pairs[i].p2 << endl;
		}
		centroid1 /= npairs;
		centroid2 /= npairs;	
		float scale = 0.0f;
#pragma omp parallel for reduction(+:scale)
		for (int i = 0; i < npairs; i++) {
			scale += dist2(pairs[i].p1, centroid1);
			scale += dist2(pairs[i].p2, centroid2);
		}
		scale = sqrt(scale / (2 * npairs));
		scale = 1.0f / scale;
		
		// Do the minimization
		float evec[6][6] = { { 0 } }, einv[6] = { 0 };
		xform alignxf;

		// First do rigid-body alignment
		if (iter_type == ICP_POINT_TO_POINT) {
			align_pt2pt(pairs, centroid1, centroid2, alignxf);
		}
		else {
			align_pt2pl(pairs, scale, centroid1, centroid2,
				evec, einv, alignxf);
		}
		maxdist = scale;
		//yyRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR
		// Apply transform, and find distance after alignment
		xf2 = alignxf * xf2;
		orthogonalize(xf2);
		//cout << xf2 << endl;
		xform nalignxf = norm_xf(alignxf);
#pragma omp parallel for
		for (int i = 0; i < npairs; i++) {
			pairs[i].p2 = alignxf * pairs[i].p2;
			pairs[i].n2 = nalignxf * pairs[i].n2;
		}
		compute_dist2(pairs, distances2, iter_type);
		err = sqrt(mean(distances2));

		timestamp t4 = now();
		if (verbose > 1) {
			cout << "Computed xform in " <<
				(t4 - t3) * 1000.0f << "msec." << endl;
			dprintf("RMS %s error after alignment = %g\n\n", dist_type, err);

		}

		return err;
	}


	// Create a CDF for simple weighted sampling
	static void make_uniform_cdfs(
		const vector<float> &weights1, vector<float> &sampcdf1,
		const vector<float> &weights2, vector<float> &sampcdf2)
	{
		const size_t nv1 = weights1.size(), nv2 = weights2.size();
		sampcdf1.resize(nv1);
		sampcdf2.resize(nv2);

		double sum_sampcdf1 = 0, sum_sampcdf2 = 0;
#pragma omp parallel
		{
#pragma omp for nowait reduction(+ : sum_sampcdf1)
			for (__int64/*size_t*/ i = 0; i < nv1; i++)
				sum_sampcdf1 += (sampcdf1[i] = weights1[i]);
#pragma omp for reduction(+ : sum_sampcdf2)
			for (__int64/*size_t*/ i = 0; i < nv2; i++)
				sum_sampcdf2 += (sampcdf2[i] = weights2[i]);
		}

		float cdf_scale1 = 1 / sum_sampcdf1;
		sampcdf1[0] *= cdf_scale1;
		for (size_t i = 1; i < nv1 - 1; i++)
			sampcdf1[i] = cdf_scale1 * sampcdf1[i] + sampcdf1[i - 1];
		sampcdf1[nv1 - 1] = 1.0f;

		float cdf_scale2 = 1 / sum_sampcdf2;
		sampcdf2[0] *= cdf_scale2;
		for (size_t i = 1; i < nv2 - 1; i++)
			sampcdf2[i] = cdf_scale2 * sampcdf2[i] + sampcdf2[i - 1];
		sampcdf2[nv2 - 1] = 1.0f;
	}


	// Do ICP.  Aligns mesh mesh2 to mesh1, updating xf2 with the new transform.
	// Returns alignment error, or -1 on failure
	float ICP(TriMesh *cloud1, TriMesh *cloud2,
		const xform &xf1, xform &xf2, const float fx,
		const float fy, const float cx, const float cy,
		float maxdist /* = 0.0f */, int verbose /* = 0 */, TNT::Matrix<float>& e_left)
	{
		timestamp t_start = now();
		// Precompute normals, connectivity (used to determine boundaries),
		// and grids (used for fast overlap computation)
		Grid *g1 = NULL, *g2 = NULL;
		for (int i = 0; i < 2; i++) {
			
			TriMesh *cloud = (i == 0) ? cloud1 : cloud2;
			Grid * &g = (i == 0) ? g1 : g2;
			cloud->need_normals();
			g = new Grid(cloud->vertices);
		}
		
		std::vector<point> pts = g2->point_bbox();
		cloud2->bbox.min.x = pts[0].x;  cloud2->bbox.min.y = pts[0].y; cloud2->bbox.min.z = pts[0].z;
		cloud2->bbox.max.x = pts[3].x;  cloud2->bbox.max.y = pts[3].y; cloud2->bbox.max.z = pts[3].z;
		std::vector<point> pts2 = g1->point_bbox();
		cloud1->bbox.min.x = pts2[0].x;  cloud1->bbox.min.y = pts2[0].y; cloud1->bbox.min.z = pts2[0].z;
		cloud1->bbox.max.x = pts2[3].x;  cloud1->bbox.max.y = pts2[3].y; cloud1->bbox.max.z = pts2[3].z;
		// Initial maxdist, thresholds
		if (maxdist <= 0.0f)
			maxdist = min(g1->bbox_size(), g2->bbox_size());
		delete g1;
		g1 = nullptr;
		delete g2;
		g2 = nullptr;
		float dist_thresh_mult = DIST_THRESH_MULT_START;
		float normdot_thresh = NORMDOT_THRESH_START;
		timestamp t_initial_iters = now();

		if (verbose > 1) {
			cout << "Time for preprocessing: " << (t_initial_iters - t_start) * 1000.0f <<
				"msec." << endl;
		}

		// First, do a few point-to-point iterations for stability in case the
		// initial misalignment is big.

		float err;
		for (int iter = 0; iter < INITIAL_ITERS; iter++) {
			err = ICP_iter(cloud1, cloud2, xf1, xf2, pts, fx, fy,
				cx, cy, dist_thresh_mult,
				normdot_thresh, maxdist, verbose, e_left,
				ICP_POINT_TO_POINT);
			//std::cout << "error:" << err << std::endl;
			
			if (err < 0) {
				return err;
			}
		}

		timestamp t_main_iters = now();
		if (verbose > 1) {
			cout << "Time for initial iterations: " << (t_main_iters - t_initial_iters) * 1000.0f <<
				"msec." << endl;
		}
		//float err;
		//std::ofstream info("./GPUxf.txt");
		// Now the real (point-to-plane) iterations
		float min_err = 0.0f;
		int iter_of_min_err = -1;
		int iter;
		//info << "xf2Orignal:" << xf2 << endl;
		for (iter = 0; iter < MAX_ITERS; iter++) {
			// Update thresholds
			dist_thresh_mult = mix(dist_thresh_mult, DIST_THRESH_MULT_FINAL,
				THRESH_RATE_OF_CHANGE);
			normdot_thresh = mix(normdot_thresh, NORMDOT_THRESH_FINAL,
				THRESH_RATE_OF_CHANGE);

			// Do an iteration
			err = ICP_iter(cloud1, cloud2, xf1, xf2,pts, fx, fy, 
				cx, cy, dist_thresh_mult,
				normdot_thresh, maxdist, verbose, e_left,
				ICP_POINT_TO_PLANE);
			//info << "iter:" << iter << "sacle:"<<maxdist<<endl;
			//info << "xf2:" <<xf2<< endl;
			//std::cout << "error:" << err << std::endl;
			if (err < 0) {
				return err;
			}

			if (err < min_err || iter_of_min_err < 0) {
				min_err = err;
				iter_of_min_err = iter;
			}

			// Stop if we've gone at least TERMINATION_ITER_THRESH
			// iterations without seeing a new minimum error
			if (iter - iter_of_min_err >= TERMINATION_ITER_THRESH &&
				iter_of_min_err >= 0) {
				iter++; // Get #-of-iters printf correct
				break;
			}

			if (err <= 0.02)
				break;

		}
		//cout << "normal icp iters:" << iter << endl;

		for (iter = 0; iter < FINAL_ITERS; iter++) {
			err = ICP_iter(cloud1, cloud2, xf1, xf2, pts, fx, fy,
				cx, cy, DIST_THRESH_MULT_FINAL,
				NORMDOT_THRESH_FINAL, maxdist, verbose, e_left,
				ICP_POINT_TO_PLANE);
			//std::cout << "error:" << err << std::endl;
			if (err < 0) {
				return err;
			}
		}
		if (verbose == 1) {
			dprintf("ICP error = %g\n", err);
			cout << "Time for ICP: " << (now() - t_start) * 1000.0f <<
				"msec." << endl;
		}
		else if (verbose > 1) {
			// err already printed out in ICP_iter
			cout << "Time for ICP: " << (now() - t_start) * 1000.0f <<
				"msec." << endl;
		}
		//dprintf("Time for ICP: %.3f msec.\n\n",
		//	(now() - t_start) * 1000.0f);
		return err;
	}

} // namespace trimesh
