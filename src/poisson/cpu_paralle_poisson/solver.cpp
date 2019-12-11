#include "solver.h"

#include "node_key.h"
#include "dot_caculator.h"

#include "octree.h"

#include <iostream>

#include <Eigen\SparseCholesky>
#include <Eigen\Sparse>

#include "saver.h"
#include <stack>

#include "extractor.h"
#include <fstream>
Solver::Solver(unsigned depth)
	:m_lookup_table_2(0), m_lookup_table_1(0), m_lookup_table_0(0)
{
	BuildTable(depth);
}

Solver::~Solver()
{
	delete[] m_lookup_table_0;
	delete[] m_lookup_table_1;
	delete[] m_lookup_table_2;
}

void Solver::BuildTable(unsigned depth)
{
	unsigned n = (1 << (depth + 1)) - 1;
	m_lookup_table_0 = new double[n*n];
	m_lookup_table_1 = new double[n*n];
	m_lookup_table_2 = new double[n*n];
	memset(m_lookup_table_0, 0, sizeof(double) * n * n);
	memset(m_lookup_table_1, 0, sizeof(double) * n * n);
	memset(m_lookup_table_2, 0, sizeof(double) * n * n);

	DotCaculator caculator(depth);
	double t1 = caculator.start();
	double t2 = caculator.end();
	
	for (unsigned i = 0; i < n; i++)
	{
		double c1, c2, w1, w2;
		index2centerandwidth(i, c1, w1);
		double start1 = t1*w1 + c1;
		double end1 = t2*w1 + c1;
		for (unsigned j = 0; j <= i; j++)
		{
			index2centerandwidth(j, c2, w2);
			unsigned idx1 = i + n * j;
			unsigned idx2 = j + n * i;

			double start = t1 * w2 + c2;
			double end = t2 * w2 + c2;

			if (start < start1) { start = start1; }
			if (end > end1) { end = end1; }
			if (start >= end) { continue; }

			double dot = caculator.dotProduct(c1, w1, c2, w2);
			if (fabs(dot) < 1e-15) { continue; }
			m_lookup_table_0[idx1] = m_lookup_table_0[idx2] = dot;
			
			m_lookup_table_1[idx1] = caculator.dDotProduct(c1, w1, c2, w2);
			m_lookup_table_1[idx2] = -m_lookup_table_1[idx1];
			
			m_lookup_table_2[idx1] = m_lookup_table_2[idx2] = caculator.d2DotProduct(c1, w1, c2, w2);
		}
	}

	//save_table(n, m_lookup_table_0, "E:\\Sim\\gpu_poisson\\dot_table_G");
	//save_table(n, m_lookup_table_1, "E:\\Sim\\gpu_poisson\\d_dot_table_G");
	//save_table(n, m_lookup_table_2, "E:\\Sim\\gpu_poisson\\d2_dot_table_G");
}

void Solver::Solve(Octree& octree)
{
	unsigned max_depth = octree.m_depth;
	DotCaculator caculator(max_depth);

	unsigned res = (1 << (max_depth + 1)) - 1;
	std::vector<PPolynomial<2>> funcs(res);
	for (unsigned i = 0; i < res; ++i)
	{
		double c, w;
		index2centerandwidth(i, c, w);
		funcs[i] = caculator.base_func.scale(w).shift(c);
	}

	const depth_info& finest_info = octree.m_depth_info[max_depth];

	std::vector<vec3> finest_v(finest_info.node_number);
	float beta = 1.0f;
	for (unsigned i = 0; i < finest_info.node_number; ++i)
	{
		vec3& v = finest_v[i];
		v.x = 0.0f; v.y = 0.0f; v.z = 0.0f;
		unsigned index = finest_info.start_index + i;
		const node& n = octree.m_nodes[index];

		double cx = 0.0;
		double cy = 0.0;
		double cz = 0.0;
		double w = 0.0;

		caculator.node_width_center(cx, cy, cz, w, n.n.k, max_depth);

		PPolynomial<2> x_func = caculator.base_func.scale(w).shift(cx);
		PPolynomial<2> y_func = caculator.base_func.scale(w).shift(cy);
		PPolynomial<2> z_func = caculator.base_func.scale(w).shift(cz);
		for (unsigned j = 0; j < 27; ++j)
		{
			if (n.neighbors[j] >= 0)
			{
				const node& nn = octree.m_nodes[n.neighbors[j]];
				for (unsigned k = 0; k < nn.n.point_number; ++k)
				{
					unsigned pindex = nn.n.start_index + k;
					point& p = octree.m_sorted_points[pindex];

					float f = (float)(x_func(p.position.x)*y_func(p.position.y)*z_func(p.position.z));
					v.x += f * p.normal.x * beta;
					v.y += f * p.normal.y * beta;
					v.z += f * p.normal.z * beta;
				}
			}
		}
	}

	std::vector<offset> node_offset(octree.m_nodes.size());
	for (unsigned d = 0; d <= octree.m_depth; ++d)
	{
		const depth_info& df = octree.m_depth_info[d];
		for (unsigned i = 0; i < df.node_number; ++i)
		{
			unsigned index = i + df.start_index;
			offset& off = node_offset[index];
			key2index(octree.m_nodes[index].n.k, off.x, off.y, off.z, d);

			double cx, wx;
			double cy, wy;
			double cz, wz;
			index2centerandwidth(off.x, cx, wx);
			index2centerandwidth(off.y, cy, wy);
			index2centerandwidth(off.z, cz, wz);

			node& n = octree.m_nodes[index];
			n.c.x = (float)cx;
			n.c.y = (float)cy;
			n.c.z = (float)cz;
			n.w = (float)wx;
		}
	}

	std::vector<float> b(octree.m_nodes.size());
	for (unsigned d = 0; d <= octree.m_depth; ++d)
	{
		const depth_info& df = octree.m_depth_info[d];
		for (unsigned i = 0; i < df.node_number; ++i)
		{
			unsigned index = i + df.start_index;
			const node& o = octree.m_nodes[index];

			unsigned xi1, yi1, zi1;

			offset& off = node_offset[index];
			xi1 = off.x; yi1 = off.y; zi1 = off.z;

			for (unsigned j = 0; j < 27; ++j)
			{
				if (o.neighbors[j] >= 0)
				{
					const node& n = octree.m_nodes[o.neighbors[j]];
					for (unsigned k = 0; k < n.n.n_number; ++k)
					{
						unsigned idx = k + n.n.n_start_index;
						if (idx >= finest_info.node_number)
							std::cout << "error." << std::endl;
						const vec3& v_ = finest_v[idx];
						vec3 u_;
		
						unsigned xi2, yi2, zi2;
						offset& off_ = node_offset[idx + finest_info.start_index];
						xi2 = off_.x; yi2 = off_.y; zi2 = off_.z;

						u_.x = (float)(m_lookup_table_1[xi1*res + xi2] * m_lookup_table_0[yi1*res + yi2] * m_lookup_table_0[zi1*res + zi2]);
						u_.y = (float)(m_lookup_table_0[xi1*res + xi2] * m_lookup_table_1[yi1*res + yi2] * m_lookup_table_0[zi1*res + zi2]);
						u_.z = (float)(m_lookup_table_0[xi1*res + xi2] * m_lookup_table_0[yi1*res + yi2] * m_lookup_table_1[zi1*res + zi2]);

						b[index] += dot(v_, u_);
					}
				}
			}
		}

		std::cout << "Splat " << d << std::endl;
	}

	std::vector<float>& phi = octree.m_phi;
	for (unsigned d = 1; d <= octree.m_depth; ++d)
	{
		const depth_info& df = octree.m_depth_info[d];

		unsigned m = df.node_number;
		Eigen::SparseMatrix<float> M(m, m);
		typedef Eigen::Triplet<float> T;
		std::vector<T> triplet_list;
		triplet_list.reserve(27 * m);
		Eigen::VectorXf B(m); B.setZero();

		for (unsigned i = 0; i < m; ++i)
		{
			const node& o = octree.m_nodes[i+df.start_index];
			unsigned xi1, yi1, zi1;
			
			offset& off = node_offset[i+df.start_index];
			xi1 = off.x; yi1 = off.y; zi1 = off.z;
			
			//for (unsigned j = 0; j < m; ++j)
			//{
			//	const node& o_ = octree.m_nodes[j + df.start_index];
			//	unsigned xi2, yi2, zi2;
			//
			//	offset& off_ = node_offset[j + df.start_index];
			//	xi2 = off_.x; yi2 = off_.y; zi2 = off_.z;
			//
			//			float v = (float)(m_lookup_table_2[xi1*res + xi2] * m_lookup_table_0[yi1*res + yi2] * m_lookup_table_0[zi1*res + zi2])
			//				+ (float)(m_lookup_table_0[xi1*res + xi2] * m_lookup_table_2[yi1*res + yi2] * m_lookup_table_0[zi1*res + zi2])
			//				+ (float)(m_lookup_table_0[xi1*res + xi2] * m_lookup_table_0[yi1*res + yi2] * m_lookup_table_2[zi1*res + zi2]);
			//	
			//			if (abs(v) > 1e-8 && i <= j)
			//			{
			//				if(i == j)
			//					triplet_list.push_back(T(i, i, v));
			//				else
			//				{
			//					triplet_list.push_back(T(i, j, v));
			//					triplet_list.push_back(T(j, i, v));
			//				}
			//			}
			//}
			for (unsigned j = 0; j < 27; ++j)
			{
				int nindex = o.neighbors[j];
				if (nindex >= 0)
				{
					unsigned index = nindex - df.start_index;
			
					offset& off_ = node_offset[nindex];
					unsigned xi2, yi2, zi2;
					xi2 = off_.x; yi2 = off_.y; zi2 = off_.z;
			
					float v = (float)(m_lookup_table_2[xi1*res + xi2] * m_lookup_table_0[yi1*res + yi2] * m_lookup_table_0[zi1*res + zi2])
						+ (float)(m_lookup_table_0[xi1*res + xi2] * m_lookup_table_2[yi1*res + yi2] * m_lookup_table_0[zi1*res + zi2])
						+ (float)(m_lookup_table_0[xi1*res + xi2] * m_lookup_table_0[yi1*res + yi2] * m_lookup_table_2[zi1*res + zi2]);
			
					if (abs(v) > 1e-8 && i <= index)
					{
						if(index == i)
							triplet_list.push_back(T(i, index, v));
						else
						{
							triplet_list.push_back(T(i, index, v));
							triplet_list.push_back(T(index, i, v));
						}
					}
				}
			}
		}

		for (unsigned i = 0; i < m; ++i)
			B(i) = b[i+df.start_index];
		M.setFromTriplets(triplet_list.begin(), triplet_list.end());
		Eigen::SimplicialCholesky<Eigen::SparseMatrix<float>> solver;
		solver.compute(M);
		Eigen::VectorXf x = solver.solve(B);
		if (solver.info() != Eigen::ComputationInfo::Success)
			std::cout << "Solve error." << std::endl;
		std::cout << "Matrix on depth " << d << std::endl;
		save(m, triplet_list, B, x, d);

		for (unsigned i = 0; i < df.node_number; ++i)
			phi[i + df.start_index] = x(i);

		//if (d < max_depth)
		//{
		//	const depth_info& ddf = octree.m_depth_info[d+1];
		//	for (unsigned i = 0; i < ddf.node_number; ++i)
		//	{
		//		float mod = 0.0f;
		//		const node& o = octree.m_nodes[i + ddf.start_index];
		//
		//		unsigned xi1, yi1, zi1;
		//		offset& off = node_offset[i + ddf.start_index];
		//		xi1 = off.x; yi1 = off.y; zi1 = off.z;
		//
		//		unsigned pindex = o.n.parent;
		//		const node& po = octree.m_nodes[pindex];
		//		for (unsigned j = 0; j < 27; ++j)
		//		{
		//			if (po.neighbors[j] >= 0)
		//			{
		//				unsigned ppindex = po.neighbors[j];
		//
		//				unsigned xi2, yi2, zi2;
		//				offset& off_ = node_offset[ppindex];
		//				xi2 = off_.x; yi2 = off_.y; zi2 = off_.z;
		//
		//				float L = (float)(m_lookup_table_2[xi1*res + xi2] * m_lookup_table_0[yi1*res + yi2] * m_lookup_table_0[zi1*res + zi2])
		//					+ (float)(m_lookup_table_0[xi1*res + xi2] * m_lookup_table_2[yi1*res + yi2] * m_lookup_table_0[zi1*res + zi2])
		//					+ (float)(m_lookup_table_0[xi1*res + xi2] * m_lookup_table_0[yi1*res + yi2] * m_lookup_table_2[zi1*res + zi2]);
		//
		//				mod += L * x(po.neighbors[j]);
		//			}
		//		}
		//
		//		b[i + ddf.start_index] -= mod;
		//	}
		//}
	}

	std::cout << "value solution." << std::endl;

	float total_value = 0.0f;
	for (unsigned i = 0; i < octree.m_sorted_points.size(); ++i)
	{
		float value = 0.0f;
		const point& p = octree.m_sorted_points[i];

		std::stack<unsigned> stac;
		stac.push(0);
		while (!stac.empty())
		{
			unsigned index = stac.top();
			stac.pop();

			const node& n = octree.m_nodes[index];

			const offset& off = node_offset[index];
			
			if (index >= finest_info.start_index)
			{
				float F = (float)(funcs[off.x](p.position.x) * funcs[off.y](p.position.y) * funcs[off.z](p.position.z));
				value += octree.m_phi[index] * F;
			}

			for (unsigned i = 0; i < 7; ++i)
			{
				if (n.n.children[i] >= 0)
				{
					const node& cn = octree.m_nodes[n.n.children[i]];
					if ((std::abs(p.position.x - cn.c.x) < cn.w )
						&& (std::abs(p.position.y - cn.c.y) < cn.w)
						&& (std::abs(p.position.z - cn.c.z) < cn.w))
					{
						stac.push(n.n.children[i]);
					}
				}
			}
		}

		total_value += value;
		octree.m_point_phi[i] = value;
	}

	float iso_value = total_value / (float)octree.m_sorted_points.size();

	auto f = [&octree, &funcs, &node_offset, &finest_info](const vec3& c)->float {
		float value = 0.0f;
		std::stack<unsigned> stac;
		stac.push(0);
		while (!stac.empty())
		{
			unsigned index = stac.top();
			stac.pop();

			const node& n = octree.m_nodes[index];

			const offset& off = node_offset[index];

			if (index >= finest_info.start_index)
			{
				float F = (float)(funcs[off.x](c.x) * funcs[off.y](c.y) * funcs[off.z](c.z));
				value += octree.m_phi[index] * F;
			}

			for (unsigned i = 0; i < 7; ++i)
			{
				if (n.n.children[i] >= 0)
				{
					const node& cn = octree.m_nodes[n.n.children[i]];
					if ((std::abs(c.x - cn.c.x) < cn.w)
						&& (std::abs(c.y - cn.c.y) < cn.w)
						&& (std::abs(c.z - cn.c.z) < cn.w))
					{
						stac.push(n.n.children[i]);
					}
				}
			}
		}
		return value;
	};

	depth_info& vf = octree.m_depth_info[octree.m_depth];
	
	std::vector<vec3> points;
	std::vector<unsigned> triangles;

	unsigned empty_count = 0;
	for (unsigned p = 0; p < vf.node_number; ++p)
	{
		const node& n = octree.m_nodes[p + vf.start_index];
		float w = n.w;
		const vec3& c = n.c;

		unsigned kind = 0;

		std::vector<vec3> edge_vertex(12);
		std::vector<vec3> vertex(8);
		std::vector<float> vertex_value(8);

		for (unsigned x = 0; x < 2; ++x)
		{
			for (unsigned y = 0; y < 2; ++y)
			{
				for (unsigned z = 0; z < 2; ++z)
				{
					float vx = c.x - 0.5f * w + (float)x * w;
					float vy = c.y - 0.5f * w + (float)y * w;
					float vz = c.z - 0.5f * w + (float)z * w;

					vec3 v;
					v.x = vx; v.y = vy; v.z = vz;
					float value = f(v) - iso_value;

					unsigned flag = (z << 2)|(y<<1) + ( y == 0 ? x : (1 - x));
					if (value > 0.0f) kind |= (1<<flag);

					vertex_value[flag] = value;
					vertex[flag] = v;

					/*unsigned orient = edge_index / 4;
					unsigned j = edge_index % 2;
					unsigned k = (edge_index % 4) / 2;

					if (orient == 2)
					{
					v.z = c.z;

					if (j == 0) v.x = c.x - w / 2.0f;
					else v.x = c.x + w / 2.0f;

					if (k == 0) v.y = c.y - w / 2.0f;
					else v.y = c.y + w / 2.0f;
					}
					else if (orient == 1)
					{
					v.y = c.y;
					if (j == 0) v.x = c.x - w / 2.0f;
					else v.x = c.x + w / 2.0f;

					if (k == 0) v.z = c.z - w / 2.0f;
					else v.z = c.z + w / 2.0f;
					}
					else
					{
					v.x = c.x;
					if (j == 0) v.y = c.y - w / 2.0f;
					else v.y = c.y + w / 2.0f;

					if (k == 0) v.z = c.z - w / 2.0f;
					else v.z = c.z + w / 2.0f;
					}*/
				}
			}
		}

		for (unsigned i = 0; i < 12; ++i)
		{
			unsigned v1 = edge_2_vertex[i][0];
			unsigned v2 = edge_2_vertex[i][1];

			const vec3& vv1 = vertex[v1];
			const vec3& vv2 = vertex[v2];

			auto f = [](const vec3& v1, const vec3& v2, float iso1, float iso2, vec3& c) {
				float lambda = std::abs(iso1) / (std::abs(iso1) + std::abs(iso2));
				c.x = (1.0f - lambda) * v1.x + lambda * v2.x;
				c.y = (1.0f - lambda) * v1.y + lambda * v2.y;
				c.z = (1.0f - lambda) * v1.z + lambda * v2.z;
			};

			f(vv1, vv2, vertex_value[v1], vertex_value[v2], edge_vertex[i]);
		}

		std::vector<int> ver;
		for (unsigned i = 0; i < 13; i = i + 3)
		{
			if (mc_triangles[kind][i] != -1)
			{
				ver.push_back(mc_triangles[kind][i]);
				ver.push_back(mc_triangles[kind][i+1]);
				ver.push_back(mc_triangles[kind][i+2]);
			}
		}
		
		if (ver.empty())
			++empty_count;

		unsigned start_vertex_index = (unsigned)points.size();
		for (unsigned i = 0; i < ver.size(); ++i)
			triangles.push_back(start_vertex_index + i);
		
		for (unsigned i = 0; i < ver.size(); ++i)
		{
			unsigned edge_index = ver[i];

			points.push_back(edge_vertex[edge_index]);
		}
	}

	std::cout << "total count. " << vf.node_number << " empty count. " << empty_count << std::endl;
	std::fstream out;
	out.open("E:\\Sim\\gpu_poisson\\test_mc.stl", std::ios::out | std::ios::binary);

	char header[80];
	memset(header, 0, 80);
	out.write(header, 80);

	unsigned tri_size = (unsigned)triangles.size() / 3;
	out.write((const char*)&tri_size, sizeof(unsigned));

	for (unsigned i = 0; i < tri_size; ++i)
	{
		unsigned index1 = triangles[3 * i];
		unsigned index2 = triangles[3 * i + 1];
		unsigned index3 = triangles[3 * i + 2];

		float vertex1[3];
		float vertex2[3];
		float vertex3[3];

		auto get = [&points](float* vertex, unsigned index) {
			vec3& v = points[index];
			memcpy(vertex, &v, 3 * sizeof(float));
		};

		get(vertex1, index1);
		get(vertex2, index2);
		get(vertex3, index3);

		float nf[3];
		vec3 v12; v12.x = vertex2[0] - vertex1[0]; v12.y = vertex2[1] - vertex1[1]; v12.z = vertex2[2] - vertex1[2];
		vec3 v13; v13.x = vertex3[0] - vertex1[0]; v13.y = vertex3[1] - vertex1[1]; v13.z = vertex3[2] - vertex1[2];
		vec3 vn; cross(v12, v13, vn);
		nf[0] = vn.x; nf[1] = vn.y; nf[2] = vn.z;
		out.write((const char*)&nf[0], 3 * sizeof(float));

		out.write((const char*)&vertex1[0], 3 * sizeof(float));
		out.write((const char*)&vertex2[0], 3 * sizeof(float));
		out.write((const char*)&vertex3[0], 3 * sizeof(float));

		unsigned short attributes = 0;
		out.write((const char*)&attributes, sizeof(unsigned short));
	}

	out.close();
}