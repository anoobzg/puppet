#include "harmonic_caculator.h"
#include <Eigen\Dense>
#include <Eigen\Sparse>
#include <Eigen\SVD>
#include "MeshCurvature.h"
#include "normal_calculator.h"
#include <fstream>

#include <stack>
HarmonicCaculator::HarmonicCaculator(Mesh& mesh)
	:m_mesh(mesh)
{
	m_curvature = new float[mesh.vertex_number];
	MeshCurvature mc(m_mesh);
	mc.CalCurvature(GuassCurvature, m_curvature);

	bool result = Memory_Importer<Polyhedron>::Build_From_Memory(m_mesh.vertex_number, m_mesh.vertex_position,
		m_mesh.triangle_number, m_mesh.triangle_index, m_poly);

	if (!result) std::cout << "Build Polyhedron Error." << std::endl;

	BuildVertex();
	BuildWeights();
}

HarmonicCaculator::~HarmonicCaculator()
{
	delete[]m_curvature;
}

void HarmonicCaculator::BuildVertex()
{
	Vertex_iterator vit = m_poly.vertices_begin();
	for (; vit != m_poly.vertices_end(); ++vit)
	{
		Polyhedron::Halfedge_around_vertex_circulator circulator(vit->halfedge());
		vit->m_normal = Normal_Caculator<Polyhedron>::CalculateNormal(circulator);
	}

	double theta = 0.001;
	for (vit = m_poly.vertices_begin(); vit != m_poly.vertices_end(); ++vit)
	{
		Halfedge_iterator eit = vit->halfedge();
		Halfedge_iterator eet = eit;

		const Point& p = vit->point();
		const Vector& n = vit->m_normal;

		bool concave = false;
		do
		{
			const Point& pj = eit->opposite()->vertex()->point();
			const Vector& nj = eit->opposite()->vertex()->m_normal;

			Vector v1 = p - pj;
			Vector v2 = nj - n;
			double len = std::sqrt(v1.squared_length());
			double d = (v1 * v2) / (len + 0.0001);
			if (d > theta)
			{
				concave = true;
				break;
			}
			eit = eit->next()->opposite();
		} while (eit != eet);

		vit->is_concave = concave;
	}
}

template<typename T>
static T _distance(T vertex1[3], T vertex2[3])
{
	return sqrt(
		(vertex1[0] - vertex2[0]) * (vertex1[0] - vertex2[0]) +
		(vertex1[1] - vertex2[1]) * (vertex1[1] - vertex2[1]) +
		(vertex1[2] - vertex2[2]) * (vertex1[2] - vertex2[2])
	);
}

static double _calAngle(double edge0, double edge1, double edge2)
{
	double cosValue = (edge0 * edge0 + edge1 * edge1 - edge2 * edge2) / (2 * edge0 * edge1);
	cosValue = (cosValue > 1.0) ? 1.0 : ((cosValue < -1.0) ? -1.0 : cosValue);
	return acos(cosValue);
}

// Test another weight method
//void HarmonicCaculator::BuildWeights()
//{
//	Halfedge_iterator eit = m_poly.edges_begin();
//	Halfedge_iterator eet = m_poly.edges_end();
//
//	static auto _getHalfEdgeOppositeAngle = [](Halfedge_iterator itEdge)
//	{
//		float vertexCoord[3][3] = 
//		{
//			itEdge->vertex()->point().x(),
//			itEdge->vertex()->point().y(),
//			itEdge->vertex()->point().z(),
//
//			itEdge->next()->vertex()->point().x(),
//			itEdge->next()->vertex()->point().y(),
//			itEdge->next()->vertex()->point().z(),
//
//			itEdge->next()->next()->vertex()->point().x(),
//			itEdge->next()->next()->vertex()->point().y(),
//			itEdge->next()->next()->vertex()->point().z(),
//		};
//		
//		return _calAngle(_distance(vertexCoord[0], vertexCoord[1]), _distance(vertexCoord[1], vertexCoord[2]), _distance(vertexCoord[0], vertexCoord[2]));
//	};
//
//	for (; eit != eet; ++eit)
//	{
//		if (eit->Set())
//		{
//			continue;
//		}
//
//		Halfedge_iterator oppo_edge = eit->opposite();
//
//		auto alpha = _getHalfEdgeOppositeAngle(eit);
//		auto beta = _getHalfEdgeOppositeAngle(oppo_edge);
//
//		double w = (1.0f / tan(alpha)) + (1.0f / tan(beta));
//		bool is_concave = eit->vertex()->is_concave | oppo_edge->vertex()->is_concave;
//		if (is_concave)
//		{
//			w = 0.001f * w;
//		}
//
//		if (!oppo_edge->Set())
//		{
//			oppo_edge->Weight() = w;
//			oppo_edge->Set() = true;
//		}
//
//		eit->Weight() = w;
//		eit->Set() = true;
//	}
//}

void HarmonicCaculator::BuildWeights()
{
	Halfedge_iterator eit = m_poly.edges_begin();
	Halfedge_iterator eet = m_poly.edges_end();

	double t_len = 0.0;
	double t_w = 0.0;
	for (; eit != eet; ++eit)
	{
		if (eit->Set())
			continue;

		Halfedge_iterator oppo_edge = eit->opposite();
		const Point& p1 = eit->vertex()->point();
		const Point& p2 = oppo_edge->vertex()->point();

		double len = std::sqrt((p1 - p2).squared_length());
		bool is_concave = eit->vertex()->is_concave | oppo_edge->vertex()->is_concave;

		float g1 = m_curvature[eit->vertex()->id()];
		float g2 = m_curvature[eit->vertex()->id()];
		double D = std::abs(g1 + g2) + 0.0001;
		double w = 1.0;
		double beta = 0.01;
		if (is_concave)
		{
			w = beta * len / D;
		}
		else
		{
			w = len / D;
		}

		//w *= 1000000.0;

		double prob = 0.0;
		if (!eit->is_border() && !oppo_edge->is_border())
		{
			Facet_iterator fit = eit->face();
			Facet_iterator ofit = oppo_edge->face();

			typename Polyhedron::Halfedge_around_facet_circulator fcirculator(fit->halfedge());
			typename Polyhedron::Halfedge_around_facet_circulator ofcirculator(ofit->halfedge());
			Vector vf = Normal_Caculator<Polyhedron>::CalculateNormal(fcirculator);
			Vector vo = Normal_Caculator<Polyhedron>::CalculateNormal(ofcirculator);
			prob = 0.1 * (vf - vo).squared_length();
			if (vf*vo < 0.0) prob *= 0.1;
		}

		eit->Weight() = w;
		eit->m_p = prob;

		t_len += prob;
		t_w += w;
		if (!oppo_edge->Set())
		{
			oppo_edge->Weight() = w;
			oppo_edge->m_p = prob;
			oppo_edge->Set() = true;

			t_len += prob;
			t_w += w;
		}

		eit->Set() = true;
	}

	t_len /= (double)m_poly.size_of_halfedges();
	t_w /= (double)m_poly.size_of_halfedges();
	for (eit = m_poly.edges_begin(); eit != eet; ++eit)
	{
		double d = eit->m_p / t_len;
		Halfedge_iterator oppo_edge = eit->opposite();
		const Point& p1 = eit->vertex()->point();
		const Point& p2 = oppo_edge->vertex()->point();

		double len = std::sqrt((p1 - p2).squared_length());
		eit->m_p = len * exp(-d);
		eit->m_p /= t_w;
	}
}

void HarmonicCaculator::Write(const std::vector<unsigned>& base, const std::vector<unsigned>& neg, const std::vector<unsigned>& pos)
{
	std::fstream out;
	out.open("E:\\Sim\\mesh_segment\\matrix", std::ios::out | std::ios::binary);

	unsigned n = m_mesh.vertex_number;
	unsigned bm = base.size();
	unsigned nm = neg.size();
	unsigned pm = pos.size();
	unsigned m = bm + nm + pm;
	unsigned num = m + n;
	out.write((const char*)&num, sizeof(unsigned));
	out.write((const char*)&n, sizeof(unsigned));

	double w = 1000.0;

	typedef Eigen::Triplet<double> T;
	std::vector<T> triplet_list;
	triplet_list.reserve(11 * n + m);
	//build M
	Vertex_iterator vit = m_poly.vertices_begin();
	for (; vit != m_poly.vertices_end(); ++vit)
	{
		unsigned i = vit->id();
		double tw = 0.0;
		Halfedge_iterator eit = vit->halfedge();
		Halfedge_iterator eet = eit;
		do
		{
			unsigned j = eit->opposite()->vertex()->id();
			double ew = eit->Weight();
			triplet_list.push_back(T(i, j, -ew));
			tw += ew;

			eit = eit->next()->opposite();
		} while (eit != eet);

		triplet_list.push_back(T(i, i, tw));
	}

	for (unsigned i = n; i < n + bm; ++i)
	{
		unsigned index = base[i - n];
		triplet_list.push_back(T(i, index, w));
	}
	for (unsigned i = n + bm; i < n + bm + nm; ++i)
	{
		unsigned index = neg[i - n - bm];
		triplet_list.push_back(T(i, index, w));
	}
	for (unsigned i = n + bm + nm; i < n + m; ++i)
	{
		unsigned index = pos[i - n - bm - nm];
		triplet_list.push_back(T(i, index, w));
	}

	unsigned size = triplet_list.size();
	out.write((const char*)&size, sizeof(unsigned));
	for (unsigned i = 0; i < size; ++i)
	{
		T& t = triplet_list.at(i);
		out.write((const char*)&t.row(), sizeof(unsigned));
		out.write((const char*)&t.col(), sizeof(unsigned));
		out.write((const char*)&t.value(), sizeof(double));
	}

	Eigen::VectorXd b(m + n);
	//build b
	for (unsigned i = 0; i < n; ++i)
		b(i) = 0.0;
	for (unsigned i = n; i < n + bm; ++i)
		b(i) = 0.5 * w;
	for (unsigned i = n + bm; i < n + bm + nm; ++i)
		b(i) = 0.0;
	for (unsigned i = n + bm + nm; i < n + m; ++i)
		b(i) = w;

	for (unsigned i = 0; i < num; ++i)
	{
		double v = b(i);
		out.write((const char*)&v, sizeof(double));
	}

	out.close();
}

void HarmonicCaculator::DoPatch(const std::vector<unsigned>& pos, float* harmonic, unsigned* pBoundarySet /*= nullptr*/, unsigned boundarySize /*= 0*/)
{
	static double alpha[4] = { 0.0, 0.5, 0.0, 1.0 };

	unsigned n = m_mesh.vertex_number;
	for (unsigned i = 0; i < n; ++i)
		*(harmonic + i) = 0.0f;

	std::vector<int> vertex_mapping(n, -1);
	std::vector<int> vertex_group(n, 2);

	if (pBoundarySet && boundarySize > 0)
	{
		for (auto& groupIndex : vertex_group)
		{
			groupIndex = 0;
		}

		for (auto posIndex : pos)
		{
			harmonic[posIndex] = 1.0f;
			vertex_group[posIndex] = 3;
		}

		for (unsigned i = 0; i < boundarySize; i++)
		{
			harmonic[pBoundarySet[i]] = 0.0f;
			vertex_group[pBoundarySet[i]] = 2;
		}
	}
	else
	{
		float center[3] = { 0.0f };
		for (unsigned i = 0; i < pos.size(); ++i)
		{
			float* pp = m_mesh.vertex_position + 3 * pos[i];
			center[0] += *pp++; center[1] += *pp++; center[2] += *pp++;
		}

		Point p(center[0] / (float)pos.size(), center[1] / (float)pos.size(), center[2] / (float)pos.size());
		//float len = 135.0f;
		float len = 200.0f;

		std::vector<bool> vertex_visited(m_mesh.vertex_number, false);
		std::vector<Vertex_iterator> seeds;
		Vertex_iterator vit = m_poly.vertices_begin();
		for (; vit != m_poly.vertices_end(); ++vit)
		{
			unsigned i = vit->id();
			if (std::find(pos.begin(), pos.end(), i) != pos.end())
			{
				seeds.push_back(vit);
				vertex_visited[i] = true;
				vertex_group[i] = 3;

				*(harmonic + i) = 1.0f;
			}
		}

		std::vector<unsigned> vertex_id;
		//build M
		while (!seeds.empty())
		{
			std::vector<Vertex_iterator> pseeds;
			for (unsigned i = 0; i < seeds.size(); ++i)
			{
				Vertex_iterator& it = seeds[i];

				Halfedge_iterator eit = it->halfedge();
				Halfedge_iterator eet = eit;
				do
				{
					unsigned j = eit->opposite()->vertex()->id();

					Point& pp = eit->opposite()->vertex()->point();
					bool boundary = false;
					if ((p - pp).squared_length() >= len)
					{
						boundary = true;
						vertex_group[j] = 2;
					}
					if (!vertex_visited[j] && !boundary)
					{
						vertex_visited[j] = true;
						pseeds.push_back(eit->opposite()->vertex());
						vertex_group[j] = 0;
					}

					eit = eit->next()->opposite();
				} while (eit != eet);

			}

			seeds.swap(pseeds);
		}
	}

	//build M
	unsigned index_count = 0;
	Vertex_iterator vit = m_poly.vertices_begin();
	for (; vit != m_poly.vertices_end(); ++vit)
	{
		unsigned i = vit->id();

		if (vertex_group[i] == 0)
		{
			vertex_mapping[i] = index_count;
			++index_count;
		}
	}

	Eigen::SparseMatrix<double> M(index_count, index_count);
	typedef Eigen::Triplet<double> T;
	std::vector<T> triplet_list;
	triplet_list.reserve(11 * index_count);
	Eigen::VectorXd b(index_count); b.setZero();

	for (vit = m_poly.vertices_begin(); vit != m_poly.vertices_end(); ++vit)
	{
		unsigned i = vit->id();
		if (vertex_group[i] == 0)
		{
			double tw = 0.0;
			double bw = 0.0;
			Halfedge_iterator eit = vit->halfedge();
			Halfedge_iterator eet = eit;
			do
			{
				unsigned j = eit->opposite()->vertex()->id();
				double ew = eit->Weight();

				if (vertex_group[j] > 0)
				{
					bw += alpha[vertex_group[j]] * ew;
				}
				else
				{
					triplet_list.push_back(T(vertex_mapping[i], vertex_mapping[j], -ew));
				}
				tw += ew;
				eit = eit->next()->opposite();
			} while (eit != eet);

			//if (tw < 1e-5)
			//	std::cout << "Error........." << std::endl;
			triplet_list.push_back(T(vertex_mapping[i], vertex_mapping[i], 1e-12 + tw));
			b(vertex_mapping[i]) = bw;
		}
	}

	M.setFromTriplets(triplet_list.begin(), triplet_list.end());
	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
	solver.compute(M);
	Eigen::VectorXd x = solver.solve(b);

	for (unsigned i = 0; i < n; ++i)
		if (vertex_group[i] == 0)
			*(harmonic + i) = x(vertex_mapping[i]);
}

void HarmonicCaculator::Do(const std::vector<unsigned>& base, const std::vector<unsigned>& neg, const std::vector<unsigned>& pos,
	float* harmonic)
{
	//Vertex_iterator vit = m_poly.vertices_begin();
	//for (; vit != m_poly.vertices_end(); ++vit)
	//{
	//	unsigned id = vit->id();
	//	if (vit->is_concave) harmonic[id] = 0.0;
	//	else harmonic[id] = 0.5;
	//}

	//unsigned n = m_mesh.vertex_number;
	//unsigned bm = base.size();
	//unsigned nm = neg.size();
	//unsigned pm = pos.size();
	//unsigned m = bm + nm + pm;
	//
	//if (m < 10) return;
	//
	//double w = 1000.0;
	//
	//Eigen::SparseMatrix<double> M(m + n, n);
	//typedef Eigen::Triplet<double> T;
	//std::vector<T> triplet_list;
	//triplet_list.reserve(11 * n + m);
	////build M
	//Vertex_iterator vit = m_poly.vertices_begin();
	//for (; vit != m_poly.vertices_end(); ++vit)
	//{
	//	unsigned i = vit->id();
	//	double tw = 0.0;
	//	Halfedge_iterator eit = vit->halfedge();
	//	Halfedge_iterator eet = eit;
	//	do
	//	{
	//		unsigned j = eit->opposite()->vertex()->id();
	//		double ew = eit->Weight();
	//		triplet_list.push_back(T(i, j, -ew));
	//		tw += ew;
	//
	//		eit = eit->next()->opposite();
	//	} while (eit != eet);
	//
	//	triplet_list.push_back(T(i, i, tw));
	//}
	//
	//for (unsigned i = n; i < n + bm; ++i)
	//{
	//	unsigned index = base[i - n];
	//	triplet_list.push_back(T(i, index, w));
	//}
	//for (unsigned i = n + bm; i < n + bm + nm; ++i)
	//{
	//	unsigned index = neg[i - n - bm];
	//	triplet_list.push_back(T(i, index, w));
	//}
	//for (unsigned i = n + bm + nm; i < n + m; ++i)
	//{
	//	unsigned index = pos[i - n - bm - nm];
	//	triplet_list.push_back(T(i, index, w));
	//}
	//
	//M.setFromTriplets(triplet_list.begin(), triplet_list.end());
	//
	//Eigen::VectorXd b(m + n);
	////build b
	//for (unsigned i = 0; i < n; ++i)
	//	b(i) = 0.0;
	//for (unsigned i = n; i < n + bm; ++i)
	//	b(i) = 0.5 * w;
	//for (unsigned i = n + bm; i < n + bm + nm; ++i)
	//	b(i) = 0.0;
	//for (unsigned i = n + bm + nm; i < n + m; ++i)
	//	b(i) = w;
	//
	//M.makeCompressed();
	//Eigen::SparseMatrix<double> TM = M.transpose();
	//Eigen::SparseMatrix<double> TMM = TM * M;
	//Eigen::VectorXd Tb = TM * b;

	//Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
	////solver.setMode(Eigen::SimplicialCholeskyLLT);
	//solver.compute(TMM);
	//Eigen::VectorXd x = solver.solve(Tb);

	//
	//std::cout << "SimplicialCholesky Solved." << std::endl;

	//Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<double> > solver;
	//solver.setTolerance(0.00001);
	//solver.setMaxIterations(100);
	//solver.compute(M);
	//Eigen::VectorXd x = solver.solve(b);

	//Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	//solver.compute(TMM);
	//Eigen::VectorXd x = solver.solve(Tb);

	//Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
	//solver.setMaxIterations(100);
	//solver.compute(TMM);
	//Eigen::VectorXd x = solver.solve(Tb);
	//std::cout << "Interations. " << solver.iterations() << std::endl;
	//std::cout << "Error. " << solver.error() << std::endl;

	//std::fstream out;
	//out.open("E:\\Sim\\mesh_segment\\result", std::ios::out);
	//for (unsigned i = 0; i < n; ++i)
	//	out << x(i) << std::endl;
	//out.close();
	//for (unsigned i = 0; i < n; ++i)
	//	*(harmonic + i) = x(i);

	static double alpha[4] = { 0.0, 0.5, 0.0, 1.0 };

	std::vector<int> vertex_group(m_mesh.vertex_number, 0); // 1 base, 2 neg, 3 pos
	for (size_t i = 0; i < base.size(); ++i)
	{
		*(harmonic + base[i]) = 0.5;
		vertex_group[base[i]] = 1;
	}
	for (size_t i = 0; i < neg.size(); ++i)
	{
		*(harmonic + neg[i]) = 0.0;
		vertex_group[neg[i]] = 2;
	}
	for (size_t i = 0; i < pos.size(); ++i)
	{
		*(harmonic + pos[i]) = 1.0;
		vertex_group[pos[i]] = 3;
	}

	unsigned n = m_mesh.vertex_number;
	unsigned c = base.size() + pos.size() + neg.size();
	unsigned m = n - c;
	std::vector<int> vertex_mapping(n, -1);

	Eigen::SparseMatrix<double> M(m, m);
	typedef Eigen::Triplet<double> T;
	std::vector<T> triplet_list;
	triplet_list.reserve(11 * m);
	Eigen::VectorXd b(m); b.setZero();

	//build M
	unsigned index_count = 0;
	Vertex_iterator vit = m_poly.vertices_begin();
	for (; vit != m_poly.vertices_end(); ++vit)
	{
		unsigned i = vit->id();

		if (vertex_group[i] == 0)
		{
			vertex_mapping[i] = index_count;
			++index_count;
		}
	}

	for (vit = m_poly.vertices_begin(); vit != m_poly.vertices_end(); ++vit)
	{
		unsigned i = vit->id();
		if (vertex_group[i] == 0)
		{
			double tw = 0.0;
			double bw = 0.0;
			Halfedge_iterator eit = vit->halfedge();
			Halfedge_iterator eet = eit;
			do
			{
				unsigned j = eit->opposite()->vertex()->id();
				double ew = eit->Weight();

				if (vertex_group[j] > 0)
				{
					bw += alpha[vertex_group[j]] * ew;
				}
				else
				{
					triplet_list.push_back(T(vertex_mapping[i], vertex_mapping[j], -ew));
				}
				tw += ew;
				eit = eit->next()->opposite();
			} while (eit != eet);

			//if (tw < 1e-5)
			//	std::cout << "Error........." << std::endl;
			triplet_list.push_back(T(vertex_mapping[i], vertex_mapping[i], 1e-12 + tw));
			b(vertex_mapping[i]) = bw;
		}
	}

	M.setFromTriplets(triplet_list.begin(), triplet_list.end());
	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
	solver.compute(M);
	Eigen::VectorXd x = solver.solve(b);

	for (unsigned i = 0; i < n; ++i)
		if(vertex_group[i] == 0)
			*(harmonic + i) = x(vertex_mapping[i]);

	std::fstream out;
	unsigned nn = 1;
	out.open("E:\\Sim\\mesh_segment\\harmonic", std::ios::out | std::ios::binary);
	out.write((const char*)&m, sizeof(unsigned));
	out.write((const char*)&nn, sizeof(unsigned));
	unsigned size = triplet_list.size();
	out.write((const char*)&size, sizeof(unsigned));
	for (unsigned i = 0; i < size; ++i)
	{
		T& t = triplet_list.at(i);
		out.write((const char*)&t.row(), sizeof(unsigned));
		out.write((const char*)&t.col(), sizeof(unsigned));
		out.write((const char*)&t.value(), sizeof(double));
	}
	for (unsigned i = 0; i < m; ++i)
	{
		for (unsigned j = 0; j < nn; ++j)
		{
			double p = x(i);
			double vb = b(i);
			out.write((const char*)&p, sizeof(double));
			out.write((const char*)&vb, sizeof(double));
		}
	}
	out.close();
}

void HarmonicCaculator::DoRandomWalk(const std::vector<std::vector<unsigned>>& base, float* harmonic)
{
	unsigned seed_face_num = 0;
	unsigned useed_face_num = 0;

	std::vector<int> vertex_group(m_mesh.vertex_number, -1);
	std::vector<int> face_group(m_mesh.triangle_number, -1);
	for (size_t i = 0; i < base.size(); ++i)
	{
		const std::vector<unsigned>& group = base[i];
		for (size_t j = 0; j < group.size(); ++j)
		{
			unsigned index = group[j];
			vertex_group[index] = i;
		}
	}
	for (unsigned i = 0; i < m_mesh.triangle_number; ++i)
	{
		unsigned* tindex = m_mesh.triangle_index + 3 * i;
		for (unsigned j = 0; j < 3; ++j)
		{
			unsigned index = *(tindex + j);
			if (face_group[i] == -1)
			{
				face_group[i] = vertex_group[index];
				vertex_group[index] = -1;
			}
		}
		if (face_group[i] >= 0) ++seed_face_num;
	}
	useed_face_num = m_mesh.triangle_number - seed_face_num;
	unsigned m = useed_face_num;
	unsigned n = (unsigned)base.size();

	Eigen::SparseMatrix<double> A(m, m);
	typedef Eigen::Triplet<double> T;
	std::vector<T> triplet_list;
	triplet_list.reserve(4 * m);

	Eigen::MatrixXd P(m, n);
	P.setZero();
	Eigen::MatrixXd B(m, n);
	B.setZero();
	std::vector<unsigned> unseed_mapping(m, 0);
	unsigned row = 0;
	Facet_iterator fit = m_poly.facets_begin();
	for (; fit != m_poly.facets_end(); ++fit)
	{
		unsigned id = fit->id();
		if (face_group[id] == -1)
		{
			fit->unseed_order = row;
			unseed_mapping[row] = id;
			++row;
		}
	}

	for (fit = m_poly.facets_begin(); fit != m_poly.facets_end(); ++fit)
	{
		unsigned id = fit->id();
		if (face_group[id] == -1)
		{
			unsigned useed_order = fit->unseed_order;
			triplet_list.push_back(T(useed_order, useed_order, 1.0));

			std::vector<double> ppp;
			{
				Halfedge_iterator hit = fit->halfedge();
				Halfedge_iterator heit = hit;
				do
				{
					ppp.push_back(hit->m_p);
					hit = hit->next();
				} while (hit != heit);
			}

			//double tp = (ppp[0] + ppp[1] + ppp[2]);
			//ppp[0] /= tp; ppp[1] /= tp; ppp[2] /= tp;

			{
				unsigned i = 0;
				Halfedge_iterator hit = fit->halfedge();
				Halfedge_iterator heit = hit;
				do
				{
					Halfedge_iterator ohit = hit->opposite();
					if (!ohit->is_border())
					{
						Facet_iterator fk = ohit->face();
						int gid = face_group[fk->id()];
						if (gid == -1)
						{
							triplet_list.push_back(T(useed_order, fk->unseed_order, -ppp[i]));
						}
						else
						{
							B(useed_order, gid) = ppp[i];
						}
					}
					hit = hit->next();
					++i;
				} while (hit != heit);
			}
		}
	}

	A.setFromTriplets(triplet_list.begin(), triplet_list.end());

	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
	//solver.setMode(Eigen::SimplicialCholeskyLLT);
	solver.compute(A);
	P = solver.solve(B);
	//Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
	//solver.setMaxIterations(100);
	//solver.compute(A);
	//P = solver.solve(B);
	//std::cout << "Interations. " << solver.iterations() << std::endl;
	//std::cout << "Error. " << solver.error() << std::endl;
	
	for (unsigned i = 0; i < m; ++i)
	{
		double max = DBL_MIN;
		unsigned index = 0;

		for (unsigned j = 0; j < n; ++j)
		{
			double v = P(i, j);
			if (v > max)
			{
				index = j;
				max = v;
			}
		}

		unsigned fid = unseed_mapping[i];
		face_group[fid] = index;
	}

	float delta = 1.0f / (float)n;
	for (unsigned i = 0; i < m_mesh.triangle_number; ++i)
	{
		unsigned* tindex = m_mesh.triangle_index + 3 * i;
		for (unsigned j = 0; j < 3; ++j)
		{
			unsigned vertex_id = *(tindex + j);
			unsigned g = face_group[i];
			*(harmonic + vertex_id) = (float)g * delta;
		}
	}

	std::fstream out;
	out.open("E:\\Sim\\mesh_segment\\random_walk", std::ios::out | std::ios::binary);
	out.write((const char*)&m, sizeof(unsigned));
	out.write((const char*)&n, sizeof(unsigned));
	unsigned size = triplet_list.size();
	out.write((const char*)&size, sizeof(unsigned));
	for (unsigned i = 0; i < size; ++i)
	{
		T& t = triplet_list.at(i);
		out.write((const char*)&t.row(), sizeof(unsigned));
		out.write((const char*)&t.col(), sizeof(unsigned));
		out.write((const char*)&t.value(), sizeof(double));
	}
	for (unsigned i = 0; i < m; ++i)
	{
		for (unsigned j = 0; j < n; ++j)
		{
			double p = P(i, j);
			double b = B(i, j);
			out.write((const char*)&p, sizeof(double));
			out.write((const char*)&b, sizeof(double));
		}
	}
	out.close();
}

#include <ppl.h>
void HarmonicCaculator::DoParallPatch(const std::vector<std::vector<unsigned>>& teeth_seeds, const std::vector<unsigned>& gum_seeds, float* harmonic)
{
	PROFILE_TIMER

	//0. unsure, 1. base, 2. neg, 3. pos, 4. boundary
	static double alpha[4] = { 0.0, 0.5, 0.0, 1.0 };

	unsigned n = m_mesh.vertex_number;
	unsigned m = (unsigned)teeth_seeds.size();
	std::vector<unsigned> vertex_identity(n, 0);
	//0 unsure, 1 teeth, 2 gum

	for (size_t i = 0; i < gum_seeds.size(); ++i)
		vertex_identity[gum_seeds[i]] = 2;
	for (size_t i = 0; i < m; ++i)
		for (size_t j = 0; j < teeth_seeds[i].size(); ++j)
			vertex_identity[teeth_seeds[i][j]] = 1;

	std::vector<std::vector<unsigned>> teeth_seeds_patch(m);

	//float max_search_len = 165.0f;
	float max_search_len = 400.0f;
	//for (size_t tp = 0; tp < m; ++tp)
	//{
	Concurrency::parallel_for<size_t>(0, m, [this, &teeth_seeds, &n, &max_search_len, &vertex_identity, &teeth_seeds_patch](size_t tp) {
		const std::vector<unsigned>& teeth_seed = teeth_seeds[tp];
		unsigned seed_num = (unsigned)teeth_seed.size();

		//std::vector<unsigned> patch;

		std::vector<int> vertex_mapping(n, -1);
		std::vector<int> vertex_group(n, 1);

		float center[3] = { 0.0f };
		for (unsigned i = 0; i < seed_num; ++i)
		{
			float* pp = m_mesh.vertex_position + 3 * teeth_seed[i];
			center[0] += *pp++; center[1] += *pp++; center[2] += *pp++;
		}

		Point p(center[0] / (float)seed_num, center[1] / (float)seed_num, center[2] / (float)seed_num);

		std::vector<bool> vertex_visited(n, false);
		std::vector<Vertex_iterator> seeds;
		Vertex_iterator vit = m_poly.vertices_begin();
		for (; vit != m_poly.vertices_end(); ++vit)
		{
			unsigned i = vit->id();
			if (std::find(teeth_seed.begin(), teeth_seed.end(), i) != teeth_seed.end())
			{
				seeds.push_back(vit);
				vertex_visited[i] = true;
				vertex_group[i] = 3;
			}
		}

		//build M
		while (!seeds.empty())
		{
			std::vector<Vertex_iterator> next_seeds;
			for (unsigned i = 0; i < seeds.size(); ++i)
			{
				Vertex_iterator& it = seeds[i];

				Halfedge_iterator eit = it->halfedge();
				Halfedge_iterator eet = eit;
				do
				{
					unsigned j = eit->opposite()->vertex()->id();

					Point& pp = eit->opposite()->vertex()->point();
					bool boundary = false;
					if ((p - pp).squared_length() >= max_search_len)
					{
						vertex_group[j] = 4;
						boundary = true;
					}
					if (!vertex_visited[j] && !boundary)
					{
						vertex_visited[j] = true;
						next_seeds.push_back(eit->opposite()->vertex());

						if (vertex_identity[j] == 2)
							vertex_group[j] = 1;
						else if (vertex_identity[j] == 1)
							vertex_group[j] = 2;
						else
							vertex_group[j] = 0;
					}

					eit = eit->next()->opposite();
				} while (eit != eet);

			}

			seeds.swap(next_seeds);
		}

		//build M
		unsigned index_count = 0;
		vit = m_poly.vertices_begin();
		std::vector<unsigned> vertex_reverse_mapping;
		for (; vit != m_poly.vertices_end(); ++vit)
		{
			unsigned i = vit->id();

			if (vertex_group[i] == 0)
			{
				vertex_mapping[i] = index_count;
				++index_count;
				vertex_reverse_mapping.push_back(i);
			}
		}

		Eigen::SparseMatrix<double> M(index_count, index_count);
		typedef Eigen::Triplet<double> T;
		std::vector<T> triplet_list;
		triplet_list.reserve(11 * index_count);
		Eigen::VectorXd b(index_count); b.setZero();

		for (vit = m_poly.vertices_begin(); vit != m_poly.vertices_end(); ++vit)
		{
			unsigned i = vit->id();
			if (vertex_group[i] == 0)
			{
				double tw = 0.0;
				double bw = 0.0;
				Halfedge_iterator eit = vit->halfedge();
				Halfedge_iterator eet = eit;
				do
				{
					unsigned j = eit->opposite()->vertex()->id();
					if (vertex_group[j] != 4)
					{
						double ew = eit->Weight();

						if (vertex_group[j] > 0)
						{
							bw += alpha[vertex_group[j]] * ew;
						}
						else
						{
							triplet_list.push_back(T(vertex_mapping[i], vertex_mapping[j], -ew));
						}
						tw += ew;
					}
					eit = eit->next()->opposite();
				} while (eit != eet);

				//if (tw < 1e-5)
				//	std::cout << "Error........." << std::endl;
				triplet_list.push_back(T(vertex_mapping[i], vertex_mapping[i], 1e-12 + tw));
				b(vertex_mapping[i]) = bw;
			}
		}

		M.setFromTriplets(triplet_list.begin(), triplet_list.end());
		Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
		solver.compute(M);
		Eigen::VectorXd x = solver.solve(b);

		for (unsigned i = 0; i < (unsigned)vertex_reverse_mapping.size(); ++i)
		{
			if (x(i) > 0.6)
				teeth_seeds_patch[tp].push_back(vertex_reverse_mapping[i]);
		}
	//}
	});

	for (size_t i = 0; i < teeth_seeds_patch.size(); ++i)
		for (size_t j = 0; j < teeth_seeds_patch[i].size(); ++j)
			vertex_identity[teeth_seeds_patch[i][j]] = 1;
	for (unsigned i = 0; i < n; ++i)
		*(harmonic + i) = vertex_identity[i] == 1 ? 1.0f : 0.5f;

	PROFILE_TIME("DoParallPatch")
}