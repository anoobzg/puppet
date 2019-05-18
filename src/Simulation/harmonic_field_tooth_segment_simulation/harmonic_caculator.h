#pragma once
#include "Mesh.h"
#include <vector>
#include <map>

#include "importer_memory.h"
#include "harmonic_polygon_header.h"
#include "profiler.h"

#include <Eigen\Sparse>

#define SEG_USE_PATCH 1
//#include <boost\graph\adjacency_list.hpp>
//
//using namespace boost;
//typedef property<edge_weight_t, double> Edge_Weight;
//typedef boost::adjacency_list<vecS, vecS, undirectedS, no_property, Edge_Weight> MeshGraph;
using namespace LauncaGeometry;
class HarmonicCaculator
{
public:
	HarmonicCaculator(Mesh& mesh);
	~HarmonicCaculator();

	void Do(const std::vector<unsigned>& base, const std::vector<unsigned>& neg, const std::vector<unsigned>& pos,
		float* harmonic);
	void DoPatch(const std::vector<unsigned>& pos, float* harmonic, unsigned* pBoundarySet = nullptr, unsigned boundarySize = 0);
	void DoRandomWalk(const std::vector<std::vector<unsigned>>& base, float* harmonic);
	void Write(const std::vector<unsigned>& base, const std::vector<unsigned>& neg, const std::vector<unsigned>& pos);

	void DoParallPatch(const std::vector<std::vector<unsigned>>& teeth_seeds, const std::vector<unsigned>& gum_seeds, float* harmonic);
protected:
	void BuildWeights();
	void BuildVertex();
private:
	Mesh& m_mesh;

	Polyhedron m_poly;
	float* m_curvature;
};