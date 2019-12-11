#pragma once
#include <Eigen\SparseCholesky>
#include <Eigen\Sparse>

void save(unsigned m, std::vector<Eigen::Triplet<float>>& triplets, Eigen::VectorXf& B, Eigen::VectorXf& V, unsigned d);
void save_table(unsigned res, double* data, const char* file);