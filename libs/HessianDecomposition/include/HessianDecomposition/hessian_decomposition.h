#pragma once

#include <dlib/clustering.h>
#include <Core/array.h>
#include <set>

namespace hessian_decomposition
{
using intV = std::vector<uint>;

std::vector<unsigned long> spectralCluster (
    const dlib::matrix<double>& A,
    const unsigned long num_clusters
);

struct Problem
{
  std::vector<intV> xmasks; // vector of LOOSELY coupled subproblems
  std::vector<uint> sizes;  // number of non zeros in each xmask
  std::vector<std::set<uint>> overlaps; // overlap with other subproblems
};

struct Decomposition
{
  std::vector<Problem> problems; // vector of INDEPENDANT subproblems
};

dlib::matrix<double> buildAdjacancyMatrix(const arr&);

std::vector<dlib::matrix<double>> buildAdjacancyMatrices(const arr & H, std::vector<Problem>& pbs);

Problem buildDecomposition(const dlib::matrix<double>& A, std::vector<unsigned long> & sparsestCut, uint numberOfCluster);

// each cluster bigger than splitting_threshold will be split into number_of_cluster
Decomposition decomposeHessian(const arr& H, uint splittingThreshold, uint numberOfCluster);
Decomposition decomposeSparseHessian(const arr& H, uint splittingThreshold, uint numberOfCluster); // to be improved
}
