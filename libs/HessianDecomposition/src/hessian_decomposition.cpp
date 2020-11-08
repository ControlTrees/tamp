#include <HessianDecomposition/hessian_decomposition.h>

#include <unordered_map>

namespace hessian_decomposition
{

static bool sanityCheck(const arr& H, const Decomposition& decomp)
{
  uint n = 0;

  for(auto pb: decomp.problems)
  {
    for(auto i = 0; i < pb.xmasks.size(); ++i)
    {
      n += pb.sizes[i] - pb.overlaps[i].size();

      uint m = 0;
      for(auto j = 0; j < pb.xmasks[i].size(); ++j)
      {
        if(pb.xmasks[i][j]) ++m;
      }

      CHECK_EQ(pb.sizes[i], m, "wrong size computation!");
    }
  }

  return n <= H.d0;
}

std::vector<unsigned long> spectralCluster (
    const dlib::matrix<double>& A,
    const unsigned long num_clusters
    )
{
  using namespace dlib;

  DLIB_CASSERT(num_clusters > 1,
               "\t std::vector<unsigned long> spectral_cluster(k,samples,num_clusters)"
               << "\n\t num_clusters can't be 0."
               );

  auto K = A; // copy

  matrix<double,0,1> D(K.nr());
  for (long r = 0; r < K.nr(); ++r)
    D(r) = sum(rowm(K,r));
  D = sqrt(reciprocal(D));
  K = diagm(D)*K*diagm(D);
  matrix<double> u,w,v;
  // Use the normal SVD routine unless the matrix is really big, then use the fast
  // approximate version.
  if (K.nr() < 1000)
    svd3(K,u,w,v);
  else
    svd_fast(K,u,w,v, num_clusters+100, 5);
  // Pick out the eigenvectors associated with the largest eigenvalues.
  rsort_columns(v,w);
  v = colm(v, range(0,num_clusters-1));
  // Now build the normalized spectral vectors, one for each input vector.
  std::vector<matrix<double,0,1> > spec_samps, centers;
  for (long r = 0; r < v.nr(); ++r)
  {
    spec_samps.push_back(trans(rowm(v,r)));
    const double len = length(spec_samps.back());
    if (len != 0)
      spec_samps.back() /= len;
  }
  // Finally do the K-means clustering
  pick_initial_centers(num_clusters, centers, spec_samps);
  find_clusters_using_kmeans(spec_samps, centers);
  // And then compute the cluster assignments based on the output of K-means.
  std::vector<unsigned long> assignments;
  for (unsigned long i = 0; i < spec_samps.size(); ++i)
    assignments.push_back(nearest_center(centers, spec_samps[i]));

  return assignments;
}

dlib::matrix<double> buildAdjacancyMatrix(const arr& H)
{
  dlib::matrix<double> A(H.d0, H.d1);

  // hessian
  if(isSparseMatrix(H))
  {
    for(auto i = 0; i < H.d0; ++i)
    {
      for(auto j = 0; j < H.d1; ++j)
      {
        A(i, j) = 0;
      }
    }

    auto Hs = dynamic_cast<rai::SparseMatrix*>(H.special);

    const auto & nzs = Hs->elems;

    for(auto i = 0; i < nzs.d0; ++i)
    {
      auto I = nzs(i, 0);
      auto J = nzs(i, 1);

      A(I, J) = 1;
    }
  }
  else
  {
    for(auto i = 0; i < H.d0; ++i)
    {
      for(auto j = 0; j < H.d1; ++j)
      {
        const auto v = H(i, j);
        if(fabs(v)>1e-7)
          A(i, j) = H(i, j);
        else
          A(i, j) = 0;
      }
    }
  }

  return A;
}

dlib::matrix<double> buildAdjacancyMatrixFrom(const arr& H, uint from, Problem & pb)
{
  dlib::matrix<double> A = dlib::zeros_matrix<double>(H.d0, H.d1);
  pb.xmasks.push_back(intV(H.d0, 0));
  pb.sizes.push_back(0);
  auto & xmask = pb.xmasks.back();
  auto & size = pb.sizes.back();

  std::queue<uint> froms;
  froms.push(from);

  while(!froms.empty())
  {
    auto from = froms.front();
    froms.pop();

    for(auto to = 0; to < H.d1; ++to)
    {
      if(H(from, to) && !A(from, to))
      {
        A(from, to) = A(to, from) = 1;
        froms.push(to);

        if(!xmask[from])
        {
          xmask[from] = 1;
          size++;
        }

        if(!xmask[to])
        {
          xmask[to] = 1;
          size++;
        }
      }
    }
  }

  return A;
}

std::vector<dlib::matrix<double>> buildAdjacancyMatrices(const arr & H, std::vector<Problem>& pbs)
{
  std::vector<dlib::matrix<double>> As;
  As.reserve(4);
  pbs.reserve(4);

  bool progressing = true;
  uint from = 0;

  intV all(H.d0);
  while(from < H.d0 && progressing)
  {
    Problem pb;
    dlib::matrix<double> A = buildAdjacancyMatrixFrom(H, from, pb);
    if(pb.sizes[0]) // found supproblem
    {
      As.push_back(A);
      pb.overlaps.push_back(std::set<uint>());
      pbs.push_back(pb);

      // bookeeping on inddices assigned to a pb
      for(auto i = 0; i < H.d0; ++i) // probably possible to improve a lot!!
        if(pb.xmasks.front()[i])
          all[i] = 1;
    }
    else
    {
      all[from] = -1;
    }

    // advance from to possible start of next cluster
    auto prevFrom = from;
    for(from = 0; all[from] != 0 && from < all.size(); ++from) // probably possible to improve a lot!!
    {}

    progressing = from > prevFrom;
  }

  return As;
}

Problem buildDecomposition(const dlib::matrix<double>& A, const Problem& orig, std::vector<unsigned long> & sparsestCut, uint numberOfCluster)
{
  Problem pb;
  pb.xmasks = std::vector<intV>(numberOfCluster);
  pb.sizes = std::vector<uint>(numberOfCluster, 0);
  pb.overlaps = std::vector<std::set<uint>>(numberOfCluster);

  for(auto & xmask: pb.xmasks)
    xmask = intV(sparsestCut.size(), 0);

  for(auto i = 0; i < sparsestCut.size(); ++i) // quadratic here!!
  {
    if(orig.xmasks.front()[i]) // test needed ?
    {
      const auto & k = sparsestCut[i];
      pb.xmasks[k][i] = 1;
      pb.sizes[k]++;

      for(auto j = 0; j < A.nc(); ++j)
      {
        if(orig.xmasks.front()[j]) // test needed ?
        {
          if(A(i, j) != 0 && sparsestCut[j] != k) // add the neighbors in other cut! crucial part for ADMM
          {
            if(!pb.xmasks[k][j])
            {
              pb.xmasks[k][j] = 1;
              pb.sizes[k]++;
              pb.overlaps[k].insert(j);
            }
          }
        }
      }
    }
  }

  return pb;
}

Decomposition decomposeHessian(const arr& H, uint splittingThreshold, uint numberOfCluster)
{
  CHECK_EQ(H.d0, H.d1, "hessian should be a square matrix");
  CHECK(!isSparseMatrix(H), "should NOT be a sparse matrix");

  Decomposition decomp;
  decomp.problems.reserve(10);

  std::vector<Problem> independant_pbs;
  auto As = buildAdjacancyMatrices(H, independant_pbs);

  for(auto i = 0; i < As.size(); ++i)
  {
    const auto& A = As[i];
    auto& pb = independant_pbs[i];

    //std::cout << A << std::endl;

    if(pb.sizes.front() > splittingThreshold)
    {
      auto sparsestCut = spectralCluster(A, numberOfCluster);
      auto splitted = buildDecomposition(A, pb, sparsestCut, numberOfCluster);

      decomp.problems.push_back(splitted);
    }
    else
    {
      decomp.problems.push_back(pb);
    }
  }

  CHECK_EQ(true, sanityCheck(H, decomp), "wrong size computations")

  return decomp;
}

Decomposition decomposeSparseHessian(const arr& H, uint splittingThreshold, uint numberOfCluster) // to be improved
{
  CHECK(isSparseMatrix(H), "should be a sparse matrix");

  auto HH = dynamic_cast<rai::SparseMatrix*>(H.special)->unsparse();

  return decomposeHessian(HH, splittingThreshold, numberOfCluster);
}

}
