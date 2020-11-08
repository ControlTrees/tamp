#include <Core/array.h>

#include <HessianDecomposition/hessian_decomposition.h>
#include <HessianDecomposition/utils.h>

#include <gtest/gtest.h>

using namespace hessian_decomposition;

TEST(DLib, UseDLib)
{
  using namespace dlib;

  matrix<double> A(13, 13);

  for(auto i = 0; i < A.nr(); ++i)
    for(auto j = 0; j < A.nc(); ++j)
      A(i, j) = 0.0;

  buildOneLooselyCoupledProblem(A);

  std::cout << A << std::endl;

  // Finally, we can also solve the same kind of non-linear clustering problem with
  // spectral_cluster().  The output is a vector that indicates which cluster each sample
  // belongs to.  Just like with kkmeans, it assigns each point to the correct cluster.,
  std::vector<unsigned long> assignments;
  EXPECT_NO_THROW(assignments = spectralCluster(A, 2));

  std::cout << mat(assignments) << std::endl;
}

TEST(DecomposeHessian, SpectralCluster)
{
  arr H(13, 13);

  buildOneLooselyCoupledProblem(H);

  auto A = buildAdjacancyMatrix(H);
  auto sparsestCut = spectralCluster(A, 2);

  EXPECT_EQ(H.d0, sparsestCut.size());
}

TEST(DecomposeHessian, OneLooselyCoupledProblem)
{
  auto H = buildOneLooselyCoupledProblem();

  auto decomp = decomposeHessian(H, H.d0 / 2 + 1, 2);

  EXPECT_EQ(1, decomp.problems.size());
  EXPECT_EQ(2, decomp.problems.front().xmasks.size());
  EXPECT_EQ(2, decomp.problems.front().sizes.size());
  EXPECT_EQ(7, decomp.problems.front().sizes[0]);
  EXPECT_EQ(8, decomp.problems.front().sizes[1]);
  EXPECT_EQ(1, decomp.problems.front().overlaps[0].size());
  EXPECT_EQ(1, decomp.problems.front().overlaps[1].size());
  EXPECT_EQ(intV({1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0}), decomp.problems.front().xmasks[0]);
  EXPECT_EQ(intV({0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1}), decomp.problems.front().xmasks[1]);
}

TEST(DecomposeHessian, TwoIndependantProblems)
{
  auto H = buildTwoIndependantProblems();

  auto decomp = decomposeHessian(H, H.d0 / 2 + 1, 2);

  EXPECT_EQ(2, decomp.problems.size());
  EXPECT_EQ(intV({1, 1, 1, 1, 1, 0, 0, 0, 0, 0}), decomp.problems[0].xmasks[0]);
  EXPECT_EQ(intV({0, 0, 0, 0, 0, 1, 1, 1, 1, 1}), decomp.problems[1].xmasks[0]);
}

TEST(DecomposeHessian, TwoIndependantProblemsSparse)
{
  auto H = buildTwoIndependantProblemsSparse();

  auto decomp = decomposeSparseHessian(H, H.d0 / 2 + 1, 2);

  EXPECT_EQ(2, decomp.problems.size());
  EXPECT_EQ(intV({1, 1, 1, 1, 1, 0, 0, 0, 0, 0}), decomp.problems[0].xmasks[0]);
  EXPECT_EQ(intV({0, 0, 0, 0, 0, 1, 1, 1, 1, 1}), decomp.problems[1].xmasks[0]);
}


TEST(DecomposeHessian, OneLooselyCoupledPlusOneIndependantProblem)
{
  auto H = buildOneLooselyCoupledPlusOneIndependantProblem();
  auto decomp = decomposeHessian(H, H.d0 / 2 + 1, 2);

  EXPECT_EQ(2, decomp.problems.size());
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

