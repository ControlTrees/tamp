#pragma once

#include <Core/array.h>

uint size(const arr& A)
{
  CHECK_EQ(A.d0, A.d1, "");
  return A.d0;
}

uint size(const dlib::matrix<double> & A)
{
  CHECK_EQ(A.nr(), A.nc(), "");
  return A.nr();
}

template< typename T>
void addEdge(uint from, uint to, T & A)
{
  A(from, to) = A(to, from) = 1.0;
};

template<typename T>
void buildOneLooselyCoupledProblem(T & A)
{
  // cluster 0: 0, 1, 4, 5, 8, 9
  // cluster 1: 2, 3, 6, 7, 10, 11, 12

  CHECK(size(A) >= 13, "please build the matrix correctly");

  addEdge(1, 0, A);
  addEdge(3, 2, A);
  addEdge(4, 0, A);
  addEdge(4, 1, A);
  addEdge(5, 0, A);
  addEdge(5, 1, A);
  addEdge(5, 4, A);
  addEdge(6, 2, A);
  addEdge(6, 3, A);
  addEdge(6, 5, A);
  addEdge(7, 2, A);
  addEdge(7, 3, A);
  addEdge(7, 6, A);
  addEdge(8, 4, A);
  addEdge(8, 5, A);
  addEdge(9, 4, A);
  addEdge(9, 5, A);
  addEdge(9, 8, A);
  addEdge(10, 6, A);
  addEdge(10, 7, A);
  addEdge(11, 6, A);
  addEdge(11, 7, A);
  addEdge(11, 10, A);
  addEdge(11, 12, A);
}

arr buildOneLooselyCoupledProblem()
{
  arr H(13, 13);

  buildOneLooselyCoupledProblem(H);

  return H;
}

arr buildTwoIndependantProblems()
{
  arr H(10, 10);

  // cluster 1
  addEdge(0, 1, H);
  addEdge(1, 2, H);
  addEdge(2, 3, H);
  addEdge(3, 4, H);
  addEdge(4, 2, H);

  // cluster 2
  addEdge(5, 6, H);
  addEdge(6, 7, H);
  addEdge(7, 8, H);
  addEdge(8, 9, H);
  addEdge(9, 6, H);

  return H;
}

arr buildOneLooselyCoupledPlusOneIndependantProblem()
{
  arr H(20, 20);

  buildOneLooselyCoupledProblem(H);

  addEdge(13, 14, H);
  addEdge(13, 15, H);
  addEdge(15, 16, H);
  addEdge(14, 16, H);

  // space left in graph, left empty on purpose

  return H;
}

arr buildTwoIndependantProblemsSparse()
{
  auto H = buildTwoIndependantProblems();

  H.special = &H.sparse();

  CHECK(isSparseMatrix(H), "should be a sparse matrix");

  return H;
}
