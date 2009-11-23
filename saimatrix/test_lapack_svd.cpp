// copied and adapted from
// http://www.cs.rochester.edu/~bh/cs400/using_lapack.html
//
// compile like this:
// g++ -Wall -o test_lapack_svd test_lapack_svd.cpp -llapack -lblas

#include <stdio.h>

extern "C" {
  
  void dgesv_(const int *N, const int *nrhs, double *A, const int *lda, int
	      *ipiv, double *b, const int *ldb, int *info);

}

int main(int argc, char ** argv)
{
  /* 3x3 matrix A
   * 76 25 11
   * 27 89 51
   * 18 60 32
   */
  double A[9] = {76, 27, 18, 25, 89, 60, 11, 51, 32};
  double b[3] = {10, 7, 43};
  
  int N = 3;
  int nrhs = 1;
  int lda = 3;
  int ipiv[3];
  int ldb = 3;
  int info;
  
  dgesv_(&N, &nrhs, A, &lda, ipiv, b, &ldb, &info);
  
  if (info == 0) /* succeed */
    printf("The solution is %lf %lf %lf\n", b[0], b[1], b[2]);
  else
    fprintf(stderr, "dgesv_ fails %d\n", info);
  
  return info;
}
