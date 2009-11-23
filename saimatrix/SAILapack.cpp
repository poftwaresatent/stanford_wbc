/*
 * saimatrix -- utility library for stanford-wbc.sourceforge.net
 *
 * Copyright (c) 1997-2008 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#include "SAILapack.h"
#include "SAIMatrix.h"

extern "C" {
  // LAPACK's divide and conquer SVD
  int dgesdd_(char *jobz, int *m, int *n, double *a, int *lda, double *s, 
	      double *u, int *ldu, double *vt, int *ldvt, double *work, int *lwork, 
	      int *iwork, int *info);
}


int sai_lapack_svd(SAIMatrix const & A, SAIMatrix & U, SAIMatrix & S, SAIMatrix & V)
{
  // dimensions
  int mm = A.row();
  int nn = A.column();
  int min_ = min(mm, nn);
  int max_ = max(mm, nn);
    
  // general parameters
  char jobz = 'S';
  int info;
    
  // matrix contents - destroyed
  double * tmp = new double[A.size()];
  for(int i=0; i<mm; i++) {
    for(int j=0; j<nn; j++) {
      tmp[i+j*mm] = A.elementAt(i,j);
    }
  }
  int lda = mm;
    
  // singular values
  int s_size = min_;
  double *s = new double[s_size];
    
  // u matrix
  int u_size = mm * s_size;
  double *u = new double[u_size];
  int ldu = mm;
    
  // vt matrix
  int vt_size = s_size * nn;
  double *vt = new double[vt_size];
  int ldvt = s_size;
    
  // svd workspace
  int lwork = 20*(3*min_*min_ + max(max_,5*min_*min_ + 4*min_));
  double *work = new double[lwork];
  int *iwork = new int[8 * min_];
    
  // LAPACK call
  dgesdd_(&jobz, &mm, &nn, tmp, &lda, s, u, &ldu, vt, &ldvt, work, &lwork, iwork,  &info);
    
  if( (info == 0) && (work[0] > lwork) ) {
    cerr << "For efficiency, increase svd() workspace" << endl;
    cerr << "lwork: " << lwork << "\topt_work: " << work[0] << endl;
  }
    
  // extract S
  S.setSize(min_, min_, true);
  for(int i=0; i<min_; i++) S[i][i] = s[i];
    
  // extract U
  U.setSize(mm, min_);
  for(int i=0; i<mm; i++) {
    for(int j=0; j<min_; j++) {
      U[i][j] = u[i+j*mm];
    }
  }
    
  // extract V
  V.setSize(nn, min_);
  for(int i=0; i<nn; i++) {
    for(int j=0; j<min_; j++) {
      V[i][j] = vt[i*min_ + j];
    }
  }
    
  // memory cleanup
  delete[] tmp;
  delete[] s;
  delete[] u;
  delete[] vt;
  delete[] work;
  delete[] iwork;
    
  // return
  return info;
}


int sai_lapack_stable_svd_inverse(SAIMatrix const & A, double epsilon, SAIMatrix & Ainv)
{
  SAIMatrix U, S, V;
  int const result(sai_lapack_svd(A, U, S, V));
  if (0 != result)
    return result;
  
  int const nrows(S.row());
  SAIMatrix Sinv(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (S[ii][ii] > epsilon) {
      Sinv[ii][ii] = 1.0 / S[ii][ii];
    }
  }
  
  Ainv = U * Sinv * U.transpose();
  
  return 0;
}
