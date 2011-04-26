/*
 * Shared copyright notice and LGPLv3 license statement.
 *
 * Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 * Copyright (C) 2010 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen (Stanford) and Luis Sentis (UT Austin)
 *          http://cs.stanford.edu/group/manips/
 *          http://www.me.utexas.edu/~hcrl/
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

#include <opspace/pseudo_inverse.hpp>
#include <Eigen/LU>
#include <Eigen/SVD>

using namespace std;

namespace opspace {

  void pseudoInverse(Matrix const & matrix,
		     double sigmaThreshold,
		     Matrix & invMatrix,
		     Vector * opt_sigmaOut)
  {
    if ((1 == matrix.rows()) && (1 == matrix.cols())) {
      // workaround for Eigen2
      invMatrix.resize(1, 1);
      if (matrix.coeff(0, 0) > sigmaThreshold) {
	invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
      }
      else {
	invMatrix.coeffRef(0, 0) = 0.0;
      }
      if (opt_sigmaOut) {
	opt_sigmaOut->resize(1);
	opt_sigmaOut->coeffRef(0) = matrix.coeff(0, 0);
      }
      return;
    }
    
    Eigen::SVD<Matrix> svd(matrix);
    // not sure if we need to svd.sort()... probably not
    int const nrows(svd.singularValues().rows());
    Matrix invS;
    invS = Matrix::Zero(nrows, nrows);
    for (int ii(0); ii < nrows; ++ii) {
      if (svd.singularValues().coeff(ii) > sigmaThreshold) {
	invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
      }
    }
    invMatrix = svd.matrixU() * invS * svd.matrixU().transpose();
    if (opt_sigmaOut) {
      *opt_sigmaOut = svd.singularValues();
    }
  }
  
}
