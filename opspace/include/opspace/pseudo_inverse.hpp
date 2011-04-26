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

#ifndef OPSPACE_PSEUDO_INVERSE_HPP
#define OPSPACE_PSEUDO_INVERSE_HPP

#include <jspace/wrap_eigen.hpp>

namespace opspace {

  using jspace::Matrix;
  using jspace::Vector;
  
  /**
     This pseudo-inverse is based on SVD, followed by threshlding on
     the singular values. This is a bit simplistic, but we have found
     that it works allright for our use cases with a sigmaThreshold of
     1e-4 or 1e-3.
  */
  void pseudoInverse(Matrix const & matrix,
		     double sigmaThreshold,
		     Matrix & invMatrix,
		     Vector * opt_sigmaOut = 0);
  
}

#endif // OPSPACE_PSEUDO_INVERSE_HPP
