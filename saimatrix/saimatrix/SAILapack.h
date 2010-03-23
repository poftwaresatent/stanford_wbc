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

#ifndef SAI_LAPACK_H
#define SAI_LAPACK_H

class SAIMatrix;

/**
   Singular value decomposition, using the LAPACK library.
   <code> A = U * D * V^T </code>
   
   \note the output arguments get resized for you.
   
   \return 0 for success. It is actually the "info" value of \c
   dgesdd(), see its manpage for details
*/
int sai_lapack_svd(SAIMatrix const & A, SAIMatrix & U, SAIMatrix & S, SAIMatrix & V);

/**
   Singular value decomposition followed by thresholding for the
   purpose of computing a stable inverse. It calls sai_lapack_svd(),
   followed by truncation of all values of \c S that lie below \c
   epsilon. \c SInv is set to a square matrix with the same number of
   rows as \c S and its diagonal is filled in with \c 1/S for all
   eigenvalues that are bigger than \c epsilon. The final result is
   <code> Ainv = U * Sinv * U^T </code>.
   
   \note It is okay to pass the same matrix as \c A and \c Ainv
   because the assignment is done at the very end.

   \return the same as sai_lapack_svd().
*/
int sai_lapack_stable_svd_inverse(SAIMatrix const & A, double epsilon, SAIMatrix & Ainv);

#endif // SAI_LAPACK_H
