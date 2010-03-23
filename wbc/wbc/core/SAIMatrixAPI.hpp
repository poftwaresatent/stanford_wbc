/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
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

/**
   \file SAIMatrixAPI.hpp
   \author Roland Philippsen
*/

#ifndef WBC_SAI_MATRIX_API_HPP
#define WBC_SAI_MATRIX_API_HPP

#include <wbcnet/data.hpp>
#include <saimatrix/SAIMatrix.h>

namespace wbc {
  
  /**
     Splint for treating SAIMatrix instances like wbcnet::MatrixAPI<> instances.
  */
  class SAIMatrixAPI
    : public wbcnet::MatrixAPI<Float>,
      public SAIMatrix
  {
  public:
    SAIMatrixAPI(int row, int col);
    SAIMatrixAPI();
    SAIMatrixAPI(const SAIMatrixAPI & rhs);
    SAIMatrixAPI(wbcnet::MatrixAPI<double> const & rhs);
    SAIMatrixAPI(const Float* rgVals, int row, int col, bool fTranspose = false);
  
    // this can probably move up the hierarchy
    SAIMatrixAPI & operator = (SAIMatrix const & rhs);
    SAIMatrixAPI & operator = (SAIVector const & rhs);
  
    virtual int NRows() const;
    virtual int NColumns() const;
    virtual bool SetSize(int nrows, int ncols);
    virtual char const * DataPointer() const;
    virtual char * DataPointer();
  };
  
}

#endif // WBC_SAI_MATRIX_API_HPP
