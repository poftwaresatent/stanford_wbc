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
   \file SAIMatrixAPI.cpp
   \author Roland Philippsen
*/

#include "SAIMatrixAPI.hpp"

namespace wbc {
  
  
  SAIMatrixAPI::
  SAIMatrixAPI(int row, int col)
    : SAIMatrix(row, col)
  {
  }


  SAIMatrixAPI::
  SAIMatrixAPI()
    : SAIMatrix()
  {
  }


  SAIMatrixAPI::
  SAIMatrixAPI(const SAIMatrixAPI & rhs)
    : SAIMatrix(rhs)
  {
  }


  SAIMatrixAPI::
  SAIMatrixAPI(wbcnet::MatrixAPI<double> const & rhs)
    : SAIMatrix(rhs.ElementPointer(), rhs.NRows(), rhs.NColumns() /*, fTranspose*/ )
  {
  }


  SAIMatrixAPI::
  SAIMatrixAPI(const Float* rgVals, int row, int col, bool fTranspose)
    : SAIMatrix(rgVals, row, col, fTranspose)
  {
  }


  SAIMatrixAPI & SAIMatrixAPI::
  operator = (SAIMatrix const & rhs)
  {
    if ((m_row != rhs.row()) || (m_col != rhs.column()))
      setSize(rhs.row(), rhs.column(), false);
    memcpy(m_data, rhs.dataPtr(), m_size * sizeof(Float));
    return *this;
  }


  SAIMatrixAPI & SAIMatrixAPI::
  operator = (SAIVector const & rhs)
  {
    if ((m_row != rhs.size()) || (m_col != 1))
      setSize(rhs.size(), 1, false);
    setColumn(0, rhs);
    return *this;
  }


  int SAIMatrixAPI::
  NRows() const
  {
    return m_row;
  }


  int SAIMatrixAPI::
  NColumns() const
  {
    return m_col;
  }


  bool SAIMatrixAPI::
  SetSize(int nrows, int ncols)
  {
    setSize(nrows, ncols);
    return true;
  }


  char const * SAIMatrixAPI::
  DataPointer() const
  {
    return reinterpret_cast<char const *>(m_data);
  }


  char * SAIMatrixAPI::
  DataPointer()
  {
    return reinterpret_cast<char *>(m_data);
  }


}
