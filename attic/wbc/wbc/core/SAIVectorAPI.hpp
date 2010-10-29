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
   \file SAIVectorAPI.hpp
   \author Roland Philippsen
*/

#ifndef WBC_SAI_VECTOR_API_HPP
#define WBC_SAI_VECTOR_API_HPP

#include <wbcnet/data.hpp>
#include <saimatrix/SAIVector.h>

namespace wbc {
  
  /**
     Splint for treating SAIVector instances like wbcnet::VectorAPI<> instances.
  */
  class SAIVectorAPI
    : public wbcnet::VectorAPI<Float>,
      public SAIVector
  {
  public:
    explicit SAIVectorAPI(int size);
    SAIVectorAPI(const SAIVector & rhs);
    explicit SAIVectorAPI(const wbcnet::Vector<double> & rhs);
    SAIVectorAPI();
    SAIVectorAPI(const Float * rgVals, int size);
  
    virtual int NElements() const;
    virtual bool SetNElements(int nelem);
    virtual char const * DataPointer() const;
    virtual char * DataPointer();
  };
  
  
  /** Useful where you can only get hold of a SAIVector but need
      something compatible with wbcnet. Also not bad for initializing
      message sizes without holding actual SAIVectorAPI memory. */
  class SAIVectorWrap
    : public wbcnet::VectorAPI<Float>
  {
  public:
    explicit SAIVectorWrap(int nelem);
    SAIVectorWrap(int nelem, SAIVector * wrapee);
  
    SAIVector * wrapee;
    int nelem;      // fallback if no wrapee
  
    virtual int NElements() const;
    virtual bool SetNElements(int nelem);
    virtual char const * DataPointer() const;
    virtual char * DataPointer();
  };
  
}

#endif // WBC_SAI_VECTOR_API_HPP
