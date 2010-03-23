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
   \file SAIVectorAPI.cpp
   \author Roland Philippsen
*/

#include "SAIVectorAPI.hpp"
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace wbc {

  SAIVectorAPI::
  SAIVectorAPI(int size)
    : SAIVector(size)
  {
  }


  SAIVectorAPI::
  SAIVectorAPI(const SAIVector & rhs)
    : SAIVector(rhs)
  {
  }


  SAIVectorAPI::
  SAIVectorAPI()
    : SAIVector()
  {
  }


  SAIVectorAPI::
  SAIVectorAPI(const Float * rgVals, int size)
    : SAIVector(rgVals, size)
  {
  }


  SAIVectorAPI::
  SAIVectorAPI(const wbcnet::Vector<double> & rhs)
    : SAIVector(rhs.ElementPointer(), rhs.NElements())
  {
  }


  int SAIVectorAPI::
  NElements() const
  {
    return m_size;
  }


  bool SAIVectorAPI::
  SetNElements(int nelem)
  {
    setSize(nelem);
    return true; // SAIVector::setSize() returns void, so assume it always succeeds
  }


  char const * SAIVectorAPI::
  DataPointer() const
  {
    return reinterpret_cast<char const *>(m_data);
  }


  char * SAIVectorAPI::
  DataPointer()
  {
    return reinterpret_cast<char *>(m_data);
  }


  SAIVectorWrap::
  SAIVectorWrap(int _nelem)
    : wrapee(0),
      nelem(_nelem)
  {
  }


  SAIVectorWrap::
  SAIVectorWrap(int _nelem, SAIVector * _wrapee)
    : wrapee(_wrapee),
      nelem(_nelem)
  {
  }


  int SAIVectorWrap::
  NElements() const
  {
    if (wrapee)
      return wrapee->size();
    return nelem;
  }


  bool SAIVectorWrap::
  SetNElements(int nelem)
  {
    if (wrapee)
      wrapee->setSize(nelem);
    this->nelem = nelem;
    return true; // SAIVector::setSize() returns void, so assume it always succeeds
  }


  char const * SAIVectorWrap::
  DataPointer() const
  {
    if (wrapee)
      return reinterpret_cast<char const *>(wrapee->dataPtr());
    LOG_WARN (logger, "wbc::SAIVectorWrap::DataPointer() [const] called without wrapee");
    return 0;
  }


  char * SAIVectorWrap::
  DataPointer()
  {
    if (wrapee)
      return reinterpret_cast<char *>(wrapee->dataPtr());
    LOG_WARN (logger, "wbc::SAIVectorWrap::DataPointer() [non-const] called without wrapee");
    return 0;
  }

}
