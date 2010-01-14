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

//===========================================================================
/*!
  \author     Luis Sentis
  \file       saiTime.hpp
*/
//===========================================================================

#ifndef WBC_SAI_TIME_H
#define WBC_SAI_TIME_H

#ifdef WIN32
#include "extras.h"
#else
#include <sys/time.h>
#endif
#include <time.h>

namespace wbc {

  /*!
    \brief This is an abstraction for a custome Time value.
  */
  class Time {

  public:

    Time();
    Time( timeval );
    Time( long sec, long usec );
    Time& operator=( const Time& );
    Time& operator=( const timeval& );
    bool operator<( const Time& ) const;
    bool operator>( const Time& ) const;
    bool operator<=( const Time& ) const;
    bool operator>=( const Time& ) const;
    bool operator==( const Time& ) const;
    bool operator!=( const Time& ) const;
    Time operator+( const Time& ) const;
    void operator+=( const Time& );
    Time operator-( const Time& ) const;
    void operator-=( const Time& );
    Time operator*( int ) const;
    void operator*=( int );
    Time operator/( int ) const;
    void operator/=( int );
    const timeval& value() const;
    timeval& value();
    inline double dValue() const { return time_.tv_sec +( 1e-6 * time_.tv_usec ); }
    void zero();

  private:

    timeval time_;
  };

}

#endif
