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
   \file saiTime.cpp
   \author Luis Sentis
*/

#include <wbc/util/saiTime.hpp>

namespace wbc {

  Time::Time() {
    time_.tv_sec = 0;
    time_.tv_usec = 0;
  }

  Time::Time( timeval time ) 
    : time_( time )
  {
  }

  Time::Time( long sec, long usec )
  {
    time_.tv_sec = sec;
    time_.tv_usec = usec;
  }

  Time& 
  Time::operator=( const Time& time ) {
    time_ = time.time_;
    return *this;
  }

  Time& 
  Time::operator=( const timeval& time ) {
    time_ = time;
    return *this;
  }

  bool 
  Time::operator<( const Time& time ) const {
    if ( time_.tv_sec < time.value().tv_sec )
      return true;

    else if ( time_.tv_sec == time.value().tv_sec )
      return time_.tv_usec < time.value().tv_usec;

    return false;
  }

  bool 
  Time::operator>( const Time& time ) const {
    if ( time_.tv_sec > time.value().tv_sec )
      return true;

    else if ( time_.tv_sec == time.value().tv_sec )
      return time_.tv_usec > time.value().tv_usec;

    return false;
  }

  bool 
  Time::operator<=( const Time& time ) const {
    if ( time_.tv_sec < time.value().tv_sec )
      return true;

    else if ( time_.tv_sec == time.value().tv_sec )
      return time_.tv_usec <= time.value().tv_usec;

    return false;
  }

  bool
  Time::operator>=( const Time& time ) const {
    if ( time_.tv_sec > time.value().tv_sec )
      return true;

    else if ( time_.tv_sec == time.value().tv_sec )
      return time_.tv_usec >= time.value().tv_usec;

    return false;
  }

  bool
  Time::operator==( const Time& time ) const {
    if (( time_.tv_sec == time.value().tv_sec )
	&& ( time_.tv_usec == time.value().tv_usec) )
      return true;
    else
      return false;
  }

  bool
  Time::operator!=( const Time& time ) const {
    if (( time_.tv_sec != time.value().tv_sec )
	|| ( time_.tv_usec != time.value().tv_usec) )
      return true;
    else
      return false;
  }

  void  Time::zero(){
    time_.tv_sec = 0;
    time_.tv_usec = 0;
  }


  const timeval& Time::value() const 
  {
    return time_;
  }


  timeval& Time::value()
  {
    return time_;
  }

  Time 
  Time::operator+( const Time& val ) const {

    timeval ans; ans.tv_usec = 0; ans.tv_sec = 0;
    timeval obj = time_;
    timeval inc = val.value();

    ans.tv_sec += ( obj.tv_usec + inc.tv_usec ) / 1000000;
    ans.tv_sec += inc.tv_sec;
    ans.tv_usec = ( obj.tv_usec + inc.tv_usec ) % 1000000;

    return Time( ans );
  }

  void
  Time::operator+=( const Time& val ) {

    timeval ans = time_;
    timeval inc = val.value();

    ans.tv_sec += ( time_.tv_usec + inc.tv_usec ) / 1000000;
    ans.tv_usec = ( time_.tv_usec + inc.tv_usec ) % 1000000;
    ans.tv_sec += inc.tv_sec;

    time_ = ans;
  }

  Time
  Time::operator-( const Time& val ) const {

    timeval ans; ans.tv_usec = 0; ans.tv_sec = 0;
    timeval obj = time_;
    timeval dec = val.value();

    if( obj.tv_sec >= dec.tv_sec ) {

      if( obj.tv_usec >= dec.tv_usec )
        ans.tv_usec = obj.tv_usec - dec.tv_usec;
      else {
        obj.tv_sec -= 1;
        ans.tv_usec = obj.tv_usec + 1000000 - dec.tv_usec;
      }
    }

    else {

      if( obj.tv_usec <= dec.tv_usec )
        ans.tv_usec = obj.tv_usec - dec.tv_usec;
      else {
        obj.tv_sec -= 1;
        ans.tv_usec = obj.tv_usec - dec.tv_usec - 1000000;
      }
    }

    ans.tv_sec = obj.tv_sec - dec.tv_sec;

    return Time(ans);
  }

  void
  Time::operator-=( const Time& val ) {

    timeval dec = val.value();

    if( time_.tv_sec >= dec.tv_sec ) {

      if( time_.tv_usec >= dec.tv_usec )
        time_.tv_usec = time_.tv_usec - dec.tv_usec;
      else {
        time_.tv_sec -= 1;
        time_.tv_usec = time_.tv_usec + 1000000 - dec.tv_usec;
      }
    }

    else {

      if( time_.tv_usec <= dec.tv_usec )
        time_.tv_usec = time_.tv_usec - dec.tv_usec;
      else {
        time_.tv_sec -= 1;
        time_.tv_usec = time_.tv_usec - dec.tv_usec - 1000000;
      }
    }

    time_.tv_sec = time_.tv_sec - dec.tv_sec;
  }

  Time 
  Time::operator*( int mul ) const {
    timeval mulTime;
    long t = time_.tv_usec * mul;
    mulTime.tv_usec = t % 1000000;
    mulTime.tv_sec = time_.tv_sec * mul + t / 1000000;
    return Time( mulTime );
  }

  void
  Time::operator*=( int mul ) {
    long t = time_.tv_usec * mul;
    time_.tv_usec = t % 1000000;
    time_.tv_sec *= mul;
    time_.tv_sec += t / 1000000;
  }

  Time 
  Time::operator/( int div ) const {
    timeval divTime;
    long t = time_.tv_sec % div;
    divTime.tv_sec = time_.tv_sec / div;
    divTime.tv_usec = ( t * 1000000 + time_.tv_usec ) / div;
    return Time( divTime );
  }

  void
  Time::operator/=( int div ) {
    long t = time_.tv_sec % div;
    time_.tv_sec /= div;
    time_.tv_usec = ( t * 1000000 + time_.tv_usec ) / div;
  }

}
