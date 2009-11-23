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
  \file       Recorder.h
*/
//===========================================================================

#ifndef WBC_RECORDER_H
#define WBC_RECORDER_H

#include <wbc/util/saiTime.hpp>
#include <saimatrix/SAIMatrix.h>
#include <saimatrix/SAIVector.h>

namespace wbc {

  /*!
    \brief This is an interface for recording data. It can record
    several types of algebraic values.
  */

  class Recorder
  {
  public:
    Recorder();
    virtual ~Recorder();
  
    //! Enumerations.
    enum State { Idle, Record, WriteToFile };
    enum Mode { Vector, Matrix };

    //! \brief Write the working directory tree.
    //! Example: "c:/foo/fee/"
    virtual void directory( const char* ) { }
    virtual const char* GetDirectory() { return NULL; }

    //! \brief Write output file name.
    //! Example: "foo.dat"
    virtual void fileName( const char* ) { }

    //! Write operating mode.
    virtual void mode( Recorder::Mode ) { }

    //! Write state.
    virtual void state( Recorder::State ) { }
    virtual Recorder::State state() const { return Idle; }

    //! Write data
    virtual void data( const Time& t, const SAIMatrix &) { }
    virtual void data( const Time& t, const SAIVector &) { }

    // write data to file and close it
    virtual void closeFile()=0;
    
    
    static void FlushAll();
  };

}

#endif

