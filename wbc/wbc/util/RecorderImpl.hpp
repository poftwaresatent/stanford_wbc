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
  \file       RecorderImpl.hpp
*/
//===========================================================================

#ifndef WBC_RECORDER_IMPL_H
#define WBC_RECORDER_IMPL_H

#include <wbc/util/Recorder.hpp>
#include <vector>
#include <fstream>

namespace wbc {

  class RecorderImpl : public Recorder 
  {
  public:

    RecorderImpl( char const * header, char const * filename, Recorder::Mode mode );
    virtual ~RecorderImpl();

    //! Write the working directory tree.
    virtual void directory( const char* );
    virtual const char* GetDirectory() { return directoryTree_.c_str(); }

    //! Write the file name.
    virtual void fileName( const char* );

    //! Write operating mode.
    virtual void mode( Recorder::Mode );

    //! Write state.
    virtual void state( Recorder::State );  
    virtual Recorder::State state() const { return state_; }

    /** Stores data in a circular fashion. */
    virtual void data( const Time& t, const SAIVector &);
    virtual void data( const Time& t, const SAIMatrix &);

    virtual void closeFile();

  private:
    static size_t const n_entries_;

    //! General attributes.
    Recorder::State state_;
    Recorder::Mode mode_;
    std::string fileName_;
    char const * header_;
    std::vector<SAIVector> vectorData_;
    std::vector<SAIMatrix> matrixData_;
    std::vector<Time> timeData_;
    std::ofstream dataFile_;
    std::string directoryTree_;

    //! Indexing attributes.
    std::vector<SAIVector>::iterator vectorBegin_;
    std::vector<SAIVector>::iterator vectorEnd_;
    std::vector<SAIMatrix>::iterator matrixBegin_;
    std::vector<SAIMatrix>::iterator matrixEnd_;
    std::vector<Time>::iterator timeBegin_;
    std::vector<Time>::iterator timeEnd_;

    Time time_;

    /** Can also run in an independent thread. */
    void writeVectorFile();
    void writeMatrixFile();
    void writeStructureFile();
  };

}

#endif

