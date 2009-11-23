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
  \file       RecorderImpl.cpp
*/
//===========================================================================

#include <wbc/util/RecorderImpl.hpp>
#include <iostream>
#include <set>
#include <stdlib.h>
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

using namespace std;

namespace wbc {
  
  // at 1kHz we would like to record 20s...
  size_t const RecorderImpl::n_entries_(20000);
  
  static std::set<Recorder*> instances;
  
  Recorder::Recorder()
  {
    instances.insert(this);
  }


  Recorder::~Recorder()
  {
    instances.erase(this);
  }


  void Recorder::FlushAll()
  {
    for (std::set<Recorder*>::const_iterator ir(instances.begin());
	 ir != instances.end(); ++ir) {
      if (Record == (*ir)->state())
	(*ir)->state(Recorder::WriteToFile);
      (*ir)->state(Recorder::Record);
    }
  }


  RecorderImpl::RecorderImpl( char const * header, char const * filename, Recorder::Mode mode )
    : header_(header)
  {
    state(Recorder::Idle);
    this->mode(mode);
    fileName(filename);
    // Set state.
    state_ = Recorder::Idle;
  }

  RecorderImpl::~RecorderImpl() 
  {
    closeFile();
  }

  void RecorderImpl::closeFile() 
  {
    switch( mode_ ) 
      {
      case Vector :
	writeVectorFile();
	break;       
      case Matrix :
	writeMatrixFile();
	break;
      }
    if (dataFile_.is_open())
      { 
	dataFile_.close();
      }
  }


  void
  RecorderImpl::directory( const char* directoryTree ) 
  {
    directoryTree_ = directoryTree;

    // Find characters with "/" and replace them with "\" for windows
    // compatibility.

    for( size_t i = directoryTree_.find("/",0); i <= directoryTree_.size();
	 i = directoryTree_.find("/",i) ) {

      directoryTree_.replace( i, 1, "\\" );
    }

    // If not existing, put a character "\" at the end of the tree.
    if( directoryTree_.find_last_of( "\\") != directoryTree_.size()-1 )
      directoryTree_.append( "\\" );
  }


  void
  RecorderImpl::fileName( const char* name ) 
  {
    fileName_ = name;
  }


  void
  RecorderImpl::mode( Recorder::Mode mode ) 
  {
    mode_ = mode;

    switch( mode_ ) {

    case Vector:
      // Allocate memory for data.
      timeData_.resize( n_entries_ );   // time and data arrays must be of equal size
      vectorData_.resize( n_entries_ ); 

      // Initialize iterators of data..
      vectorBegin_ = vectorEnd_ = vectorData_.begin();
      timeBegin_ = timeEnd_ = timeData_.begin();

      break;

    case Matrix:

      // Allocate memory for data.
      timeData_.resize( n_entries_ );   // time and data arrays must be of equal size 
      matrixData_.resize( n_entries_ );

      // Initialize iterators for data.
      matrixBegin_ = matrixEnd_ = matrixData_.begin();
      timeBegin_ = timeEnd_ = timeData_.begin();

      break;
    }
  }


  void RecorderImpl::data( const Time& t, const SAIVector& vectorValue ) 
  {
	 if( mode_ != Vector ) {
	  LOG_ERROR (logger, "RecorderImpl::data(): vector version called on wrong mode");
	  return;
	 }

    time_ = t;

    if ( state_ == Recorder::WriteToFile || state_ == Recorder::Idle ) 
      return;

    if ( vectorValue.fEmpty() )
      return;

    // Write data to vector container.
    *vectorEnd_++ = vectorValue;
    *timeEnd_++ = time_;

    // If reach the end of the vector write out
    if ( vectorEnd_ == vectorData_.end() ) 
      {
	switch( mode_ ) 
	  {
	  case Vector :
	    writeVectorFile();
	    break;       
	  case Matrix :
	    writeMatrixFile();
	    break;
	  }
      }
  }

  void RecorderImpl::data( const Time& t, const SAIMatrix& matrixValue ) 
  {
	  if( mode_ != Matrix ) {
	  LOG_ERROR (logger, "RecorderImpl::data(): matrix version called on wrong mode");
	  return;
	 }

    time_ = t;

    if ( state_ == Recorder::WriteToFile || state_ == Recorder::Idle ) return;

    if ( matrixValue.fEmpty() )
      return;

    // Write data to vector container.
    *matrixEnd_++ = matrixValue;
    *timeEnd_++ = time_;


    // If reach the end of the vector write out
    if ( matrixEnd_ == matrixData_.end() ) 
      {
	switch( mode_ ) 
	  {
	  case Vector :
	    writeVectorFile();
	    break;       
	  case Matrix :
	    writeMatrixFile();
	    break;
	  }
      }
  }


  void RecorderImpl::state( Recorder::State state ) 
  {
    state_ = state;
    switch ( state_ ) 
      {
      case Idle :
	break;
      
      case WriteToFile :
	switch( mode_ ) 
	  {
	  case Vector :
	    writeVectorFile();
	    break;
	  case Matrix :
	    writeMatrixFile();
	    break;
	  }
	break;
      
      case Record :
	break;

      default:
	cout << "Error: Not a recorder state" << endl;
      }
  }


  void RecorderImpl::writeVectorFile() 
  {
    // If no data return.
    if ( vectorBegin_ == vectorEnd_ ) 
      return;

    // Open file.
    if (!dataFile_.is_open())
      {
	string path = directoryTree_;
	path += fileName_;
	dataFile_.open( path.c_str() );
	dataFile_.precision( 7 );
	dataFile_ << header_ << "\n";
      }

    // While there are components.
    while ( vectorBegin_ != vectorEnd_ )
      {
	// Write time in miliseconds.
	dataFile_ << timeBegin_->dValue() << "\t";

	//Write size.
	dataFile_ << vectorBegin_->size() << "\t";

	// Write data.
	for( int i = 0; i < vectorBegin_->size(); i++ )
	  dataFile_ << vectorBegin_->elementAt(i) << "\t";

	// End of line
	dataFile_ << endl;

	++vectorBegin_;
	++timeBegin_;
      }

    // Close file.
    // dataFile_.close();
  
    vectorBegin_ = vectorEnd_ = vectorData_.begin();
    timeBegin_ = timeEnd_ = timeData_.begin();
  }


  void RecorderImpl::writeMatrixFile() 
  {
    // If no data return.
    if ( matrixBegin_ == matrixEnd_ ) return;

    // Open file.
    if (!dataFile_.is_open())
      {
	string path = directoryTree_;
	path += fileName_;
	dataFile_.open( path.c_str() );
	dataFile_.precision( 7 );
	dataFile_ << header_ << "\n";
      }
    // While there are components.
    while ( matrixBegin_ != matrixEnd_ ) {

      // Write data.
      for( int i = 0; i < matrixBegin_->row(); i++ ) {

	// Write time in miliseconds.
	dataFile_ << timeBegin_->value().tv_sec +
	  timeBegin_->value().tv_usec / 1e6 << "\t";

	// Write size.
	dataFile_ << matrixBegin_->row() << "\t";
	dataFile_ << matrixBegin_->column() << "\t";

	// Write data.
	for ( int j = 0; j < matrixBegin_->column(); j++ )
	  dataFile_ << matrixBegin_->elementAt(i,j) << "\t";

	// Return line.
	dataFile_ << endl;
      }

      // End of line
      dataFile_ << endl;

      ++matrixBegin_;
      ++timeBegin_;
    }

    // Close file.
    // dataFile_.close();
  
    matrixBegin_ = matrixEnd_ = matrixData_.begin();
    timeBegin_ = timeEnd_ = timeData_.begin();
  }

}
