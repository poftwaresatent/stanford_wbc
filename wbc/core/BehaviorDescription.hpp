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
 * \file       BehaviorDescription.hpp
 * \author     Luis Sentis and Roland Philippsen
 */

#ifndef WBC_BEHAVIOR_DESCRIPTION_HPP
#define WBC_BEHAVIOR_DESCRIPTION_HPP

#include <wbcnet/msg/Service.hpp>
#include <wbc/core/TaskSet.hpp>
#include <wbc/util/saiTime.hpp>
#include <wbc/util/Recorder.hpp>
#include <string>
#include <vector>
#include <map>
#include <stdexcept>

#ifndef WIN32
#include <stdint.h>
#else
#include "wbcnet/win32/win32_compat.hpp"
#endif

namespace wbc {
  
  /**
     \brief This interface is used to create robot movements.
     
     A robot movement is list ot task primitives to be performed at a time. 
     This entity forms the basic elements for the states used in the chore 
     description state machines. A chore therefore is a sequence of movements.
  */
  
  class BehaviorDescription
  {
  protected:
    /**
       Subclasses are required to provide a name. This makes it
       straightforward to implement behavior directories / catalogues.
    */
    explicit BehaviorDescription(std::string const & name);
  
  public:

    typedef enum {Null_Mode_Type, Actuation_Type} ModeType;
    typedef enum {Null_Mode_Value, Overactuated_Value, Stabilization_Value} ModeValue;

    virtual ModeValue mode( ModeType ) const {return Null_Mode_Value;}

    typedef std::vector<TaskSet*> task_set_vector;
  
    std::string const name;
    
    /** Hook for initialization at runtime, i.e. from XML files. After
	the creation of a new BehaviorDescription subclass instance,
	this method can get called any number of times, for example
	passing each XML tag with its corresponding value. Subclasses
	can intercept the key-value pairs they are interested in, and
	delegate to their superclass if they do not handle that
	particular key. The base class implementation simply returns
	false.
	
	\note Errors should be signalled by throwing an exception. The
	return value only indicates whether this particular (sub)class
	has handled the value or not.
	
	\return true if you have handled the key-value pair, false otherwise.
    */
    virtual bool handleInit(std::string const & key, std::string const & value) throw(std::runtime_error);
    
    virtual ~BehaviorDescription();

    virtual TaskSet* activeTaskSet() = 0;
    virtual void onUpdate() = 0;
  
    int lookupTaskSetIndex(TaskSet const * taskSet) const;
  
    /** Stores the RobotControlModel pointer and passes it on to
	subclasses by calling loadMovementPrimitives(). During that
	call, subclasses call registerTaskSet() to inform the base class
	of their task sets. */
    void robotControlModel( RobotControlModel * robmodel ) throw(std::runtime_error);
  
    task_set_vector const & allTaskSets() const { return m_task_sets; }

    virtual void record( const Time & time) {}
  
    /**
       Hook for behaviors that need some sort of re-init when they are being activated.
    */
    virtual void reset() {};
  
    /**
       Hook for behaviors that can accept commands, typically sent from
       the user process. For example, the desired end-effector position
       and orientation can be encoded as a 7x1 matrix and passed down to
       an EndEffectorBehavior.
       
       Command requests are divided into three parts:
       - a command ID (see wbcnet::srv_command_t)
       - an input code vector of int32_t
       - an input matrix of double
       
       The result of a command is similarly divided into three parts:
       - a result ID (see wbcnet::srv_result_t) returned by this method
       - an output code vector of int32_t
       - an output matrix of double
       
       Command IDs and other codes are application-defined (see
       e.g. wbcnet/msg/Service.hpp).  The input code and matrix can be
       of any size, their interpretation depends on the commandID and
       how the behavior implements it. The implementing behavior
       subclass is responsible for checking the incoming sizes. In
       case of size mismatches, there are several entries of
       wbcnet::srv_result_t that you can use to signal such an error
       to the caller. For example, in order to set controller gains,
       the code vector could contain a list of gain IDs, and the
       matrix could be a list of values to assign to these gains.
       
       The output code and matrix are optional. In order to store
       values there, implementing subclasses can use the abilities of
       wbcnet::Vector<> and wbcnet::Matrix<>, which are defined in
       wbcnet/data.hpp (several layers of API and implementation
       hierarchy, make sure to check their superclasses).
       
       \note IMPORTANT: The default implementation of this method does
       some simple dispatching based on the commandID parameter. If
       your want to rely on this dispatching in the superclass, make
       sure that your overriding method calls the superclass at some
       point. For example, if the commandID is wbcnet::SRV_KEY_PRESS,
       then handleKey() ends up being called, which is much easier to
       override if you do not want to worry about marshalling of codes
       and matrices.

       For example, if you want to catch cases that are not
       implemented in the superclass, call that first and react only
       if the call is not implemented there:
       \code
       int SomeBehaviorSubclass::handleCommand(...) {
       int result(handleStdCommand(...));
       if (wbcnet::SRV_NOT_IMPLEMENTED != result) {
         return result;
       }
       // do subclass specific handling here...
       return result;
       }
       \endcode
       
       \return A code according to wbcnet::srv_result_t,
       e.g. wbcnet::SRV_SUCCESS for success,
       wbcnet::SRV_NOT_IMPLEMENTED for a missing implementation. See
       wbcnet/msg/Service.hpp for more details.
    */
    virtual int handleCommand(int commandID,
			      wbcnet::srv_code_t const * code_in,
			      wbcnet::srv_matrix_t const * data_in,
			      wbcnet::srv_code_t * code_out,
			      wbcnet::srv_matrix_t * data_out);
    
    virtual void SingularValues( int tasklevel, SAIMatrix const & SingularValues ) {}
  
    /**
       The user pressed a key on the keyboard (ncurses mode), encoded
       as a wbcnet::SRV_KEY_PRESS commandID to handleCommand(). Look
       at <curses.h> for the symbolic names of the key codes. The
       default here in the superclass returns
       wbcnet::SRV_ABSTRACT_METHOD.
     
       ATTENTION if your subclass overrides handleCommand(), then
       handleKey() will NOT get called UNLESS your handleCommand()
       calls the wbc::BehaviorDescription::handleCommand() at some
       point, or you explicitly call handleKey().
       
       \note The CMake build setup (from \c wbc.cmake) detects whether curses are available
       on your system, and sets the HAVE_CURSES preprocessor symbol
       accordingly on the compiler command line. Thus you can use code like this:
       \code
       #ifdef HAVE_CURSES
       # include <curses.h>
       #endif // HAVE_CURSES
       \endcode
       
       \return wbcnet::srv_status_t
    */
    virtual int handleKey(int keycode);
    
    /**
       The user requests a goal change, encoded as a
       wbcnet::SRV_SET_GOAL commandID to handleCommand(). Implementers
       should check the dimension of the vector before using it. The
       default implementation returns wbcnet::SRV_ABSTRACT_METHOD.
       
       \return wbcnet::srv_status_t
     */
    virtual int handleSetGoal(SAIVector const & goal);
    
    /**
       The user requests a change to the gains, encoded as a
       wbcnet::SRV_SET_GAINS commandID to
       handleCommand(). Implementers should check the dimension of the
       vector before using it. The default implementation returns
       wbcnet::SRV_ABSTRACT_METHOD.
       
       \return wbcnet::srv_status_t
     */
    virtual int handleSetGains(SAIVector const & gains);
    
    /**
       The user requests the Jacobian, encoded as a
       wbcnet::SRV_GET_JACOBIAN commandID to
       handleCommand(). Implementers should resize and fill in the
       jacobian parameter. The default implementation returns
       wbcnet::SRV_ABSTRACT_METHOD.
       
       \return wbcnet::srv_status_t
     */
    virtual int handleGetJacobian(SAIMatrix & jacobian);
    
    ////virtual int32_t getState(int32_t & state) const;
  
  protected:
    /** Gets called by robotControlModel(). Subclasses have to
	initialize their task sets in here and call registerTaskSet()
	for each of them, such that clients can use numTotalTaskLevels()
	and allTaskSets() without being aware of the specific behavior
	subclass they are handling. */
    virtual void loadMovementPrimitives( RobotControlModel * robmodel )
      throw(std::runtime_error) = 0;
  
    /** Must be called by subclasses when they initialize the behavior
	state machine during their implementation of
	loadMovementPrimitives(). It updates m_task_sets. */
    void registerTaskSet(TaskSet * ts);
  
    RobotControlModel * robModel() { return m_robModel; }
  
    Recorder * createRecorder(char const * header, char const * filename, Recorder::Mode mode);
    Recorder * getRecorder();

  
  private:
    typedef std::map<TaskSet const *, int> reverse_lookup_t;
  
    RobotControlModel * m_robModel;
  
    /** Updated when subclasses call registerTaskSet(). */
    task_set_vector m_task_sets;
    reverse_lookup_t m_lookup;

    Recorder * recorder_;
  };

}

#endif
