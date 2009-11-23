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

#include <wbc/core/TaskSet.hpp>
#include <wbc/util/saiTime.hpp>
#include <wbc/util/Recorder.hpp>
#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <stdint.h>

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
     
       ATTENTION there is some historic weirdness in this method and how
       it interacts with handleStdCommand(). See also in particular
       handleKey().
     
       Commands are divided into two parts, a code vector and a
       matrix. The code vector contains at least one element (the
       command ID), the matrix can be any size (up to some maximum) or
       even empty. The role of the code vector is to tell the behavior
       what the command is about, and the matrix contains the data
       required to execute the command. For example, in order to set
       controller gains, the code vector could contain a list of gain
       IDs, and the matrix could be a list of values to assign to these
       gains.
     
       Command IDs and other codes are application-defined (see
       e.g. wbcrun/service.hpp). The interpretation of the extra codes
       (beyond codeVector[0], if nCodes>1) and matrix parameters depends
       on the command ID. It is up to the imlementing subclass to check
       for correct sizes.
     
       \note The default implementation just calls handleStdCommand(),
       which tries some standard dispatching to
       e.g. handleKey(). Subclasses that want to add functionality while
       retaining the standard commands should call handleStdCommand()
       within their implementation of handleCommand().
     
       \return An application-specific status (see
       e.g. wbcrun/service.hpp).
    */
    virtual int32_t handleCommand(int32_t const * codeVector,
				  size_t nCodes,
				  SAIMatrix const & matrix);
  
    /**
       Attempts to interpret the codeVector according to some "standard"
       commands. If it finds a matching code, it will delegate to the
       corresponding specific method. If it does not find a match, it
       will return wbcrun::srv::NOT_IMPLEMENTED. This can be used
       e.g. by subclasses to attempt standard commands, and then refine
       the call in case it fail, like this:
       \code
       int32_t SomeBehaviorSubclass::handleCommand(...) {
       int32_t result(handleStdCommand(...));
       if (wbcrun::srv::NOT_IMPLEMENTED != result)
       return result;
       // do subclass specific handling here...
       return result;
       }
       \endcode
     
       ATTENTION there is some historical weirdness here, which might
       get refactored away some day: handleCommand() calls
       handleStdCommand(), which calls e.g. handleKey(). But
       handleCommand() is overrideable, so if subclass implementors
       forget to call handleStdCommand() from within their
       handleCommand(), then e.g. handleKey() never gets called.
     
       Currently recognized commands are:
       - wbcrun::srv::KEY_PRESS -- call handleKey(codeVector[1]) if nCodes > 1
     
       \todo add: get and/or set (state, task type, dimension, prog
       gain, diff gain, max vel, max acc, goal, current)
    */
    int32_t handleStdCommand(int32_t const * codeVector,
			     size_t nCodes,
			     SAIMatrix const & matrix);
  
    virtual void SingularValues( int tasklevel, SAIMatrix const & SingularValues ) {}
  
    /**
       The user pressed a key on the keyboard (ncurses mode). Look at
       <curses.h> for the symbolic names of the key codes.
     
       ATTENTION if your subclass overrides handleCommand(), then
       handleKey() will NOT get called UNLESS your handleCommand() calls
       handleStdCommand() at some point (or you explicitly call
       handleKey()). This is some historic weirdness: the servo process
       calls handleCommand(), which calls handleStdCommand(), which
       calls handleKey().
     
       \note Default implementation returns wbcrun::srv::NOT_IMPLEMENTED.
     
       \note The CMake build setup (from \c wbc.cmake) detects whether curses are available
       on your system, and sets the HAVE_CURSES preprocessor symbol
       accordingly on the compiler command line. Thus you can use code like this:
       \code
       #ifdef HAVE_CURSES
       # include <curses.h>
       #endif // HAVE_CURSES
       \endcode
     
       \return one of wbcrun::srv::status_t
    */
    virtual int32_t handleKey(int32_t keycode);

    // int32_t handleSetGoal(SAIMatrix const & matrix);

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
