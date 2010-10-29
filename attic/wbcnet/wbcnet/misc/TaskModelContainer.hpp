/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef WBCNET_TASK_MODEL_CONTAINER_HPP
#define WBCNET_TASK_MODEL_CONTAINER_HPP

#ifdef WIN32
#include "wbcnet/win32/win32_compat.hpp"
#endif

#include <string>
#include <vector>

namespace wbcnet {
  
  class MatrixStorageAPI;
  
  /**
     Default traits for TaskModelContainer instantiations. By default,
     matrices are deleted at destruction time, and the container
     automatically grows its storage when required.
  */
  struct task_model_container_default_traits {
    static bool delete_matrices() { return true; }
    static bool auto_grow() { return true; }
  };
  
  
  /**
     Container of the actual storage of task model matrices. See
     TaskModelAPI and TaskModel for the task model interface you
     probably actually care about.  There are three types of model
     matrices:
     - Some are only dependent on the robot structure, the robot
       state, and/or some general relation to the environment. These
       are stored in TaskModelContainer::indep_mx. Examples are
       Coriolis and centrifugal effects.
     - Some depend on the task set (a.k.a. behavior state), but not on
       a specific task in the hierarchy. These are stored in
       TaskModelContainer::set_mx.
     - At the finest level of granularity, a matrix might depend on
       the task set and the exact task in the hierarchy. These are
       stored in TaskModelContainer::task_mx.
       
     \note By default, the destructor of TaskModelContainer will
     delete all the matrices for you. You can switch this off by
     passing task_model_container_traits_nodelete as traits_type
     template argument.
   */
  template<typename matrix_type, typename traits_type = task_model_container_default_traits>
  class TaskModelContainer
  {
  public:
    typedef matrix_type matrix_t;
    typedef traits_type traits_t;
    typedef std::vector<matrix_t*> matrix_array_t;
    typedef std::vector<matrix_array_t*> matrix_array2d_t;
    typedef std::vector<std::string> string_array_t;

    matrix_array_t indep_mx;	// [matrix_id]
    mutable matrix_array2d_t set_mx;    // [matrix_id][set_id]
    mutable matrix_array2d_t task_mx;	// [matrix_id][task_id]
    string_array_t indep_nm;
    string_array_t set_nm;
    string_array_t task_nm;
    
    ~TaskModelContainer() {
      if (traits_t::delete_matrices()) {
	typedef typename matrix_array2d_t::iterator m2dit_t;
	typedef typename matrix_array_t::iterator mit_t;
	for (m2dit_t ii(task_mx.begin()); ii != task_mx.end(); ++ii) {
	  for (mit_t jj((*ii)->begin()); jj != (*ii)->end(); ++jj)
	    delete *jj;
	  delete *ii;
	}
	for (m2dit_t ii(set_mx.begin()); ii != set_mx.end(); ++ii) {
	  for (mit_t jj((*ii)->begin()); jj != (*ii)->end(); ++jj)
	    delete *jj;
	  delete *ii;
	}
	for (mit_t ii(indep_mx.begin()); ii != indep_mx.end(); ++ii)
	  delete *ii;
      }
    }
    
    matrix_t * AddIndepMx(std::string const & name) {
      matrix_t * mx(new matrix_t());
      indep_mx.push_back(mx);
      indep_nm.push_back(name);
      return mx;
    }
    
    matrix_array_t * AddSetMx(std::string const & name, size_t initial_size) {
      matrix_array_t * mxa(new matrix_array_t());
      for (size_t ii(0); ii < initial_size; ++ii)
	mxa->push_back(new matrix_t());
      set_mx.push_back(mxa);
      set_nm.push_back(name);
      return mxa;
    }
    
    /** \note Declared \c const so that it can be called from
	GetMatrix() when traits_t::auto_grow() is true. */
    void EnsureSetSize(size_t required_size) const {
      typedef typename matrix_array2d_t::iterator m2dit_t;
      for (m2dit_t ima(set_mx.begin()); ima != set_mx.end(); ++ima) {
	matrix_array_t & ma(**ima);
	while (ma.size() < required_size) {
	  ma.push_back(new matrix_t());
	}
      }
    }
    
    matrix_array_t * AddTaskMx(std::string const & name, size_t initial_size) {
      matrix_array_t * mxa(new matrix_array_t());
      for (size_t ii(0); ii < initial_size; ++ii)
	mxa->push_back(new matrix_t());
      task_mx.push_back(mxa);
      task_nm.push_back(name);
      return mxa;
    }
    
    /** \note Declared \c const so that it can be called from
	GetMatrix() when traits_t::auto_grow() is true. */
    void EnsureTaskSize(size_t required_size) const {
      typedef typename matrix_array2d_t::iterator m2dit_t;
      for (m2dit_t ima(task_mx.begin()); ima != task_mx.end(); ++ima) {
	matrix_array_t & ma(**ima);
	while (ma.size() < required_size) {
	  ma.push_back(new matrix_t());
	}
      }
    }
    
    virtual matrix_t * GetMatrix(ssize_t setID,
				 ssize_t taskID,
				 size_t matrixID) const {
      if (-1 == taskID) {
	
	// either task-independent...
	if (-1 == setID) {
	  if (indep_mx.size() > matrixID)
	    return indep_mx[matrixID];
	  return 0;
	}
	
	// ...or set-dependent
	if (set_mx.size() > matrixID) {
	  matrix_array_t & ma(*set_mx[matrixID]);
	  if (ma.size() > static_cast<size_t>(setID))
	    return ma[setID];
	  if (traits_t::auto_grow()) {
	    EnsureSetSize(setID + 1);
	    return ma[setID];
	  }
	}
	return 0;
	
      }
      
      // task-dependent
      if (task_mx.size() > matrixID) {
	matrix_array_t & ma(*task_mx[matrixID]);
	if (ma.size() > static_cast<size_t>(taskID))
	  return ma[taskID];
	if (traits_t::auto_grow()) {
	  EnsureTaskSize(taskID + 1);
	  return ma[taskID];
	}
      }
      return 0;
    }
    
    virtual std::string const & GetName(ssize_t setID,
					ssize_t taskID,
					size_t matrixID) const {
      static std::string oops("(no such matrix)");
      if (-1 == taskID) {
	
	// either task-independent...
	if (-1 == setID) {
	  if (indep_nm.size() > matrixID)
	    return indep_nm[matrixID];
	  return oops;
	}
	
	// ...or set-dependent
	if (set_nm.size() > matrixID)
	  return set_nm[matrixID];
	return oops;
      }
      
      // task-dependent
      if (task_nm.size() > matrixID)
	return task_nm[matrixID];
      return oops;
    }
  };
  
}

#endif // WBCNET_TASK_MODEL_CONTAINER_HPP
