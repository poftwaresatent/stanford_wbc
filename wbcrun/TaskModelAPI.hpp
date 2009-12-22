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

#ifndef WBCRUN_TASK_MODEL_API_HPP
#define WBCRUN_TASK_MODEL_API_HPP

#include <wbcrun/TaskModelContainer.hpp>
#include <wbcnet/TaskAtomizer.hpp>

namespace wbcrun {
  
  
  class TaskModelAPI
    : public wbcnet::TaskAtomizer
  {
  public:
    explicit inline TaskModelAPI(wbcnet::endian_mode_t endian_mode)
      : wbcnet::TaskAtomizer(endian_mode) {}
    
    /**
       Table for selecting from the task-independent, the
       set-dependent, or the task-dependent set of matrices:
       <table>
       <tr><th>  setID</th><th> taskID</th><th>matrixID</th><th>selected set</th></tr>
       <tr><td>     -1</td><td>     -1</td><td> 0...A-1</td><td>task-independent matrices</td></tr>
       <tr><td>0...B-1</td><td>     -1</td><td> 0...C-1</td><td>set-dependent matrices</td></tr>
       <tr><td>ignored</td><td>0...D-1</td><td> 0...E-1</td><td>task-dependent matrices</td></tr>
       </table>
       Where
       - A is the number of task-independent matrices (e.g. mass inertia)
       - B is the number of states (a.k.a task sets) in the behavior
       - C is the number of set-dependent matrices in each set (e.g. support null space)
       - D is the TOTAL number of tasks in all states of the behavior
       - E is the number of task-dependent matrices in each task (e.g. Lambda star)
       
       \note for \c taskID != -1 the \c setID is ignored, but it is
       prudent to set it to something other than -1 in order to not
       depend on the selection logic. <em>Suggestion:</em> always set
       \c setID to zero when accessing task-dependent matrices.
       
       \return the matching matrix or null if it does not exist.
    */
    virtual wbcnet::MatrixStorageAPI * GetMatrix(ssize_t setID,
						 ssize_t taskID,
						 size_t matrixID) const = 0;
    
    /**
       See the table explaining the matrix selection in GetMatrix().
       
       \return the name of the matching matrix, or "(no such matrix)"
       if it does not exist.
    */
    virtual std::string const & GetName(ssize_t setID,
					ssize_t taskID,
					size_t matrixID) const = 0;
  };
  
  
  template<typename matrix_type, typename traits_type = task_model_container_default_traits>
  class TaskModel
    : public TaskModelAPI
  {
  protected:
    typedef TaskModelContainer<matrix_type, traits_type> container_t;
    typedef typename container_t::matrix_t matrix_t;
    typedef typename container_t::matrix_array_t matrix_array_t;
    
    container_t m_container;

    matrix_t * AddIndepMx(std::string const & name)
    { return m_container.AddIndepMx(name); }
    
    matrix_array_t * AddSetMx(std::string const & name, size_t initial_size)
    { return m_container.AddSetMx(name, initial_size); }
    
    matrix_array_t * AddTaskMx(std::string const & name, size_t initial_size)
    { return m_container.AddTaskMx(name, initial_size); }
    
  public:
    explicit inline TaskModel(wbcnet::endian_mode_t endian_mode)
      : TaskModelAPI(endian_mode) {}
    
    virtual wbcnet::MatrixStorageAPI * GetMatrix(ssize_t setID,
						 ssize_t taskID,
						 size_t matrixID) const
    { return m_container.GetMatrix(setID, taskID, matrixID); }
    
    virtual std::string const & GetName(ssize_t setID,
					ssize_t taskID,
					size_t matrixID) const 
    { return m_container.GetName(setID, taskID, matrixID); }
    
  };
  
}

#endif // WBCRUN_TASK_MODEL_API_HPP
