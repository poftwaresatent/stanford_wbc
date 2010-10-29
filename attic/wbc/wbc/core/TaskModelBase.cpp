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
   \file TaskModelBase.cpp
   \author Roland Philippsen
*/

#include "TaskModelBase.hpp"
#include <wbc/core/BehaviorDescription.hpp>
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace wbc {


  void TaskModelBase::
  appendJacobians(SAIMatrix & Jlevel, TaskSet::TaskList2D::const_iterator ilevel)
  {
    for (TaskSet::TaskList::const_iterator itask(ilevel->begin());
	 itask != ilevel->end(); ++itask) {
      SAIMatrix const & JTask((*itask)->Jacobian());
      if ((*itask)->taskType() == TaskDescription::Contact_Task || JTask.fEmpty())
	continue;
      Jlevel.appendVertically( JTask );
    }
  }


  TaskModelBase::
  TaskModelBase(wbcnet::endian_mode_t endian_mode)
    : wbcnet::TaskModel<SAIMatrixAPI>(endian_mode)
  {
  }


  void TaskModelBase::
  SetAcquisitionTime(timeval const & tt)
  {
    m_acquisition_time = tt;	// hmm... pretty obscure, goes all the way up to wbcnet::TaskAtomizer
  }


  bool TaskModelBase::
  Reset(int requestID, BehaviorDescription const & behavior)
  {
    m_level_offset.clear();
    int ttntasks(0);
    BehaviorDescription::task_set_vector const & tasksets(behavior.allTaskSets());
    for (size_t its(0); its < tasksets.size(); ++its) {
      m_level_offset.push_back(ttntasks);
      ttntasks += tasksets[its]->numLevels();
    }
    
    if (tasksets.size() > static_cast<size_t>(wbcnet::TaskAtomizer::capacity)) {
      LOG_ERROR (logger,
		 "TaskModelBase::Reset(): number of task sets " << tasksets.size()
		 << " exceeds wbcnet::TaskAtomizer::capacity " << wbcnet::TaskAtomizer::capacity);
      return false;
    }
    m_container.EnsureSetSize(tasksets.size());
    
    if (ttntasks > wbcnet::TaskAtomizer::capacity) {
      LOG_ERROR (logger,
		 "TaskModelBase::Reset(): total number of tasks " << ttntasks
		 << " exceeds wbcnet::TaskAtomizer::capacity " << wbcnet::TaskAtomizer::capacity);
      return false;
    }
    m_container.EnsureTaskSize(ttntasks);
    
    int const ngenmx(m_container.indep_nm.size()); // actually constant throughout process lifetime
    int const nsets(tasksets.size());
    int const nsetmx(m_container.set_nm.size()); // actually constant throughout process lifetime
    int const ntaskmx(m_container.task_nm.size()); // actually constant throughout process lifetime
    wbcnet::TaskAtomizer::status const
      st(wbcnet::TaskAtomizer::Reset(requestID, ngenmx, nsets, nsetmx, ttntasks, ntaskmx));
    if (wbcnet::TaskAtomizer::OK != st) {
      LOG_ERROR (logger,
		 "TaskModelBase::Reset(): wbcnet::TaskAtomizer::Reset() failed: "
		 << wbcnet::TaskAtomizer::StatusString(st) << "\n"
		 << "  requestID: " << (int) requestID << "\n"
		 << "  ngenmx: " << ngenmx << "\n"
		 << "  nsets: " << nsets << "\n"
		 << "  nsetmx: " << nsetmx << "\n"
		 << "  ttntasks: " << ttntasks << "\n"
		 << "  ntaskmx: " << ntaskmx);
      return false;
    }
    return true;
  }


  int TaskModelBase::
  ComputeTaskID(int taskset_idx, int level_idx) const
  {
    if ((0 > taskset_idx) || (static_cast<int>(m_level_offset.size()) <= taskset_idx))
      return -1;
    if (0 > level_idx)
      return -2;
    int res(m_level_offset[taskset_idx] + level_idx);
    if (GetTTNTasks() <= res)
      return -3;
    return res;
  }
  
  
  void TaskModelBase::
  prettyPrint (std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    if ( ! title.empty())
      os << title << "\n";
    std::string const prf2(prefix + "  ");
    std::string const prf3(prf2 + "  ");
    
    if (m_container.indep_mx.empty()) {
      os << prefix << "no task-independent matrices\n";
    }
    else {
      if (1 == m_container.indep_mx.size()) {
	os << prefix << "task-independent: 1 matrix\n";
      }
      else {
	os << prefix << "task-independent: " << m_container.indep_mx.size() << " matrices\n";
      }
      for (size_t jj(0); jj < m_container.indep_mx.size(); ++jj)
	m_container.indep_mx[jj]->prettyPrint(os, prefix + m_container.indep_nm[jj], prf2);
    }
    
    if (m_container.set_mx.empty()) {
      os << prefix << "no set-dependent matrices\n";
    }
    else {
      os << prefix << "set-dependent: "
	 << m_container.set_mx.size() << " matrices in " << GetNSets() << " sets\n";
      for (int ii(0); ii < GetNSets(); ++ii) {
	os << prf2 << "set ID: " << ii << "\n";
	for (size_t jj(0); jj < m_container.set_mx.size(); ++jj)
	  (*m_container.set_mx[jj])[ii]->prettyPrint(os, prf2 + m_container.set_nm[jj], prf3);
      }
    }
    
    if (m_container.task_mx.empty()) {
      os << prefix << "no task-dependent matrices\n";
    }
    else {
      os << prefix << "task-dependent: "
	 << m_container.task_mx.size() << " matrices in " << GetTTNTasks() << " tasks\n";
      for (int ii(0); ii < GetTTNTasks(); ++ii) {
	os << prf2 << "task ID: " << ii << "\n";
	for (size_t jj(0); jj < m_container.task_mx.size(); ++jj)
	  (*m_container.task_mx[jj])[ii]->prettyPrint(os, prf2 + m_container.task_nm[jj], prf3);
      }
    }
  }
  
}
