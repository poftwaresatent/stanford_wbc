/*
 * Copyright (c) 2010 Stanford University
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
   \file UserProcess.hpp
   \author Roland Philippsen
*/

#ifndef WBC_USER_PROCESS_HPP
#define WBC_USER_PROCESS_HPP

#include <wbc/bin/Process.hpp>
#include <wbcnet/msg/Service.hpp>
#include <wbc/bin/directory.hpp>
#include <wbcnet/msg/TaskSpec.hpp>

namespace wbcnet {
  class NetConfig;
}

namespace wbc {
  
  /**
     Do not forget to call UserProcess::Cleanup() at exit.
  */
  class UserProcess
    : public Process
  {
  public:
    UserProcess();
    ~UserProcess();
    
    /** Has to be called at exit. */
    static void Cleanup();
    
    virtual bool Step() throw(std::exception);
    virtual int HandleMessagePayload(wbcnet::unique_id_t msg_id);
    
    void Init(wbcnet::NetConfig const & netconf) throw(std::exception);
    void InteractiveGoalRequest();
    void InteractiveKeyPressLoop();
    void XmlRpcLoop();
    
    wbcnet::Channel * m_channel;
    wbcnet::msg::Service m_user_request;
    wbcnet::msg::Service m_user_reply;
    wbcnet::msg::TaskSpec m_task_spec;
    
  protected:
    DirectoryCmdClient * m_directory_client;
    
    listing_t const & GetBehaviorList() const throw(std::exception);
    
  private:
    mutable listing_t m_lazy_behavior_list;
  };
  
}

#endif // WBC_USER_PROCESS_HPP
