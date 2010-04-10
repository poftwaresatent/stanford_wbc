/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2008-2010 Stanford University
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

#ifndef JSPACE_ROBOT_PROXY_HPP
#define JSPACE_ROBOT_PROXY_HPP

#include <jspace/RobotAPI.hpp>

namespace wbcnet {
  class Channel;
}

namespace jspace {
  
  /** \note See proxy_util.hpp, especially its subclass
      jspace::RobotTransactionPolicy. */
  class TransactionPolicy;
  
  class RobotProxyServer
  {
  public:
    RobotProxyServer();
    virtual ~RobotProxyServer();
    
    void reset();
    
    Status init(RobotAPI * robot, bool own_robot,
		wbcnet::Channel * channel, bool own_channel);
    
    Status handle();
    
  protected:
    RobotAPI * robot_;
    bool own_robot_;
    wbcnet::Channel * channel_;
    bool own_channel_;
  };
  
  
  class RobotProxyClient
    : public RobotAPI
  {
  public:
    RobotProxyClient();
    virtual ~RobotProxyClient();
    
    void reset();
    
    Status init(wbcnet::Channel * channel, bool own_channel,
		TransactionPolicy * tpol, bool own_tpol);
    
    virtual Status readState(State & state);
    
    virtual Status writeCommand(std::vector<double> const & command);
    
    virtual void shutdown();
    
  protected:
    wbcnet::Channel * channel_;
    bool own_channel_;
    TransactionPolicy * tpol_;
    bool own_tpol_;
  };
  
}

#endif // JSPACE_ROBOT_PROXY_HPP
