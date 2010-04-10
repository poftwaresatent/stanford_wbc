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

#include "robot_proxy.hpp"
#include "proxy_util.hpp"


namespace jspace {
  
  
  RobotProxyServer::
  RobotProxyServer()
    : robot_(0),
      own_robot_(false),
      channel_(0),
      own_channel_(false)
  {
  }
  
  
  RobotProxyServer::
  ~RobotProxyServer()
  {
    reset();
  }
  
  
  void RobotProxyServer::
  reset()
  {
    if (own_robot_) {
      delete robot_;
    }
    robot_ = 0;
    if (own_channel_) {
      delete channel_;
    }
    channel_ = 0;
  }
  
  
  Status RobotProxyServer::
  init(RobotAPI * robot, bool own_robot,
       wbcnet::Channel * channel, bool own_channel)
  {
    Status status;
    
    if ((0 != robot_) || (0 != channel_)) {
      status.ok = false;
      status.errstr = "already initialized";
      return status;
    }
    
    robot_ = robot;
    own_robot_ = own_robot;
    channel_ = channel;
    own_channel_ = own_channel;
    
    return status;
  }
  
  
  Status RobotProxyServer::
  handle()
  {
    Status mst;
    
    if ((0 == robot_) || (0 == channel_)) {
      mst.ok = false;
      mst.errstr = "not initialized";
      return mst;
    }
    
    wbcnet::Buffer request(0, -1);
    wbcnet::com_status cs(channel_->Receive(request));
    if ( wbcnet::COM_OK != cs) {
      if (wbcnet::COM_TRY_AGAIN == cs) {
	return mst;
      }
      mst.ok = false;
      mst.errstr = wbcnet::com_status_str(cs);
      return mst;
    }
    
    if (request.GetSize() < sizeof(msg_rq_t)) {
      mst.ok = false;
      mst.errstr = "truncated request";
      return mst;
    }
    
    wbcnet::Buffer reply(0, -1);
    switch (*reinterpret_cast<msg_rq_t const *>(request.GetData())) {
      
    case RQ_ROBOT_READ_STATE:
      {
	State state;
	Status const sst(robot_->readState(state));
	if ( ! pack_status(reply, 0, sst)) {
	  mst.ok = false;
	  mst.errstr = "error packing Status";
	  return mst;
	}
	if ( ! pack_state(reply, packsize_status(sst), state)) {
	  mst.ok = false;
	  mst.errstr = "error packing State";
	  return mst;
	}
      }
      break;
      
    case RQ_ROBOT_WRITE_COMMAND:
      {
	std::vector<double> command;
	if ( ! unpack_vector(request, sizeof(msg_rq_t), command) ) {
	  mst.ok = false;
	  mst.errstr = "error unpacking command";
	  return mst;
	}
	Status const sst(robot_->writeCommand(command));
	if ( ! pack_status(reply, 0, sst)) {
	  mst.ok = false;
	  mst.errstr = "error packing Status";
	  return mst;
	}
      }
      break;
      
    case RQ_ROBOT_SHUTDOWN:
      {
	robot_->shutdown();
	Status const sst; // always succeeds... but we still send something
	if ( ! pack_status(reply, 0, sst)) {
	  mst.ok = false;
	  mst.errstr = "error packing Status";
	  return mst;
	}
      }
      break;
      
    default:
      {
	Status const sst(false, "protocol error");
	if ( ! pack_status(reply, 0, sst)) {
	  mst.ok = false;
	  mst.errstr = "error packing Status";
	  return mst;
	}
      }
    }
    
    if (0 >= reply.GetSize()) {
      mst.ok = false;
      mst.errstr = "empty reply";
      return mst;
    }
    
    cs = channel_->Send(reply);
    if (wbcnet::COM_OK != cs) {
      // When sending, wbcnet::COM_TRY_AGAIN is also an error... it
      // probably "never" happens though.
      mst.ok = false;
      mst.errstr = wbcnet::com_status_str(cs);
      return mst;
    }
    
    return mst;
  }
  
  
  RobotProxyClient::
  RobotProxyClient()
    : channel_(0),
      own_channel_(false),
      tpol_(0),
      own_tpol_(false)
  {
  }
  
  
  RobotProxyClient::
  ~RobotProxyClient()
  {
    reset();
  }
  
  
  void RobotProxyClient::
  reset()
  {
    if (own_channel_) {
      delete channel_;
    }
    channel_ = 0;
    if (own_tpol_) {
      delete tpol_;
    }
    tpol_ = 0;
  }
  
  
  Status RobotProxyClient::
  init(wbcnet::Channel * channel, bool own_channel, TransactionPolicy * tpol, bool own_tpol)
  {
    Status status;
    
    if ((0 != channel_) || (0 != tpol_)){
      status.ok = false;
      status.errstr = "already initialized";
      return status;
    }
    
    channel_ = channel;
    own_channel_ = own_channel;
    tpol_ = tpol;
    own_tpol_ = own_tpol;
    
    return status;
  }
  
  
  Status RobotProxyClient::
  readState(State & state)
  {
    Status mst;
    
    if (0 == channel_) {
      mst.ok = false;
      mst.errstr = "not initialized";
      return mst;
    }
    
    wbcnet::Buffer buffer(0, -1);
    if ( ! pack_rq(buffer, 0, RQ_ROBOT_READ_STATE)) {
      mst.ok = false;
      mst.errstr = "error packing request code";
      return mst;
    }
    wbcnet::com_status cs(channel_->Send(buffer));
    if (wbcnet::COM_OK != cs) {
      mst.ok = false;
      mst.errstr = wbcnet::com_status_str(cs);
      return mst;
    }
    
    mst = tpol_->PreReceive();
    if ( ! mst) {
      return mst;
    }
    cs = channel_->Receive(buffer);
    if (wbcnet::COM_OK != cs) {
      mst.ok = false;
      mst.errstr = wbcnet::com_status_str(cs);
      return mst;
    }
    
    if ( ! unpack_status(buffer, 0, mst)) {
      mst.ok = false;
      mst.errstr = "error unpacking Status";
      return mst;
    }
    if ( ! unpack_state(buffer, packsize_status(mst), state)) {
      mst.ok = false;
      mst.errstr = "error unpacking State";
      return mst;
    }
    
    return mst;
  }
  
  
  Status RobotProxyClient::
  writeCommand(std::vector<double> const & command)
  {
    Status mst;
    
    if (0 == channel_) {
      mst.ok = false;
      mst.errstr = "not initialized";
      return mst;
    }
    
    wbcnet::Buffer buffer(0, -1);
    if ( ! pack_rq(buffer, 0, RQ_ROBOT_WRITE_COMMAND)) {
      mst.ok = false;
      mst.errstr = "error packing request code";
      return mst;
    }
    if ( ! pack_vector(buffer, sizeof(msg_rq_t), command)) {
      mst.ok = false;
      mst.errstr = "error packing command";
      return mst;
    }
    
    wbcnet::com_status cs(channel_->Send(buffer));
    if (wbcnet::COM_OK != cs) {
      mst.ok = false;
      mst.errstr = wbcnet::com_status_str(cs);
      return mst;
    }
    
    mst = tpol_->PreReceive();
    if ( ! mst) {
      return mst;
    }
    cs = channel_->Receive(buffer);
    if (wbcnet::COM_OK != cs) {
      mst.ok = false;
      mst.errstr = wbcnet::com_status_str(cs);
      return mst;
    }
    
    if ( ! unpack_status(buffer, 0, mst)) {
      mst.ok = false;
      mst.errstr = "error unpacking Status";
      return mst;
    }
    
    return mst;
  }
  
  
  /**
     \todo It would probably make sense to change to superclass
     interface to return a Status, just like the other methods...
  */
  void RobotProxyClient::
  shutdown()
  {
    if (0 == channel_) {
      return;
    }
    
    wbcnet::Buffer buffer(0, -1);
    if ( ! pack_rq(buffer, 0, RQ_ROBOT_WRITE_COMMAND)) {
      return;
    }
    
    wbcnet::com_status cs(channel_->Send(buffer));
    if (wbcnet::COM_OK != cs) {
      return;
    }
    
    Status mst(tpol_->PreReceive());
    if ( ! mst) {
      return;
    }
    cs = channel_->Receive(buffer);
  }
  
}
