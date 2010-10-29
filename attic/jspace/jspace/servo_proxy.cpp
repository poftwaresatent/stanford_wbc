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

#include "servo_proxy.hpp"
#include "proxy_util.hpp"


namespace jspace {
  
  
  ServoProxyServer::
  ServoProxyServer()
    : servo_(0),
      own_servo_(false),
      channel_(0),
      own_channel_(false)
  {
  }
  
  
  ServoProxyServer::
  ~ServoProxyServer()
  {
    reset();
  }
  
  
  void ServoProxyServer::
  reset()
  {
    if (own_servo_) {
      delete servo_;
    }
    servo_ = 0;
    if (own_channel_) {
      delete channel_;
    }
    channel_ = 0;
  }
  
  
  Status ServoProxyServer::
  init(ServoAPI * servo, bool own_servo,
       wbcnet::Channel * channel, bool own_channel)
  {
    Status status;
    
    if ((0 != servo_) || (0 != channel_)) {
      status.ok = false;
      status.errstr = "already initialized";
      return status;
    }
    
    servo_ = servo;
    own_servo_ = own_servo;
    channel_ = channel;
    own_channel_ = own_channel;
    
    return status;
  }
  
  
  Status ServoProxyServer::
  handle()
  {
    Status mst;
    
    if ((0 == servo_) || (0 == channel_)) {
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
    
    if (static_cast<size_t>(request.GetSize()) < sizeof(msg_rq_t)) {
      mst.ok = false;
      mst.errstr = "truncated request";
      return mst;
    }
    
    wbcnet::Buffer reply(0, -1);
    switch (*reinterpret_cast<msg_rq_t const *>(request.GetData())) {
      
    case RQ_SERVO_GET_INFO:
      {
	ServoInfo info(0, 0);
	Status const sst(servo_->getInfo(info));
	if ( ! pack_status(reply, 0, sst)) {
	  mst.ok = false;
	  mst.errstr = "error packing Status";
	  return mst;
	}
	if ( ! pack_servo_info(reply, packsize_status(sst), info)) {
	  mst.ok = false;
	  mst.errstr = "error packing ServoInfo";
	  return mst;
	}
      }
      break;
      
    case RQ_SERVO_GET_STATE:
      {
	ServoState state(0);
	Status const sst(servo_->getState(state));
	if ( ! pack_status(reply, 0, sst)) {
	  mst.ok = false;
	  mst.errstr = "error packing Status";
	  return mst;
	}
	if ( ! pack_servo_state(reply, packsize_status(sst), state)) {
	  mst.ok = false;
	  mst.errstr = "error packing ServoState";
	  return mst;
	}
      }
      break;
      
    case RQ_SERVO_SELECT_CONTROLLER:
      {
	std::string name;
	if ( ! unpack_name(request, sizeof(msg_rq_t), name) ) {
	  mst.ok = false;
	  mst.errstr = "error unpacking name";
	  return mst;
	}
	Status const sst(servo_->selectController(name));
	if ( ! pack_status(reply, 0, sst)) {
	  mst.ok = false;
	  mst.errstr = "error packing Status";
	  return mst;
	}
      }
      break;
      
    case RQ_SERVO_SET_GOAL:
      {
	Vector goal;
	if ( ! unpack_vector(request, sizeof(msg_rq_t), goal) ) {
	  mst.ok = false;
	  mst.errstr = "error unpacking goal";
	  return mst;
	}
	Status const sst(servo_->setGoal(goal));
	if ( ! pack_status(reply, 0, sst)) {
	  mst.ok = false;
	  mst.errstr = "error packing Status";
	  return mst;
	}
      }
      break;
      
    case RQ_SERVO_SET_GAINS:
      {
	Vector kp;
	if ( ! unpack_vector(request, sizeof(msg_rq_t), kp) ) {
	  mst.ok = false;
	  mst.errstr = "error unpacking kp";
	  return mst;
	}
	Vector kd;
	if ( ! unpack_vector(request, sizeof(msg_rq_t) + packsize_vector(kp), kd) ) {
	  mst.ok = false;
	  mst.errstr = "error unpacking kd";
	  return mst;
	}
	Status const sst(servo_->setGains(kp, kd));
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
  
  
  ServoProxyClient::
  ServoProxyClient()
    : channel_(0),
      own_channel_(false),
      tpol_(0),
      own_tpol_(false)
  {
  }
  
  
  ServoProxyClient::
  ~ServoProxyClient()
  {
    reset();
  }
  
  
  void ServoProxyClient::
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
  
  
  Status ServoProxyClient::
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
  
  
  Status ServoProxyClient::
  getInfo(ServoInfo & info) const
  {
    Status mst;
    
    if (0 == channel_) {
      mst.ok = false;
      mst.errstr = "not initialized";
      return mst;
    }
    
    wbcnet::Buffer buffer(0, -1);
    if ( ! pack_rq(buffer, 0, RQ_SERVO_GET_INFO)) {
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
    
    while (true) {
      mst = tpol_->PreReceive();
      if ( ! mst) {
	return mst;
      }
      cs = channel_->Receive(buffer);
      if (wbcnet::COM_TRY_AGAIN != cs) {
	break;
      }
      mst = tpol_->WaitReceive();
      if ( ! mst) {
	return mst;
      }
    }
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
    if ( ! unpack_servo_info(buffer, packsize_status(mst), info)) {
      mst.ok = false;
      mst.errstr = "error unpacking ServoInfo";
      return mst;
    }
    
    return mst;
  }
  
  
  Status ServoProxyClient::
  getState(ServoState & state) const
  {
    Status mst;
    
    if (0 == channel_) {
      mst.ok = false;
      mst.errstr = "not initialized";
      return mst;
    }
    
    wbcnet::Buffer buffer(0, -1);
    if ( ! pack_rq(buffer, 0, RQ_SERVO_GET_STATE)) {
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
    
    while (true) {
      mst = tpol_->PreReceive();
      if ( ! mst) {
	return mst;
      }
      cs = channel_->Receive(buffer);
      if (wbcnet::COM_TRY_AGAIN != cs) {
	break;
      }
      mst = tpol_->WaitReceive();
      if ( ! mst) {
	return mst;
      }
    }
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
    if ( ! unpack_servo_state(buffer, packsize_status(mst), state)) {
      mst.ok = false;
      mst.errstr = "error unpacking ServoState";
      return mst;
    }
    
    return mst;
  }
  
  
  Status ServoProxyClient::
  selectController(std::string const & name)
  {
    Status mst;
    
    if (0 == channel_) {
      mst.ok = false;
      mst.errstr = "not initialized";
      return mst;
    }
    
    wbcnet::Buffer buffer(0, -1);
    if ( ! pack_rq(buffer, 0, RQ_SERVO_SELECT_CONTROLLER)) {
      mst.ok = false;
      mst.errstr = "error packing request code";
      return mst;
    }
    if ( ! pack_name(buffer, sizeof(msg_rq_t), name)) {
      mst.ok = false;
      mst.errstr = "error packing name";
      return mst;
    }
    
    wbcnet::com_status cs(channel_->Send(buffer));
    if (wbcnet::COM_OK != cs) {
      mst.ok = false;
      mst.errstr = wbcnet::com_status_str(cs);
      return mst;
    }
    
    while (true) {
      mst = tpol_->PreReceive();
      if ( ! mst) {
	return mst;
      }
      cs = channel_->Receive(buffer);
      if (wbcnet::COM_TRY_AGAIN != cs) {
	break;
      }
      mst = tpol_->WaitReceive();
      if ( ! mst) {
	return mst;
      }
    }
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
  
  
  Status ServoProxyClient::
  setGoal(Vector const & goal)
  {
    Status mst;
    
    if (0 == channel_) {
      mst.ok = false;
      mst.errstr = "not initialized";
      return mst;
    }
    
    wbcnet::Buffer buffer(0, -1);
    if ( ! pack_rq(buffer, 0, RQ_SERVO_SET_GOAL)) {
      mst.ok = false;
      mst.errstr = "error packing request code";
      return mst;
    }
    if ( ! pack_vector(buffer, sizeof(msg_rq_t), goal)) {
      mst.ok = false;
      mst.errstr = "error packing goal";
      return mst;
    }
    
    wbcnet::com_status cs(channel_->Send(buffer));
    if (wbcnet::COM_OK != cs) {
      mst.ok = false;
      mst.errstr = wbcnet::com_status_str(cs);
      return mst;
    }
    
    while (true) {
      mst = tpol_->PreReceive();
      if ( ! mst) {
	return mst;
      }
      cs = channel_->Receive(buffer);
      if (wbcnet::COM_TRY_AGAIN != cs) {
	break;
      }
      mst = tpol_->WaitReceive();
      if ( ! mst) {
	return mst;
      }
    }
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
  
  
  Status ServoProxyClient::
  setGains(Vector const & kp, Vector const & kd)
  {
    Status mst;
    
    if (0 == channel_) {
      mst.ok = false;
      mst.errstr = "not initialized";
      return mst;
    }
    
    wbcnet::Buffer buffer(0, -1);
    if ( ! pack_rq(buffer, 0, RQ_SERVO_SET_GAINS)) {
      mst.ok = false;
      mst.errstr = "error packing request code";
      return mst;
    }
    if ( ! pack_vector(buffer, sizeof(msg_rq_t), kp)) {
      mst.ok = false;
      mst.errstr = "error packing kp";
      return mst;
    }
    if ( ! pack_vector(buffer, sizeof(msg_rq_t) + packsize_vector(kp), kd)) {
      mst.ok = false;
      mst.errstr = "error packing kd";
      return mst;
    }
    
    wbcnet::com_status cs(channel_->Send(buffer));
    if (wbcnet::COM_OK != cs) {
      mst.ok = false;
      mst.errstr = wbcnet::com_status_str(cs);
      return mst;
    }
    
    while (true) {
      mst = tpol_->PreReceive();
      if ( ! mst) {
	return mst;
      }
      cs = channel_->Receive(buffer);
      if (wbcnet::COM_TRY_AGAIN != cs) {
	break;
      }
      mst = tpol_->WaitReceive();
      if ( ! mst) {
	return mst;
      }
    }
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
  
}
