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
#include <wbcnet/data.hpp>
#include <wbcnet/com.hpp>
#include <stdint.h>
#include <string.h>

#undef DEBUG_SERVO_PROXY
#ifdef DEBUG_SERVO_PROXY
# include <stdio.h>
# define PDEBUG(fmt, arg...) fprintf(stderr, fmt, ## arg)
#else // DEBUG_SERVO_PROXY
# define PDEBUG(fmt, arg...) /*NOP*/
#endif // DEBUG_SERVO_PROXY

namespace {
  
  typedef uint16_t msg_rq_t;
  typedef uint16_t msg_size_t;
  typedef uint8_t msg_bool_t;
  
  enum {
    RQ_GET_INFO,
    RQ_GET_STATE,
    RQ_SELECT_CONTROLLER,
    RQ_SET_GOAL,
    RQ_SET_GAINS,
  };
  
  static msg_size_t packsize_name(std::string const & name);
  static msg_size_t packsize_vector(std::vector<double> const & data);
  static msg_size_t packsize_status(jspace::Status const & status);
  static msg_size_t packsize_info(jspace::ServoInfo const & info);
  static msg_size_t packsize_state(jspace::ServoState const & state);
  
  static bool pack_rq(wbcnet::Buffer & buffer, size_t offset, msg_rq_t rq);
  static bool pack_name(wbcnet::Buffer & buffer, size_t offset, std::string const & name);
  static bool pack_vector(wbcnet::Buffer & buffer, size_t offset, std::vector<double> const & data);
  static bool pack_status(wbcnet::Buffer & buffer, size_t offset, jspace::Status const & status);
  static bool pack_info(wbcnet::Buffer & buffer, size_t offset, jspace::ServoInfo const & info);
  static bool pack_state(wbcnet::Buffer & buffer, size_t offset, jspace::ServoState const & state);
  
  static bool unpack_name(wbcnet::Buffer const & buffer, size_t offset, std::string & name);
  static bool unpack_vector(wbcnet::Buffer const & buffer, size_t offset, std::vector<double> & data);
  static bool unpack_status(wbcnet::Buffer const & buffer, size_t offset, jspace::Status & status);
  static bool unpack_info(wbcnet::Buffer const & buffer, size_t offset, jspace::ServoInfo & info);
  static bool unpack_state(wbcnet::Buffer const & buffer, size_t offset, jspace::ServoState & state);
  
}


namespace jspace {
  
  
  SleepTransactionPolicy::
  SleepTransactionPolicy(size_t wait_us)
    : wait_us_(wait_us)
  {
  }
  
  
  // Status SleepTransactionPolicy::
  // PreSend()
  // {
  //   Status ok;
  //   return ok;
  // }
  
  
  // Status SleepTransactionPolicy::
  // PostSend()
  // {
  //   Status ok;
  //   return ok;
  // }
  
  
  Status SleepTransactionPolicy::
  WaitReceive()
  {
    usleep(wait_us_);
    Status ok;
    return ok;
  }
  
  
  Status SleepTransactionPolicy::
  PreReceive()
  {
    Status ok;
    return ok;
  }
  
  
  // Status SleepTransactionPolicy::
  // PostReceive()
  // {
  //   Status ok;
  //   return ok;
  // }
  
  
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
    
    if (request.GetSize() < sizeof(msg_rq_t)) {
      mst.ok = false;
      mst.errstr = "truncated request";
      return mst;
    }
    
    wbcnet::Buffer reply(0, -1);
    switch (*reinterpret_cast<msg_rq_t const *>(request.GetData())) {
      
    case RQ_GET_INFO:
      {
	ServoInfo info;
	Status const sst(servo_->getInfo(info));
	if ( ! pack_status(reply, 0, sst)) {
	  mst.ok = false;
	  mst.errstr = "error packing Status";
	  return mst;
	}
	if ( ! pack_info(reply, packsize_status(sst), info)) {
	  mst.ok = false;
	  mst.errstr = "error packing ServoInfo";
	  return mst;
	}
      }
      break;
      
    case RQ_GET_STATE:
      {
	ServoState state;
	Status const sst(servo_->getState(state));
	if ( ! pack_status(reply, 0, sst)) {
	  mst.ok = false;
	  mst.errstr = "error packing Status";
	  return mst;
	}
	if ( ! pack_state(reply, packsize_status(sst), state)) {
	  mst.ok = false;
	  mst.errstr = "error packing ServoState";
	  return mst;
	}
      }
      break;
      
    case RQ_SELECT_CONTROLLER:
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
      
    case RQ_SET_GOAL:
      {
	std::vector<double> goal;
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
      
    case RQ_SET_GAINS:
      {
	std::vector<double> kp;
	if ( ! unpack_vector(request, sizeof(msg_rq_t), kp) ) {
	  mst.ok = false;
	  mst.errstr = "error unpacking kp";
	  return mst;
	}
	std::vector<double> kd;
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
    if ( ! pack_rq(buffer, 0, RQ_GET_INFO)) {
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
    if ( ! unpack_info(buffer, packsize_status(mst), info)) {
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
    if ( ! pack_rq(buffer, 0, RQ_GET_STATE)) {
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
    if ( ! unpack_state(buffer, packsize_status(mst), state)) {
      mst.ok = false;
      mst.errstr = "error unpacking ServoState";
      return mst;
    }
    
    return mst;
  }
  
  
  Status ServoProxyClient::
  selectController(std::string const & name)
  {
    return Status(false, "implement me!");
  }
  
  
  Status ServoProxyClient::
  setGoal(std::vector<double> const & goal)
  {
    return Status(false, "implement me!");
  }
  
  
  Status ServoProxyClient::
  setGains(std::vector<double> const & kp, std::vector<double> const & kd)
  {
    return Status(false, "implement me!");
  }
  
}

namespace {
  
  
  msg_size_t packsize_name(std::string const & name)
  {
    return sizeof(msg_size_t) + name.size();
  }
  
  
  msg_size_t packsize_vector(std::vector<double> const & data)
  {
    return sizeof(msg_size_t) + data.size() * sizeof(double);
  }
  
  
  msg_size_t packsize_status(jspace::Status const & status)
  {
    return sizeof(msg_size_t) + sizeof(msg_bool_t) + status.errstr.size();
  }
  
  
  msg_size_t packsize_info(jspace::ServoInfo const & info)
  {
    msg_size_t packsize(sizeof(msg_size_t));
    // For info.controller_name, we need the overall number of entries
    // first, because each entry has a variable size.
    packsize += sizeof(msg_size_t);
    for (size_t ii(0); ii < info.controller_name.size(); ++ii) {
      packsize += packsize_name(info.controller_name[ii]);
    }
    // Likewise for info.dof_name...
    packsize += sizeof(msg_size_t);
    for (size_t ii(0); ii < info.dof_name.size(); ++ii) {
      packsize += packsize_name(info.dof_name[ii]);
    }
    // But not for info.limit_lower...
    packsize += packsize_vector(info.limit_lower);
    // Nor for info.limit_upper...
    packsize += packsize_vector(info.limit_upper);
    return packsize;
  }
  
  
  msg_size_t packsize_state(jspace::ServoState const & state)
  {
    return sizeof(msg_size_t)
      + packsize_name(state.active_controller)
      + packsize_vector(state.goal)
      + packsize_vector(state.actual)
      + packsize_vector(state.kp)
      + packsize_vector(state.kd);
  }
  
  
  bool pack_rq(wbcnet::Buffer & buffer, size_t offset, msg_rq_t rq)
  {
    if ( ! buffer.Resize(offset + sizeof(msg_rq_t))) {
      return false;
    }
    memcpy(buffer.GetData() + offset, &rq, sizeof(msg_rq_t));
    
    PDEBUG ("pack_rq(%d): %s\n", (int) rq,
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, sizeof(msg_rq_t)).c_str());
    
    return true;
  }
  
  
  bool pack_name(wbcnet::Buffer & buffer, size_t offset, std::string const & name)
  {
    msg_size_t const packsize(packsize_name(name));
    if ( ! buffer.Resize(offset + packsize)) {
      return false;
    }
    memcpy(buffer.GetData() + offset, &packsize, sizeof(packsize));
    memcpy(buffer.GetData() + offset + sizeof(packsize), name.data(), packsize - sizeof(packsize));
    
    PDEBUG ("pack_name(%s): %s\n", name.c_str(),
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool pack_vector(wbcnet::Buffer & buffer, size_t offset, std::vector<double> const & data)
  {
    msg_size_t const packsize(packsize_vector(data));
    if ( ! buffer.Resize(offset + packsize)) {
      return false;
    }
    memcpy(buffer.GetData() + offset, &packsize, sizeof(packsize));
    memcpy(buffer.GetData() + offset + sizeof(packsize), &data[0], packsize - sizeof(packsize));
    
    PDEBUG ("pack_vector(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool pack_status(wbcnet::Buffer & buffer, size_t offset, jspace::Status const & status)
  {
    msg_size_t const packsize(packsize_status(status));
    if ( ! buffer.Resize(offset + packsize)) {
      return false;
    }
    memcpy(buffer.GetData() + offset, &packsize, sizeof(packsize));
    msg_bool_t const ok(status.ok ? 1 : 0);
    memcpy(buffer.GetData() + offset + sizeof(packsize), &ok, sizeof(ok));
    memcpy(buffer.GetData() + offset + sizeof(packsize) + sizeof(ok),
	   status.errstr.data(), packsize - sizeof(packsize) - sizeof(ok));
    
    PDEBUG ("pack_status(%s \"%s\"): %s\n", status.ok ? "OK" : "FAIL", status.errstr.c_str(),
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool pack_info(wbcnet::Buffer & buffer, size_t offset, jspace::ServoInfo const & info)
  {
    msg_size_t const packsize(packsize_info(info));
    if ( ! buffer.Resize(offset + packsize)) {
      return false;
    }
    
    char * buf(buffer.GetData() + offset);
    memcpy(buf, &packsize, sizeof(packsize));
    buf += sizeof(packsize);
    
    msg_size_t nelem(info.controller_name.size());
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    for (size_t ii(0); ii < info.controller_name.size(); ++ii) {
      nelem = info.controller_name[ii].size();
      memcpy(buf, &nelem, sizeof(nelem));
      buf += sizeof(nelem);
      memcpy(buf, info.controller_name[ii].data(), nelem);
      buf += nelem;
    }
    
    nelem = info.dof_name.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    for (size_t ii(0); ii < info.dof_name.size(); ++ii) {
      nelem = info.dof_name[ii].size();
      memcpy(buf, &nelem, sizeof(nelem));
      buf += sizeof(nelem);
      memcpy(buf, info.dof_name[ii].data(), nelem);
      buf += nelem;
    }
    
    nelem = info.limit_lower.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf, &info.limit_lower[0], nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    nelem = info.limit_upper.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf, &info.limit_upper[0], nelem * sizeof(double));
    //buf += nelem * sizeof(double);
    
    PDEBUG ("pack_info(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool pack_state(wbcnet::Buffer & buffer, size_t offset, jspace::ServoState const & state)
  {
    msg_size_t const packsize(packsize_state(state));
    if ( ! buffer.Resize(offset + packsize)) {
      return false;
    }
    
    char * buf(buffer.GetData() + offset);
    memcpy(buf, &packsize, sizeof(packsize));
    buf += sizeof(packsize);
    
    msg_size_t nelem(state.active_controller.size());
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf, state.active_controller.data(), nelem);
    buf += nelem;
    
    nelem = state.goal.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf, &state.goal[0], nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    nelem = state.actual.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf, &state.actual[0], nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    nelem = state.kp.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf, &state.kp[0], nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    nelem = state.kd.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf, &state.kd[0], nelem * sizeof(double));
    //buf += nelem * sizeof(double);
    
    PDEBUG ("pack_state(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool unpack_name(wbcnet::Buffer const & buffer, size_t offset, std::string & name)
  {
    msg_size_t packsize;
    if (buffer.GetSize() < offset + sizeof(packsize)) {
      return false;
    }
    char * buf(buffer.GetData() + offset);
    memcpy(&packsize, buf, sizeof(packsize));
    if (buffer.GetSize() < offset + packsize) {
      return false;
    }
    buf += sizeof(packsize);
    
    msg_size_t nelem;
    memcpy(&nelem, buf, sizeof(nelem));
    if (0 == nelem) {
      name.clear();
      return true;
    }
    buf += sizeof(nelem);
    
    name.resize(nelem);
    name.copy(buf, nelem);
    
    PDEBUG ("unpack_name(%s): %s\n", name.c_str(),
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool unpack_vector(wbcnet::Buffer const & buffer, size_t offset, std::vector<double> & data)
  {
    msg_size_t packsize;
    if (buffer.GetSize() < offset + sizeof(packsize)) {
      return false;
    }
    char * buf(buffer.GetData() + offset);
    memcpy(&packsize, buf, sizeof(packsize));
    if (buffer.GetSize() < offset + packsize) {
      return false;
    }
    buf += sizeof(packsize);
    
    msg_size_t nelem;
    memcpy(&nelem, buf, sizeof(nelem));
    if (0 == nelem) {
      data.clear();
      return true;
    }
    buf += sizeof(nelem);
    
    data.resize(nelem);
    memcpy(&data[0], buf, nelem * sizeof(double));
    
    PDEBUG ("unpack_vector(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool unpack_status(wbcnet::Buffer const & buffer, size_t offset, jspace::Status & status)
  {
    msg_size_t packsize;
    if (buffer.GetSize() < offset + sizeof(packsize)) {
      return false;
    }
    char * buf(buffer.GetData() + offset);
    memcpy(&packsize, buf, sizeof(packsize));
    if (buffer.GetSize() < offset + packsize) {
      return false;
    }
    buf += sizeof(packsize);
    
    msg_bool_t ok;
    if (packsize < sizeof(packsize) + sizeof(ok)) {
      return false;
    }
    memcpy(&ok, buf, sizeof(ok));
    buf += sizeof(ok);
    status.ok = (ok == 0 ? false : true);
    
    msg_size_t const nelem(packsize - sizeof(packsize) - sizeof(ok));
    if (nelem == 0) {
      status.errstr.clear();
      return true;
    }
    
    status.errstr.resize(nelem);
    // the last ", nelem" should not be necessary, but there is a bug
    // in GCC http://gcc.gnu.org/bugzilla/show_bug.cgi?id=43634
    status.errstr.replace(0, nelem, buf, nelem);
    //buf += nelem;
    
    PDEBUG ("unpack_status(%s \"%s\"): %s\n", status.ok ? "OK" : "FAIL", status.errstr.c_str(),
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool unpack_info(wbcnet::Buffer const & buffer, size_t offset, jspace::ServoInfo & info)
  {
    msg_size_t packsize;
    if (buffer.GetSize() < offset + sizeof(packsize)) {
      return false;
    }
    char * buf(buffer.GetData() + offset);
    memcpy(&packsize, buf, sizeof(packsize));
    if (buffer.GetSize() < offset + packsize) {
      return false;
    }
    buf += sizeof(packsize);
    
    info.controller_name.clear();
    msg_size_t nelem;
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    info.controller_name.resize(nelem);
    for (msg_size_t ii(0); ii < nelem; ++ii) {
      msg_size_t nchars;
      memcpy(&nchars, buf, sizeof(nchars));
      buf += sizeof(nchars);
      if (0 < nchars) {
	info.controller_name[ii].resize(nchars);
	info.controller_name[ii].replace(0, nchars, buf, nchars);
	buf += nchars;
      }
    }
    
    info.dof_name.clear();
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    info.dof_name.resize(nelem);
    for (msg_size_t ii(0); ii < nelem; ++ii) {
      msg_size_t nchars;
      memcpy(&nchars, buf, sizeof(nchars));
      buf += sizeof(nchars);
      if (0 < nchars) {
	info.dof_name[ii].resize(nchars);
	info.dof_name[ii].replace(0, nchars, buf, nchars);
	buf += nchars;
      }
    }
    
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    info.limit_lower.resize(nelem);
    memcpy(&info.limit_lower[0], buf, nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    info.limit_upper.resize(nelem);
    memcpy(&info.limit_upper[0], buf, nelem * sizeof(double));
    //buf += nelem * sizeof(double);
    
    PDEBUG ("unpack_info(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool unpack_state(wbcnet::Buffer const & buffer, size_t offset, jspace::ServoState & state)
  {
    msg_size_t packsize;
    if (buffer.GetSize() < offset + sizeof(packsize)) {
      return false;
    }
    char * buf(buffer.GetData() + offset);
    memcpy(&packsize, buf, sizeof(packsize));
    if (buffer.GetSize() < offset + packsize) {
      return false;
    }
    buf += sizeof(packsize);
    
    msg_size_t nelem;
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    state.active_controller.resize(nelem);
    state.active_controller.replace(0, nelem, buf, nelem);
    buf += nelem;
    
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    state.goal.resize(nelem);
    memcpy(&state.goal[0], buf, nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    state.actual.resize(nelem);
    memcpy(&state.actual[0], buf, nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    state.kp.resize(nelem);
    memcpy(&state.kp[0], buf, nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    state.kd.resize(nelem);
    memcpy(&state.kd[0], buf, nelem * sizeof(double));
    //buf += nelem * sizeof(double);
    
    PDEBUG ("unpack_state(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
}
