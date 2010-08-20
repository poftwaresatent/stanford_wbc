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

#include "proxy_util.hpp"
#include <string.h>

#ifdef WIN32
# define PDEBUG(whatever) /*NOP*/
# pragma warning (disable : 4002)
# pragma warning (disable : 4003)
#else // WIN32
# undef DEBUG_PROXY_UTIL
# ifdef DEBUG_PROXY_UTIL
#  include <stdio.h>
#  define PDEBUG(fmt, arg...) fprintf(stderr, fmt, ## arg)
# else // DEBUG_PROXY_UTIL
#  define PDEBUG(fmt, arg...) /*NOP*/
# endif // DEBUG_PROXY_UTIL
#endif // WIN32


namespace jspace {
  
  
  Status RobotTransactionPolicy::
  WaitReceive()
  {
    Status zonk(false, "RobotTransactionPolicy::WaitReceive() should probably never be called");
    return zonk;
  }
  
  
  Status RobotTransactionPolicy::
  PreReceive()
  {
    Status ok;
    return ok;
  }
  
  
  ServoTransactionPolicy::
  ServoTransactionPolicy(size_t wait_us)
    : wait_us_(wait_us)
  {
  }
  
  
  Status ServoTransactionPolicy::
  WaitReceive()
  {
    usleep(wait_us_);
    Status ok;
    return ok;
  }
  
  
  Status ServoTransactionPolicy::
  PreReceive()
  {
    Status ok;
    return ok;
  }
  
  
  msg_size_t packsize_name(std::string const & name)
  {
    return sizeof(msg_size_t) + name.size();
  }
  
  
  msg_size_t packsize_vector(Vector const & data)
  {
    return sizeof(msg_size_t) + data.size() * sizeof(double);
  }
  
  
  msg_size_t packsize_status(jspace::Status const & status)
  {
    return sizeof(msg_size_t) + sizeof(msg_bool_t) + status.errstr.size();
  }
  
  
  msg_size_t packsize_servo_info(jspace::ServoInfo const & info)
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
  
  
  msg_size_t packsize_servo_state(jspace::ServoState const & state)
  {
    return sizeof(msg_size_t)
      + packsize_name(state.active_controller)
      + packsize_vector(state.goal)
      + packsize_vector(state.actual)
      + packsize_vector(state.kp)
      + packsize_vector(state.kd);
  }
  
  
  msg_size_t packsize_state(jspace::State const & state)
  {
    return sizeof(msg_size_t) + 2 * sizeof(msg_time_t)
      + packsize_vector(state.position_)
      + packsize_vector(state.velocity_)
      + packsize_vector(state.force_);
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
  
  
  bool pack_vector(wbcnet::Buffer & buffer, size_t offset, Vector const & data)
  {
    msg_size_t const packsize(packsize_vector(data));
    if ( ! buffer.Resize(offset + packsize)) {
      return false;
    }
    memcpy(buffer.GetData() + offset, &packsize, sizeof(packsize));
    memcpy(buffer.GetData() + offset + sizeof(packsize),
	   &const_cast<Vector &>(data).coeffRef(0),
	   packsize - sizeof(packsize));
    
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
  
  
  bool pack_servo_info(wbcnet::Buffer & buffer, size_t offset, jspace::ServoInfo const & info)
  {
    msg_size_t const packsize(packsize_servo_info(info));
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
    memcpy(buf,
	   &const_cast<jspace::ServoInfo &>(info).limit_lower.coeffRef(0),
	   nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    nelem = info.limit_upper.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf,
	   &const_cast<jspace::ServoInfo &>(info).limit_upper.coeffRef(0),
	   nelem * sizeof(double));
    //buf += nelem * sizeof(double);
    
    PDEBUG ("pack_servo_info(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool pack_servo_state(wbcnet::Buffer & buffer, size_t offset, jspace::ServoState const & state)
  {
    msg_size_t const packsize(packsize_servo_state(state));
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
    memcpy(buf,
	   &const_cast<jspace::ServoState&>(state).goal.coeffRef(0),
	   nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    nelem = state.actual.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf,
	   &const_cast<jspace::ServoState&>(state).actual.coeffRef(0),
	   nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    nelem = state.kp.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf,
	   &const_cast<jspace::ServoState&>(state).kp.coeffRef(0),
	   nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    nelem = state.kd.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf,
	   &const_cast<jspace::ServoState&>(state).kd.coeffRef(0),
	   nelem * sizeof(double));
    //buf += nelem * sizeof(double);
    
    PDEBUG ("pack_servo_state(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool pack_state(wbcnet::Buffer & buffer, size_t offset, jspace::State const & state)
  {
    msg_size_t const packsize(packsize_state(state));
    if ( ! buffer.Resize(offset + packsize)) {
      return false;
    }
    
    char * buf(buffer.GetData() + offset);
    memcpy(buf, &packsize, sizeof(packsize));
    buf += sizeof(packsize);
    
    msg_time_t tt(state.time_sec_);
    memcpy(buf, &tt, sizeof(tt));
    buf += sizeof(tt);
    
    tt = state.time_usec_;
    memcpy(buf, &tt, sizeof(tt));
    buf += sizeof(tt);
    
    msg_size_t nelem(state.position_.size());
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf,
	   &const_cast<jspace::State&>(state).position_.coeffRef(0),
	   nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    nelem = state.velocity_.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf,
	   &const_cast<jspace::State&>(state).velocity_.coeffRef(0),
	   nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    nelem = state.force_.size();
    memcpy(buf, &nelem, sizeof(nelem));
    buf += sizeof(nelem);
    memcpy(buf,
	   &const_cast<jspace::State&>(state).force_.coeffRef(0),
	   nelem * sizeof(double));
    //buf += nelem * sizeof(double);
    
    PDEBUG ("pack_state(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool unpack_name(wbcnet::Buffer const & buffer, size_t offset, std::string & name)
  {
    msg_size_t namelen;
    if (static_cast<size_t>(buffer.GetSize()) < offset + sizeof(namelen)) {
      return false;
    }
    char * buf(buffer.GetData() + offset);
    memcpy(&namelen, buf, sizeof(namelen));
    if (static_cast<size_t>(buffer.GetSize()) < offset + namelen) {
      return false;
    }
    buf += sizeof(namelen);
    namelen -= sizeof(namelen);	// in the message we have the pack length, which includes the length of the length
    
    PDEBUG ("unpack_name(): namelen = %d\n", (int) namelen);
    
    name.resize(namelen);
    name.replace(0, namelen, buf, namelen);
    
    PDEBUG ("unpack_name(%s): %s\n", name.c_str(),
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, namelen + sizeof(namelen)).c_str());
    
    return true;
  }
  
  
  bool unpack_vector(wbcnet::Buffer const & buffer, size_t offset, Vector & data)
  {
    msg_size_t arrlen;
    if (static_cast<size_t>(buffer.GetSize()) < offset + sizeof(arrlen)) {
      return false;
    }
    char * buf(buffer.GetData() + offset);
    memcpy(&arrlen, buf, sizeof(arrlen));
    if (static_cast<size_t>(buffer.GetSize()) < offset + arrlen) {
      return false;
    }
    buf += sizeof(arrlen);
    arrlen -= sizeof(arrlen);
    
    if (0 != arrlen % sizeof(double)) {
      PDEBUG ("unpack_vector(): arrlen %d is not an integer multiple of %d\n", (int) arrlen, sizeof(double));
      return false;
    }
    
    data.resize(arrlen / sizeof(double));
    memcpy(&data.coeffRef(0), buf, arrlen);
    
    PDEBUG ("unpack_vector(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, arrlen + sizeof(arrlen)).c_str());
    
    return true;
  }
  
  
  bool unpack_status(wbcnet::Buffer const & buffer, size_t offset, jspace::Status & status)
  {
    msg_size_t packsize;
    if (static_cast<size_t>(buffer.GetSize()) < offset + sizeof(packsize)) {
      return false;
    }
    char * buf(buffer.GetData() + offset);
    memcpy(&packsize, buf, sizeof(packsize));
    if (static_cast<size_t>(buffer.GetSize()) < offset + packsize) {
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
  
  
  bool unpack_servo_info(wbcnet::Buffer const & buffer, size_t offset, jspace::ServoInfo & info)
  {
    msg_size_t packsize;
    if (static_cast<size_t>(buffer.GetSize()) < offset + sizeof(packsize)) {
      return false;
    }
    char * buf(buffer.GetData() + offset);
    memcpy(&packsize, buf, sizeof(packsize));
    if (static_cast<size_t>(buffer.GetSize()) < offset + packsize) {
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
    
    PDEBUG ("unpack_servo_info(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool unpack_servo_state(wbcnet::Buffer const & buffer, size_t offset, jspace::ServoState & state)
  {
    msg_size_t packsize;
    if (static_cast<size_t>(buffer.GetSize()) < offset + sizeof(packsize)) {
      return false;
    }
    char * buf(buffer.GetData() + offset);
    memcpy(&packsize, buf, sizeof(packsize));
    if (static_cast<size_t>(buffer.GetSize()) < offset + packsize) {
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
    if (0 < nelem) {
      memcpy(&state.goal[0], buf, nelem * sizeof(double));
      buf += nelem * sizeof(double);
    }
    
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    state.actual.resize(nelem);
    if (0 < nelem) {
      memcpy(&state.actual[0], buf, nelem * sizeof(double));
      buf += nelem * sizeof(double);
    }
    
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    state.kp.resize(nelem);
    if (0 < nelem) {
      memcpy(&state.kp[0], buf, nelem * sizeof(double));
      buf += nelem * sizeof(double);
    }
    
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    state.kd.resize(nelem);
    if (0 < nelem) {
      memcpy(&state.kd[0], buf, nelem * sizeof(double));
      //buf += nelem * sizeof(double);
    }
    
    PDEBUG ("unpack_servo_state(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
  
  bool unpack_state(wbcnet::Buffer const & buffer, size_t offset, jspace::State & state)
  {
    msg_size_t packsize;
    if (static_cast<size_t>(buffer.GetSize()) < offset + sizeof(packsize)) {
      return false;
    }
    char * buf(buffer.GetData() + offset);
    memcpy(&packsize, buf, sizeof(packsize));
    if (static_cast<size_t>(buffer.GetSize()) < offset + packsize) {
      return false;
    }
    buf += sizeof(packsize);
    
    msg_time_t tt;
    memcpy(&tt, buf, sizeof(tt));
    buf += sizeof(tt);
    state.time_sec_ = tt;
    
    memcpy(&tt, buf, sizeof(tt));
    buf += sizeof(tt);
    state.time_usec_ = tt;
    
    msg_size_t nelem;
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    state.position_.resize(nelem);
    memcpy(&state.position_[0], buf, nelem * sizeof(double));
    buf += nelem * sizeof(double);
    
    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    state.velocity_.resize(nelem);
    memcpy(&state.velocity_[0], buf, nelem * sizeof(double));
    buf += nelem * sizeof(double);

    memcpy(&nelem, buf, sizeof(nelem));
    buf += sizeof(nelem);
    state.force_.resize(nelem);
    memcpy(&state.force_[0], buf, nelem * sizeof(double));
    //buf += nelem * sizeof(double);
    
    PDEBUG ("unpack_state(): %s\n",
	    wbcnet::hexdump_buffer(buffer.GetData() + offset, packsize).c_str());
    
    return true;
  }
  
}
