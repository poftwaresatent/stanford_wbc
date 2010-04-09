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

#ifndef JSPACE_PROXY_UTIL_HPP
#define JSPACE_PROXY_UTIL_HPP

#include <jspace/Status.hpp>
#include <jspace/State.hpp>
#include <jspace/ServoAPI.hpp>
#include <wbcnet/data.hpp>
#include <wbcnet/com.hpp>
#include <string>

#ifdef WIN32
# include <wbcnet/win32/win32_compat.hpp>
#else // WIN32
# include <stdint.h>
#endif // WIN32


namespace jspace {
  
  typedef uint16_t msg_rq_t;
  typedef uint16_t msg_size_t;
  typedef uint8_t msg_bool_t;
  typedef uint32_t msg_time_t;
  
  enum {
    RQ_ROBOT_READ_STATE,
    RQ_ROBOT_WRITE_COMMAND,
    RQ_ROBOT_SHUTDOWN,
    RQ_SERVO_GET_INFO,
    RQ_SERVO_GET_STATE,
    RQ_SERVO_SELECT_CONTROLLER,
    RQ_SERVO_SET_GOAL,
    RQ_SERVO_SET_GAINS
  };
  
  msg_size_t packsize_name(std::string const & name);
  msg_size_t packsize_vector(std::vector<double> const & data);
  msg_size_t packsize_status(jspace::Status const & status);
  msg_size_t packsize_servo_info(jspace::ServoInfo const & info);
  msg_size_t packsize_servo_state(jspace::ServoState const & state);
  ////  msg_size_t packsize_state(jspace::State const & state);
  
  bool pack_rq(wbcnet::Buffer & buffer, size_t offset, msg_rq_t rq);
  bool pack_name(wbcnet::Buffer & buffer, size_t offset, std::string const & name);
  bool pack_vector(wbcnet::Buffer & buffer, size_t offset, std::vector<double> const & data);
  bool pack_status(wbcnet::Buffer & buffer, size_t offset, jspace::Status const & status);
  bool pack_servo_info(wbcnet::Buffer & buffer, size_t offset, jspace::ServoInfo const & info);
  bool pack_servo_state(wbcnet::Buffer & buffer, size_t offset, jspace::ServoState const & state);
  ////  bool pack_state(wbcnet::Buffer & buffer, size_t offset, jspace::State const & state);
  
  bool unpack_name(wbcnet::Buffer const & buffer, size_t offset, std::string & name);
  bool unpack_vector(wbcnet::Buffer const & buffer, size_t offset, std::vector<double> & data);
  bool unpack_status(wbcnet::Buffer const & buffer, size_t offset, jspace::Status & status);
  bool unpack_servo_info(wbcnet::Buffer const & buffer, size_t offset, jspace::ServoInfo & info);
  bool unpack_servo_state(wbcnet::Buffer const & buffer, size_t offset, jspace::ServoState & state);
  ////  bool unpack_state(wbcnet::Buffer const & buffer, size_t offset, jspace::State & state);
  
}

#endif // JSPACE_PROXY_UTIL_HPP
