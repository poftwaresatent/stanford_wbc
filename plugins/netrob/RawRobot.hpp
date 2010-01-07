/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file plugins/netrob/RawRobot.hpp
   \author Roland Philippsen
*/

#ifndef NETROB_RAW_ROBOT_HPP
#define NETROB_RAW_ROBOT_HPP

#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/RobotFactory.hpp>
#include <wbcnet/SockWrap.hpp>


namespace netrob {
  
  
  template<typename value_t>
  struct raw_state_s
  {
    typedef value_t value_type;
    
    size_t const npos;
    size_t const nvel;
    size_t const nbytes;
    
    value_type * joint_positions;
    value_type * joint_velocities;
    
    wbcnet::Buffer buffer;
    
    raw_state_s(size_t npos_, size_t nvel_)
      : npos(npos_),
	nvel(nvel_),
	nbytes(sizeof(value_type) * (npos + nvel)),
	buffer(nbytes, nbytes)
    {
      joint_positions = reinterpret_cast<value_type*>(buffer.GetData());
      joint_velocities = joint_positions + npos;
    }
  };
  
  
  template<typename value_t>
  struct raw_command_s
  {
    typedef value_t value_type;
    
    size_t const ncom;
    size_t const nbytes;
    
    value_type * joint_torques;
    
    wbcnet::Buffer buffer;
    
    raw_command_s(size_t ncom_)
      : ncom(ncom_),
	nbytes(sizeof(value_t) * ncom),
	buffer(nbytes, nbytes)
    {
      joint_torques = reinterpret_cast<value_type*>(buffer.GetData());
    }
  };
  
  
  class RawRobot
    : public wbc::BidirectionalRobotAPI
  {
  public:
    RawRobot(size_t npos, size_t nvel, size_t ncom, 
	     /** Will be deleted in the dtor */
	     wbcnet::SockWrap * sock_wrap);
    
    virtual ~RawRobot();
    
    virtual bool readSensors(SAIVector & jointAngles, SAIVector & jointVelocities,
			     timeval & acquisition_time, SAIMatrix * opt_force);
    virtual bool writeCommand(SAIVector const & command);
    virtual void shutdown() const;
    
    virtual bool writeSensors(SAIVector const & jointAngles, SAIVector const & jointVelocities,
			      SAIMatrix const * opt_force);
    virtual bool readCommand(SAIVector & command);
    
    raw_state_s<float> m_state;
    raw_command_s<float> m_command;
    wbcnet::SockWrap * m_sock_wrap;
    struct timeval m_tick;
  };
  
  
  struct RawFactory
    : public wbc::RobotFactory
  {
    virtual RawRobot * parse(std::string const & spec, wbc::ServoInspector * servo_inspector);
    virtual void dumpHelp(std::string const & prefix, std::ostream & os) const;
  };

}

#endif // NETROB_RAW_ROBOT_HPP
