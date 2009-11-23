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
   \author Roland Philippsen
*/

#ifndef ROBOT_FAKE_HPP
#define ROBOT_FAKE_HPP


#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/RobotFactory.hpp>


class RobotFake
  : public wbc::BidirectionalRobotAPI
{
public:
  RobotFake(int extra_usleep);
  
  virtual bool readSensors(SAIVector & jointAngles, SAIVector & jointVelocities,
			   timeval & acquisition_time, SAIMatrix * opt_force);
  virtual bool writeCommand(SAIVector const & command);
  virtual void shutdown() const;
  
  virtual bool writeSensors(SAIVector const & jointAngles, SAIVector const & jointVelocities,
			    SAIMatrix const * opt_force);
  virtual bool readCommand(SAIVector & command);

protected:
  int const extra_usleep;
};


struct FactoryFake
  : public wbc::RobotFactory
{
  virtual RobotFake * parse(std::string const & spec, wbc::ServoInspector * servo_inspector);
  virtual void dumpHelp(std::string const & prefix, std::ostream & os) const;
};

#endif // ROBOT_FAKE_HPP
