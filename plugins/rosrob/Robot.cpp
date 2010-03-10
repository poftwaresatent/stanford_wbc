/*
 * ROS support for Stanford-WBC http://stanford-wbc.sourceforge.net/
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
   \file plugins/rosrob/Robot.cpp
   \author Roland Philippsen
*/

#include "Robot.hpp"
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>

// Quick hack around build sys bug: the rosrob plugin depends on ROS, which has log4cxx.
#define HAVE_LOG4CXX
#include <wbcnet/log.hpp>

#include <wbcnet/strutil.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sys/time.h>

using namespace sfl;
using namespace std;

static wbcnet::logger_t logger(wbcnet::get_logger("rosrob"));


static boost::shared_ptr<sensor_msgs::JointState const> global_joint_states;

static void joint_states_callback(boost::shared_ptr<sensor_msgs::JointState const> const & joint_states)
{
  global_joint_states = joint_states;
}


namespace wbc_rosrob_plugin {
  
  
  Robot::
  Robot(ros::NodeHandle & ros_node,
	std::string const & param_prefix,
	std::string const & urdf_param_name,
	std::string const & joint_states_topic_name)
    : m_ok(false),
      m_model(param_prefix)
  {
    try {
      m_model.initFromParam(ros_node, urdf_param_name, 0, 1);
      m_subscriber = ros_node.subscribe(joint_states_topic_name, 1, joint_states_callback);
      for (size_t ii(0); ii < m_model.joint_name_.size(); ++ii) {
	m_joint_index_map.insert(make_pair(m_model.joint_name_[ii], ii));
      }
      if (m_joint_index_map.empty()) {
	throw runtime_error("no joint names in model");
      }
      m_ok = true;
    }
    catch (std::runtime_error const & ee) {
      LOG_ERROR (logger, "wbc_rosrob_plugin::Robot ctor: EXCEPTION " << ee.what());
    }
  }
  
  
  Robot::
  ~Robot()
  {
  }
  
  
  bool Robot::
  readSensors(SAIVector & jointPositions, SAIVector & jointVelocities, timeval & acquisition_time,
	      SAIMatrix * opt_force)
  {
    if ( ! m_ok) {
      LOG_ERROR (logger, "wbc_rosrob_plugin::Robot::readSensors(): not properly initialized");
      return false;
    }
    
    if ( ! ros::ok()) {
      LOG_INFO (logger, "wbc_rosrob_plugin::Robot::readSensors(): ros::ok() failed");
      return false;
    }
    ros::spinOnce();
    
    if (opt_force) {
      opt_force->setSize(m_model.contact_nrows_, m_model.contact_ncols_, true);
    }
    
    jointPositions.setSize(m_model.ndof_, true);
    jointVelocities.setSize(m_model.ndof_, true);
    if ( ! global_joint_states) {
      LOG_WARN (logger, "wbc_rosrob_plugin::Robot::readSensors(): never received a message, pretend all is zero");
      acquisition_time.tv_sec = 0;
      acquisition_time.tv_usec = 0;
      return true;
    }
    
    if ((global_joint_states->name.size() != global_joint_states->position.size())
	|| (global_joint_states->name.size() != global_joint_states->velocity.size())) {
      LOG_ERROR (logger,
		 "wbc_rosrob_plugin::Robot::readSensors(): inconsistent vector sizes in received message\n"
		 << "  number of names:      " << global_joint_states->name.size() << "\n"
		 << "  number of positions:  " << global_joint_states->position.size() << "\n"
		 << "  number of velocities: " << global_joint_states->velocity.size());
      return false;
    }
    
    int nmatches(0);
    for (size_t ii(0); ii < global_joint_states->name.size(); ++ii) {
      joint_index_map_t::const_iterator jj(m_joint_index_map.find(global_joint_states->name[ii]));
      if (m_joint_index_map.end() != jj) {
	jointPositions[jj->second] = global_joint_states->position[ii];
	jointVelocities[jj->second] = global_joint_states->velocity[ii];
	++nmatches;
      }
    }
    acquisition_time.tv_sec = global_joint_states->header.stamp.sec;
    acquisition_time.tv_usec = global_joint_states->header.stamp.nsec / 1000;
    
    if (nmatches != m_model.ndof_) {
      LOG_WARN (logger, "wbc_rosrob_plugin::Robot::readSensors(): model has " << m_model.ndof_
		<< " DOF but there were " << nmatches << " matching joint names in the message");
    }
    
    return true;
  }
  
  
  bool Robot::
  writeCommand(SAIVector const & command)
  {
    LOG_WARN (logger, "wbc_rosrob_plugin::Robot::writeCommand(): this method has no effect");
    return true;
  }
  
  
  void Robot::
  shutdown() const
  {
  }
  
  
  Robot * Factory::
  parse(std::string const & spec, wbc::ServoInspector * servo_inspector)
  {
    std::string param_prefix("/pr2_stanford_wbc/");
    std::string node_home("~");
    std::string urdf_param_name("/robot_description");
    std::string joint_states_topic_name("/joint_states");
    bool init_ros_please(true);
    bool disable_ros_sigint(true);
    
    // XXXX to do: implement spec parsing...
    //...    sfl::splitstring(spec, '+', net_spec, channel_spec);
    
    if (init_ros_please) {
      int argc(1);
      char * argv[1];
      argv[0] = strdup("wbc_rosrob_plugin");
      if (disable_ros_sigint) {
	ros::init(argc, argv, "wbc_rosrob_plugin", ros::init_options::NoSigintHandler);
      }
      else {
	ros::init(argc, argv, "wbc_rosrob_plugin");
      }
      free(argv[0]);
    }
    
    ros::NodeHandle ros_node(node_home);
    Robot * ros_robot(new wbc_rosrob_plugin::Robot(ros_node,
						   param_prefix,
						   urdf_param_name,
						   joint_states_topic_name));
    return ros_robot;
  }
  
  
  void Factory::
  dumpHelp(std::string const & prefix, std::ostream & os) const
  {
    os << prefix << "spec = nothing so far... it hardcodes the default\n"
       << prefix << "  default = ~:/robot_description:/joint_states\n";
  }
  
}
