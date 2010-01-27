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
   \file plugins/rosrob/Robot.hpp
   \author Roland Philippsen
*/

#ifndef WBC_ROSROB_PLUGIN_ROBOT_HPP
#define WBC_ROSROB_PLUGIN_ROBOT_HPP

#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/RobotFactory.hpp>
#include <wbc/ros/Model.hpp>
#include <ros/subscriber.h>


namespace wbc_rosrob_plugin {
  
  
  /**
     A wbc::RobotAPI implementation that spawns a ROS node that
     listens to joint state messages. You cannot send any joint torque
     commands to it though (you can try, but it does nothing). This
     class is useful for debugging, as it allows to retrieve the
     robot's state in a format directly compatible with the WBC
     framework.
     
     \sa wbc_rosrob_plugin::Factory::parse().
   */
  class Robot
    : public wbc::RobotAPI
  {
  public:
    /**
       Create a ROS node that listens to joint state messages. It
       needs to know how the TAO tree used by the WBC got created, in
       order to filter the incoming message contents by joint
       name.
       
       You need to properly initialize the ROS system yourself. For
       example...
       
       \code
       ros::init(argc, argv, "pr2_stanford_wbc", ros::init_options::NoSigintHandler);
       //...
       ros::NodeHandle ros_node("~");
       std::string const urdf_param_name("/robot_description");
       std::string const joint_states_topic_name("/joint_states");
       wbc_rosrob_plugin::Robot * ros_robot(new wbc_rosrob_plugin::Robot(ros_node,
                                                                         urdf_param_name,
                                                                         joint_states_topic_name));
       SAIVector jointAngles, jointVelocities;
       timeval acquisition_time;
       while (ros_robot->readSensors(jointAngles, jointVelocities, acquisition_time, 0)) {
         do_something_cool(jointAngles);
       }
       // ...
       delete ros_robot;
       \endcode
       
       ...or you can use wbc_rosrob_plugin::Factory to do most of that
       work for you.
    */
    Robot(ros::NodeHandle & ros_node,
	  std::string const & urdf_param_name,
	  std::string const & joint_states_topic_name);
    
    virtual ~Robot();
    
    virtual bool readSensors(SAIVector & jointAngles, SAIVector & jointVelocities,
			     timeval & acquisition_time, SAIMatrix * opt_force);
    virtual bool writeCommand(SAIVector const & command);
    virtual void shutdown() const;
    
  protected:
    typedef std::map<std::string, size_t> joint_index_map_t;
    joint_index_map_t m_joint_index_map;
    
    bool m_ok;
    wbcros::Model m_model;
    ros::Subscriber m_subscriber;
  };
  
  
  /**
     Factory for wbc_rosrob_plugin::Robot instances.
  */
  struct Factory
    : public wbc::RobotFactory
  {
    /**
       Create a wbc_rosrob_plugin::Robot by parsing a specification
       string. The servo_inspector can be NULL in this case. You need
       to properly initialize the ROS system yourself though. For
       example...

       \code
       ros::init(argc, argv, "pr2_stanford_wbc", ros::init_options::NoSigintHandler);
       wbc_rosrob_plugin::Robot * ros_robot(wbc_rosrob_plugin::Factory::parse("", 0));
       if ( ! ros_robot) {
         oopsie_could_not_create_ros_robot();
       }
       \endcode
    */
    virtual Robot * parse(std::string const & spec, wbc::ServoInspector * servo_inspector);
    
    virtual void dumpHelp(std::string const & prefix, std::ostream & os) const;
  };

}

#endif // WBC_ROSROB_PLUGIN_ROBOT_HPP
