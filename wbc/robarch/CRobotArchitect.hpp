/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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

#ifndef CROBOTARCHITECT_HPP_
#define CROBOTARCHITECT_HPP_

#include <string>
#include <cassert>
#include <vector>
#include <list>

//Robot Definition template
#include "CRobotDefinition.hpp"

//Data structures passed to the template
#include "glob_rob_ds/SGlobalRobotDS.hpp"
#include "glob_rob_ds/SGraphicsRobotLink.hpp"
#include "glob_rob_ds/SControllerRobotLink.hpp"

namespace robotarchitect
{

using namespace std;

	/**This class takes links passed from the xml parser and 
	 * converts them into a robot representation readable by
	 * the graphics engine, the dynamics engine or the controller.
	 */
	class CRobotArchitect
	{
	protected:	
		/***********************
		 * Controller robot data
		 ***********************/
		bool buildCrRobot;
	    /**The controller robot definition contains a root link vector and
	    * a child link vector */
		CRobotDefinition<SControllerRobotLink>* crRobotDef;
		
		/*********************
		 * Graphics robot data
		 *********************/
		bool buildGrRobot;
		/**The graphics robot definition contains a root link vector and
		   * a child link vector */
		CRobotDefinition<SGraphicsRobotLink>* grRobotDef;	
	
	public:
		CRobotArchitect();
		virtual ~CRobotArchitect();
		
		/**Creates a robot from the given xml file. The boolean flags specify
		 * whether the graphics robot or the controller robot or both should
		 * be built.
		 * Returns false if it can't find the file, or if both flags are false.
		 * 
		 * This function should be implemented by parsers who inherit from
		 * CRobotArchitect */
		virtual bool readRobotDefinition(const string arg_file, const bool buildControllerRobot, 
								 const bool buildGraphicsRobot)=0;
								 
		/**Adds a link to the Controller robot definition	 */
		bool addLink(const SControllerRobotLink &arg_link2add);
		
		/**Adds a link to the Graphics robot definition	 */
		bool addLink(const SGraphicsRobotLink &arg_link2add);
		
		/**Builds a set of robots based on the specifications in the links */
		int buildRobotsFromLinks();
		
	protected:
		/*This function will build the controller robots from crRobotDef */
		bool buildControllerRobots();
		
		/*This function will build the graphics robots from grRobotDef */
		bool buildGraphicsRobots();
		
	public:
		/** Return the controller robot definition
		 * Note: Please call buildRobotsFromLinks() before using this */ 
		CRobotDefinition<SControllerRobotLink>* returnControllerRobot();
		/** Return the graphics robot definition
		 * Note: Please call buildRobotsFromLinks() before using this */
		CRobotDefinition<SGraphicsRobotLink>* returnGraphicsRobot();
		
		#ifdef TESTING_FUNCTIONS_ON
		void printRobotTree(const SControllerRobotLink link, int depth);
		void printLinkStructure();
		#endif
	};

}//End of namespace robotarchitect

#endif /*CROBOTARCHITECT_HPP_*/
