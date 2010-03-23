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

//THIS IS A TEST FILE TO BE DELETED SOON!
//#define CRobotDef
#define CRobotArch 

//Test code
#include <stdio.h>

#ifdef CRobotDef
	#include "CRobotDefinition.hpp"
#endif
#ifdef CRobotArch
//	#include "CRobotArchitect.hpp"
	#include "osimarchitect/COsimArchitect.hpp"
#endif

using namespace std;
using namespace robotarchitect;

int main(int argc, char* argv[])
{
	#ifdef CRobotDef
		printf("\nTesting CRobotDefinition");
		CRobotDefinition<SGraphicsRobotLink> grRobot;
		CRobotDefinition<SControllerRobotLink> crRobot;
	#endif
	
	#ifdef CRobotArch
		printf("\nTesting CRobotArchitect");
		COsimArchitect bobo;
		//OsimFiles
		//"osimarchitect/osimFiles/subject01_simbodySM.osim"
		//"osimarchitect/osimFiles/arm26Mod.osim"
		bobo.readRobotDefinition(argv[1],true,false); 
		bobo.buildRobotsFromLinks();
		
		CRobotDefinition<SControllerRobotLink>* crRobot;
		crRobot = bobo.returnControllerRobot();
		#ifdef TESTING_FUNCTIONS_ON
			bobo.CRobotArchitect::printLinkStructure();
		#endif
	#endif
	
	printf("\nTests completed!!");
	return -1;
}
