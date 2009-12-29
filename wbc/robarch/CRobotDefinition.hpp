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

#ifndef CROBOTDEFINITION_HPP_
#define CROBOTDEFINITION_HPP_

#include <vector>
#include <map>
#include <string>
#include "glob_rob_ds/SGlobalRobotDS.hpp"


namespace robotarchitect
{

using namespace std;

/**
 * This template class may contain the link definitions of a robot for:
 * 1. A set of robots in the wbc controller
 * 2. A set of robots in the graphical environment
 * 3. A set of robots in the dynamics engine
 * 
 * A tree representation is used wherein root links are connected to
 * subtrees of links.
 * 
 * The entire class definition is in this file since templates have
 * to be created in a single file.
 * */
template <typename robot_link>
class CRobotDefinition
{
public:
	/**
	 * Contains all the global robot data -- Such as gravity etc.
	 */
	// NOTE TODO : This should ideally be in a separate class which
	// should contain a set of RobotDefinitions and a GlobalData struct,
	// thereby completely defining a robotic world.
  SGlobalRobotDS globalRobData_;

private:
  /**
	 * Stores the root links. The child links are connected to the
	 * root links by the RobotArchitect.
	 * The number of robots specified in the data should be equal to
	 * the number of root links
	 */
	std::vector<robot_link*>* rootLinkVector;
	
	/**
	 * Stores the child links. The child links are connected to the
	 * root links by the RobotArchitect
	 */
	std::vector<robot_link*>* childLinkVector;
	
	/**Map to contain the final linked robotic branching representation
	 * Links are addressable by their names.
	 */
	std::map<string,robot_link*>* linkName2LinkMap;
	
	/**If this is true then dereference all pointers in the destructor;
	 */
	bool derefPtrs;
	
private:
	CRobotDefinition();
public:
	CRobotDefinition(bool arg_derefPtrs);
	virtual ~CRobotDefinition();
	
	void addLink(const robot_link & arg_link2add, const bool isRoot, const string arg_name);
	void addGlobalData(SGlobalRobotDS* arg_globData){  globalRobData_ = arg_globData;  }
	//NOTE TODO Consider implementing removeLink as well
	std::vector<robot_link*>* getRootLinkVector(){	return rootLinkVector;	}
	std::vector<robot_link*>* getChildLinkVector(){	return childLinkVector;	}
	std::map<string,robot_link*>* getName2LinkMap(){	return linkName2LinkMap;	}
	SGlobalRobotDS* getGlobData(){ return &globalRobData_;  }
};


/**
 * Initializes the vectors. 
 */
template <typename robot_link>
CRobotDefinition<robot_link>::CRobotDefinition(bool arg_derefPtrs)
{
	derefPtrs = arg_derefPtrs;
	rootLinkVector = new std::vector<robot_link*>();
	childLinkVector = new std::vector<robot_link*>();
	linkName2LinkMap = new std::map<string,robot_link*>();
}

/**
 * Destroys the link vectors if they haven't been returned to the
 * RobotArchitect.
 * If the vectors have been returned, they have to be deallocated
 * by the RobotArchitect.
 */
template <typename robot_link>
CRobotDefinition<robot_link>::~CRobotDefinition()
{
	if(derefPtrs)
	{
		delete linkName2LinkMap;
		//NOTE TODO GO into the vectors and delete each node individually. 
		delete rootLinkVector;
	  delete childLinkVector;	  
	}
}

/**
 * Adds a link to the root or child link vectors depending on the 
 * type of link to be added.
 */
template <typename robot_link>
void CRobotDefinition<robot_link>::addLink(const robot_link& arg_link2add, const bool isRoot, const string arg_name)
{	
	robot_link* tLnk;
	if(isRoot)
	{
		tLnk = new robot_link(arg_link2add);
		rootLinkVector->push_back(tLnk);
		linkName2LinkMap->insert( pair<string,robot_link*>(arg_name, tLnk) );
	}
	else
	{
		tLnk = new robot_link(arg_link2add);
		childLinkVector->push_back(tLnk);		
		linkName2LinkMap->insert( pair<string,robot_link*>(arg_name, tLnk) );
	}		
}

}//End of namespace robotarchitect

#endif /*CROBOTDEFINITION_HPP_*/
