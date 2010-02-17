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
template <typename robot_link_ds>
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
	std::vector<robot_link_ds*>* root_link_vector_;
	
	/**
	 * Stores the child links. The child links are connected to the
	 * root links by the RobotArchitect
	 */
	std::vector<robot_link_ds*>* child_link_vector_;
	
	/**Map to contain the final linked robotic branching representation
	 * Links are addressable by their names.
	 */
	std::map<string,robot_link_ds*>* link_name_to_link_map_;
	
	/**If this is true then dereference all pointers in the destructor;
	 */
	bool deref_owned_ptrs_;
	
private:
	CRobotDefinition();
public:
	CRobotDefinition(bool arg_deref_ptrs);
	virtual ~CRobotDefinition();
	
	void addLink(const robot_link_ds & arg_link2add, const bool isRoot, const string arg_name);
	void addGlobalData(SGlobalRobotDS* arg_globData){  globalRobData_ = arg_globData;  }
	//NOTE TODO Consider implementing removeLink as well
	std::vector<robot_link_ds*>* getRootLinkVector(){	return root_link_vector_;	}
	std::vector<robot_link_ds*>* getChildLinkVector(){	return child_link_vector_;	}
	std::map<string,robot_link_ds*>* getName2LinkMap(){	return link_name_to_link_map_;	}
	SGlobalRobotDS* getGlobData(){ return &globalRobData_;  }
};


/**
 * Initializes the vectors. 
 */
template <typename robot_link_ds>
CRobotDefinition<robot_link_ds>::CRobotDefinition(bool arg_deref_ptrs)
{
	deref_owned_ptrs_ = arg_deref_ptrs;
	root_link_vector_ = new std::vector<robot_link_ds*>();
	child_link_vector_ = new std::vector<robot_link_ds*>();
	link_name_to_link_map_ = new std::map<string,robot_link_ds*>();
}

/**
 * Destroys the link vectors if they haven't been returned to the
 * RobotArchitect.
 * If the vectors have been returned, they have to be deallocated
 * by the RobotArchitect.
 */
template <typename robot_link_ds>
CRobotDefinition<robot_link_ds>::~CRobotDefinition()
{
	if(deref_owned_ptrs_)
	{
		delete link_name_to_link_map_;
		//NOTE TODO GO into the vectors and delete each node individually. 
		delete root_link_vector_;
	  delete child_link_vector_;	  
	}
}

/**
 * Adds a link to the root or child link vectors depending on the 
 * type of link to be added.
 */
template <typename robot_link_ds>
void CRobotDefinition<robot_link_ds>::addLink(const robot_link_ds& arg_link2add, const bool isRoot, const string arg_name)
{	
	robot_link_ds* tLnk;
	if(isRoot)
	{
		tLnk = new robot_link_ds(arg_link2add);
		root_link_vector_->push_back(tLnk);
		link_name_to_link_map_->insert( pair<string,robot_link_ds*>(arg_name, tLnk) );
	}
	else
	{
		tLnk = new robot_link_ds(arg_link2add);
		child_link_vector_->push_back(tLnk);		
		link_name_to_link_map_->insert( pair<string,robot_link_ds*>(arg_name, tLnk) );
	}		
}

}//End of namespace robotarchitect

#endif /*CROBOTDEFINITION_HPP_*/
