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

#include "CRobotArchitect.hpp"

#ifdef TESTING_FUNCTIONS_ON
#include <iostream>
using namespace std;
#endif

namespace robotarchitect
{

/* Constructor:
 */ 
CRobotArchitect::CRobotArchitect()
{
	//NOTE TODO The "true" constructor argument is hard-wired for now
	robdef_ = NULL;
	robot_initialized_ = false;	
}

/* Destructor: Does nothing.
 */
CRobotArchitect::~CRobotArchitect()
{}


/**Adds a link to the Controller robot definition
 */
bool CRobotArchitect::addLink(const SControllerRobotLink &arg_link2add)
{
	//bool isLinkConsistent = false;	
	//NOTE TODO Check isLinkConsistent (if necessary-low pri)	
	robdef_->addLink(arg_link2add,arg_link2add.is_root_, arg_link2add.link_name_);
	return true; //return isLinkConsistent;
}

/**Builds a set of robots based on the specifications in the links
 * Returns:
 * true = Success
 * false = Failure (describe)
 */
bool CRobotArchitect::buildRobotsFromLinks()
{
	bool flag = true;
	
	//Connect the links in CRobotDefinition<SControllerRobotLink> robdef_;
	
	//Vars to connect the robot links in the robot definition
	SControllerRobotLink* parentLinkAddr; //Parent link address 
	std::vector<SControllerRobotLink*>* rootLinkVec, * childLinkVec; //Pointers to link vectors
	std::map<string,SControllerRobotLink*>* robotName2LinkMap; //Pointer to the link-linkName_ map
	
	rootLinkVec = robdef_->getRootLinkVector();
	if( ((int)rootLinkVec->size()) == 0)
	{//No robots present in file 
		return false;		
	}
	childLinkVec = robdef_->getChildLinkVector();
	robotName2LinkMap = robdef_->getName2LinkMap();
	
	//NOTE TODO: Loop over the root links and assign ids (all -1 for now) to them.
  std::vector<SControllerRobotLink*>::iterator clink, clinke;
  for(clink = robdef_->getRootLinkVector()->begin(),
     clinke = robdef_->getRootLinkVector()->end();
     clink != clinke; ++clink)
  {
    (*clink)->parent_addr_ = NULL;
    (*clink)->link_id_ =  -1;
    (*clink)->parent_link_id_ =  -1;
    cout<<" "<<(*clink)->link_id_<<" ";
  }

	//Loop over the child links, connect them to their parents and assign ids
	for(clink = robdef_->getChildLinkVector()->begin(),
     clinke = robdef_->getChildLinkVector()->end();
     clink != clinke; ++clink)
  {
    string tstr = (*clink)->parent_link_name_;
  	parentLinkAddr = (SControllerRobotLink*)((*robotName2LinkMap)[tstr]);		
		(*clink)->parent_addr_ = parentLinkAddr;
		parentLinkAddr->child_addr_vector_.push_back((*robotName2LinkMap)[(*clink)->link_name_]);
		(*clink)->link_id_ =  clink - robdef_->getChildLinkVector()->begin(); //diff between iterators gives id
		cout<<" "<<(*clink)->link_id_<<" ";
  }

	//Parent ids need to be assigned in another loop to handle unordered link trees
	for(clink = robdef_->getChildLinkVector()->begin(),
	     clinke = robdef_->getChildLinkVector()->end();
	     clink != clinke; ++clink)
  {
    string tstr = (*clink)->parent_link_name_;
    parentLinkAddr = (SControllerRobotLink*)((*robotName2LinkMap)[tstr]);
    (*clink)->parent_link_id_ = parentLinkAddr->link_id_;
  }
	return flag;
}

/*This function returns a vector containing "Controller" branching representations 
 * for all the robots defined in the xml file 
 * Note: Please call buildRobotsFromLinks() before using this */
CRobotDefinition<SControllerRobotLink>* CRobotArchitect::returnControllerRobot()
{	return robdef_;	}


#ifdef TESTING_FUNCTIONS_ON

	void CRobotArchitect::printRobotTree(const SControllerRobotLink link, int depth)
	{
		vector<SControllerRobotLink*>::const_iterator clink, clinke;
		for(clink = link.childAddrVector.begin(),
     clinke = link.childAddrVector.end();
     clink != clinke; ++clink)
	  {
	  	cout<<"Link: "<<(*clink)->linkName_<<", Parent:"<<(*clink)->parentName_<<", Depth:"<<depth<<endl;			
	  }
	  for(clink = link.childAddrVector.begin(),
     clinke = link.childAddrVector.end();
     clink != clinke; ++clink)
	  {
	  	printRobotTree( (**clink),depth+1);			
	  }
	}
	
	void CRobotArchitect::printLinkStructure()
	{
		cout<<"\nPrint Root Links\n";
		std::vector<SControllerRobotLink*>::iterator rlink, rlinke;
		for(rlink = robdef_->getRootLinkVector()->begin(),
	     rlinke = robdef_->getRootLinkVector()->end();
	     rlink != rlinke; ++rlink)
	  {
	  	cout<<"Link: "<<(*rlink)->linkName_<<", Parent:"<<(*rlink)->parentName_<<", Depth:"<<0<<endl;	  	
		}
		
		cout<<"\n\nPrint Child Links\n";
		std::vector<SControllerRobotLink*>::iterator clink, clinke;
		for(clink = robdef_->getChildLinkVector()->begin(),
	     clinke = robdef_->getChildLinkVector()->end();
	     clink != clinke; ++clink)
	  {
	  	cout<<"Link: "<<(*clink)->linkName_<<", Parent:"<<(*clink)->parentName_<<", Depth:"<<0<<endl;	  	
		}
		
		cout<<"\n\nPrint Tree\n";
		//Print tree
		for(rlink = robdef_->getRootLinkVector()->begin(),
	     rlinke = robdef_->getRootLinkVector()->end();
	     rlink != rlinke; ++rlink)
	  {	  	
	  	cout<<"Link: "<<(*rlink)->linkName_<<", Parent:"<<(*rlink)->parentName_<<", Depth:"<<0<<endl;
	  	printRobotTree(*(*rlink),1);
		}
	}
#endif

}//End of namespace robotarchitect
