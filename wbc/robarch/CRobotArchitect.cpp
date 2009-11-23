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
#endif

namespace robotarchitect
{

/* Constructor:
 */ 
CRobotArchitect::CRobotArchitect()
{
	//NOTE TODO The "true" constructor argument is hard-wired for now
	crRobotDef = NULL;
	grRobotDef = NULL;
	buildCrRobot = false;	
	buildGrRobot = false;	
}

CRobotArchitect::~CRobotArchitect()
{
	delete crRobotDef;
	delete grRobotDef;
}


/**Adds a link to the Controller robot definition
 */
bool CRobotArchitect::addLink(const SControllerRobotLink &arg_link2add)
{
	//bool isLinkConsistent = false;	
	//NOTE TODO Check isLinkConsistent (if necessary-low pri)	
	crRobotDef->addLink(arg_link2add,arg_link2add.is_root, arg_link2add.linkName_);
	return true; //return isLinkConsistent;
}

/**Adds a link to the Graphics robot definition
 */
bool CRobotArchitect::addLink(const SGraphicsRobotLink &arg_link2add)
{
	//bool isLinkConsistent = false;	
	//NOTE TODO Check isLinkConsistent (if necessary-low pri)	
	grRobotDef->addLink(arg_link2add,arg_link2add.is_root, arg_link2add.name);
	return true; //return isLinkConsistent;
}	

/**Builds a set of robots based on the specifications in the links
 * Returns:
 * 0 = Success
 * 1 = Both robots failed
 * 2 = Controller robot(s) failed
 * 3 = Graphics robot(s) failed * 
 */
int CRobotArchitect::buildRobotsFromLinks()
{
	bool crflag = false, grflag = false;
	int retval = 1;
	
	//Build the robots -> Cr and Gr are separate subsystems and require different objects
	if(buildCrRobot)
	{	crflag = buildControllerRobots();	}
	if(buildGrRobot)
	{	grflag = buildGraphicsRobots();		}
	
	if(!crflag&&!grflag)
	{	retval = 1;	}
	else if(!crflag)
	{	retval = 2;	}
	else if(!grflag)
	{	retval = 3;	}
	else
	{	retval = 0;	}
	
	return retval;	 
}

/*This function will build the controller robots from crRobotDef
 */
bool CRobotArchitect::buildControllerRobots()
{
	bool flag = true;
	
	//Connect the links in CRobotDefinition<SControllerRobotLink> crRobotDef;
	
	//Vars to connect the robot links in the robot definition
	SControllerRobotLink* parentLinkAddr; //Parent link address 
	std::vector<SControllerRobotLink*>* rootLinkVec, * childLinkVec; //Pointers to link vectors
	std::map<string,SControllerRobotLink*>* robotName2LinkMap; //Pointer to the link-linkName_ map
	
	rootLinkVec = crRobotDef->retRootLinkVector();
	if( ((int)rootLinkVec->size()) == 0)
	{//No robots present in file 
		return false;		
	}
	childLinkVec = crRobotDef->retChildLinkVector();
	robotName2LinkMap = crRobotDef->retName2LinkMap(); 
	
	//NOTE TODO: Loop over the root links and assign ids (all -1 for now) to them.
  std::vector<SControllerRobotLink*>::iterator clink, clinke;
  for(clink = crRobotDef->retRootLinkVector()->begin(),
     clinke = crRobotDef->retRootLinkVector()->end();
     clink != clinke; ++clink)
  {
    (*clink)->parentAddr = NULL;
    (*clink)->link_id =  -1;
    (*clink)->parent_link_id =  -1;
    cout<<" "<<(*clink)->link_id<<" ";
  }

	//Loop over the child links, connect them to their parents and assign ids
	for(clink = crRobotDef->retChildLinkVector()->begin(),
     clinke = crRobotDef->retChildLinkVector()->end();
     clink != clinke; ++clink)
  {
  	string tstr = (*clink)->parentName_;
  	parentLinkAddr = (SControllerRobotLink*)((*robotName2LinkMap)[tstr]);		
		(*clink)->parentAddr = parentLinkAddr;
		parentLinkAddr->childAddrVector.push_back((*robotName2LinkMap)[(*clink)->linkName_]);
		(*clink)->link_id =  clink - crRobotDef->retChildLinkVector()->begin();
		cout<<" "<<(*clink)->link_id<<" ";
  }

	//Parent ids need to be assigned in another loop to handle unordered link trees
	for(clink = crRobotDef->retChildLinkVector()->begin(),
	     clinke = crRobotDef->retChildLinkVector()->end();
	     clink != clinke; ++clink)
  {
    string tstr = (*clink)->parentName_;
    parentLinkAddr = (SControllerRobotLink*)((*robotName2LinkMap)[tstr]);
    (*clink)->parent_link_id = parentLinkAddr->link_id;
  }
	return flag;
}

/*This function will build the graphics robots from grRobotDef
 */
bool CRobotArchitect::buildGraphicsRobots()
{
	bool flag = false;
	
	//NOTE TODO connect the links in CRobotDefinition<SGraphicsRobotLink> grRobotDef;
	// This needs a copy paste from  buildControllerRobots and will be done once the
	// controller code has been tested.
	return flag;
}

/*This function returns a vector containing "Controller" branching representations 
 * for all the robots defined in the xml file 
 * Note: Please call buildRobotsFromLinks() before using this */
CRobotDefinition<SControllerRobotLink>* CRobotArchitect::returnControllerRobot()
{	return crRobotDef;	}

/*This function returns a vector containing "Graphics" branching representations 
 * for all the robots defined in the xml file 
 * Note: Please call buildRobotsFromLinks() before using this */
CRobotDefinition<SGraphicsRobotLink>* CRobotArchitect::returnGraphicsRobot()
{	return grRobotDef;	}


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
		for(rlink = crRobotDef->retRootLinkVector()->begin(),
	     rlinke = crRobotDef->retRootLinkVector()->end();
	     rlink != rlinke; ++rlink)
	  {
	  	cout<<"Link: "<<(*rlink)->linkName_<<", Parent:"<<(*rlink)->parentName_<<", Depth:"<<0<<endl;	  	
		}
		
		cout<<"\n\nPrint Child Links\n";
		std::vector<SControllerRobotLink*>::iterator clink, clinke;
		for(clink = crRobotDef->retChildLinkVector()->begin(),
	     clinke = crRobotDef->retChildLinkVector()->end();
	     clink != clinke; ++clink)
	  {
	  	cout<<"Link: "<<(*clink)->linkName_<<", Parent:"<<(*clink)->parentName_<<", Depth:"<<0<<endl;	  	
		}
		
		cout<<"\n\nPrint Tree\n";
		//Print tree
		for(rlink = crRobotDef->retRootLinkVector()->begin(),
	     rlinke = crRobotDef->retRootLinkVector()->end();
	     rlink != rlinke; ++rlink)
	  {	  	
	  	cout<<"Link: "<<(*rlink)->linkName_<<", Parent:"<<(*rlink)->parentName_<<", Depth:"<<0<<endl;
	  	printRobotTree(*(*rlink),1);
		}
	}
#endif

}//End of namespace robotarchitect
