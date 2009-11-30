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

//==============================================================================
/*!
  \author			Luis Sentis
  \file				BranchingRepresentation.h
*/
//==============================================================================

#ifndef WBC_Branching_Representation_H
#define WBC_Branching_Representation_H

#include <wbc/util/tao_util.hpp>
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <tao/matrix/TaoDeMath.h>	
#include <map>
#include <list>
#include <string>

class taoDNode;
class taoNode;	
class taoNodeRoot;

namespace wbc {

  using namespace std;
  
  
  /*!
    \brief An entity that provides the support to build a branching representation
    of TaoNodes.
  */

  class BranchingRepresentation {
    friend class TiXmlBRParser;
    friend class OsimBRParser;
    friend class BRBuilder;
    BranchingRepresentation();

  public:
    ~BranchingRepresentation();
    
    inline taoNodeRoot* rootNode() {return rootNode_; };  
  
    inline idToNodeMap_t & idToNodeMap() { return idToNodeMap_; }
    inline taoDNode* node(int nodeID) { return idToNodeMap_[nodeID];}
    
    taoDNode * findLink( std::string const & name );
    taoDNode * findJoint( std::string const & name );
    
    inline int numJoints() const {return numJoints_;}
    inline int numActuatedJoints() const {return unactuationMatrix_.row();}
  	  
    inline const SAIVector& defaultJointPositions() const { return defaultJointPosVec_; } 
    inline const SAIVector& upperJointLimits() const { return upperJointLimitVec_; } 
    inline const SAIVector& lowerJointLimits() const { return lowerJointLimitVec_; } 

    double surfaceDepth( taoDNode const * node ) const;

    inline double totalMass() const { return totalMass_; }
    inline const SAIVector& gravityAcceleration() const { return grav_; }
  
    inline const SAIMatrix& unactuationMatrix() const { return unactuationMatrix_; }

    /** \return relative position of force sensor. Returns zero vector
	      if no link tag is found */
    const SAIVector& forceSensorWRTJointFrame( taoDNode const * node ) const;
    
  private:
    // Attributes 
    taoNodeRoot* rootNode_; 
    idToNodeMap_t idToNodeMap_; 
    std::map<std::string, taoDNode*> linkNameToNodeMap_;
    std::map<std::string, taoDNode*> jointNameToNodeMap_;
    std::map<taoDNode const *, SAIVector> linkToSensorMap_;
    std::map<taoDNode const *, double> linkToSurfaceDepthMap_;
    int numJoints_;
    double totalMass_;
    SAIVector grav_;
    SAIMatrix unactuationMatrix_;

    // Constant
    static const SAIVector zero3;
  
    // Supporting attributes
    deVector3 pos_;
    SAIVector defaultJointPosVec_;
    SAIVector upperJointLimitVec_; //hard jointLimits
    SAIVector lowerJointLimitVec_; //hard jointLimits
    int parentNodeID_;

    // Private functions
    
    /** \return tao node from ID. */
    taoDNode* findNodeID( taoDNode*, int);

    static std::string canonicalJointName( const std::string & );
    static std::string canonicalLinkName( const std::string & );
  };

}

#endif // WBC_Branching_Representation_H
