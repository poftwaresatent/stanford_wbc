#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/Kinematics.hpp>
#include <wbc/core/Dynamics.hpp>


namespace jspace {
  
  
  Model::
  Model(wbc::RobotControlModel * robmodel)
    : robmodel_(robmodel),
      state_tick_(0),
      robmodel_tick_(0)
  {
  }
  
  
  void Model::
  setState(State const & state)
  {
    state_ = state;
    ++state_tick_;
  }
  
  
  int Model::
  getNNodes()
  {
    return robmodel_->branching()->idToNodeMap().size();
  }
  
  
  int Model::
  getNJoints()
  {
    return robmodel_->branching()->numJoints();
  }
  
  
  taoDNode * Model::
  getNode(int id)
  {
    return robmodel_->branching()->idToNodeMap()[id];
  }
  
  
  taoDNode * Model::
  getNode(std::string const & name_or_alias)
  {
    return robmodel_->branching()->findNode(name_or_alias);
  }
  
  
  taoDNode * Model::
  getJoint
  {
    return robmodel_->branching()->findJoint(name_or_alias);
  }
  
  
  int Model::
  getNodeID(taoDNode const * node)
  {
    return node->getID();
  }
  
  
  bool Model::
  computeTransformation(taoDNode const * from_node,
			taoDNode const * to_node,
			SAITransform & out_transform)
  {
    cerr << "IMPLEMENT jspace::Model::computeTransformation()\n";
    out_transform.identity();
    return false;
  }
  
  
  bool Model::
  computeLinkJacobian(taoDNode const * node,
		      SAIMatrix & jacobian)
  {
    if ( ! node) {
      return false;
    }
    
    static SAIVector const null(0);
    // wastes a tmp object...
    jacobian = robmodel_->kinematics()->JacobianAtPoint(node, null);
    return true;
  }
  
  
  bool Model::
  computePointJacobian(taoDNode const * node,
		       SAITransform const * frame,
		       SAIVector const & point,
		       SAIMatrix & jacobian)
  {
    if (0 != frame) {
      std::cerr << "IMPLEMENT non-global frame handling for jspace::Model::computePointJacobian()\n";
      return false;
    }
    
    if ( ! node) {
      return false;
    }
    
    // wastes a tmp object...
    jacobian = robmodel_->kinematics()->JacobianAtPoint(node, point);
    return true;
  }
  
  
  void Model::
  getGravity(SAIVector & gravity) const
  {
  }


#error "mais on s'en tape"
    
    /** Compute (or retrieve from the cache) the Coriolis and
	contrifugal joint-torque vector. */
    void Model::
  getCoriolisCentrifugal(SAIVector & coriolis_centrifugal) const;
    
    /** Compute (or retrieve from the cache) the joint-space
	mass-inertia matrix, a.k.a. the kinetic energy matrix. */
    void Model::
  getMassInertia(SAIMatrix & mass_inertia) const;
    
    /** Computed (or retrieve from the cache) the inverse joint-space
	mass-inertia matrix. */
    void Model::
  getInverseMassInertia(SAIMatrix & inverse_mass_inertia) const;
    
    
  protected:
    typedef std::map<std::string, taoDNode*> name_to_node_t;
    typedef std::map<taoDNode*, std::string> node_to_name_t;
    typedef std::map<int, taoDNode*> id_to_node_t;
    
    taoNodeRoot * tao_root_;
    name_to_node_t name_to_node_;
    node_to_name_t node_to_name_;
    id_to_node_t id_to_node_;
    
    // Probably some more supporting attributes, but will try to hide
    // TAO from the clients.
  };
  
}
