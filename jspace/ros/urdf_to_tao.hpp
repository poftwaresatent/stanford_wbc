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

/**
   \file urdf_to_tao.hpp Convert a URDF robot model into a TAO tree.
   \author Roland Philippsen
*/

#ifndef JSPACE_ROS_URDF_TO_TAO_HPP
#define JSPACE_ROS_URDF_TO_TAO_HPP


#include <jspace/tao_util.hpp>
#include <stdexcept>
#include <vector>
#include <set>


namespace urdf {
  class Model;
  class Link;
}

namespace jspace {
  namespace ros {  

  /**
     Abstract interface for determining whether a link should be
     considered "fixed" by urdf_to_tao::convert. Fixed links are
     typically used as a convenient shortcut when constructing a
     branching model from elementary components, or to attach
     auxiliary frames to existing links. During conversion to TAO,
     these fixed links must be fused into their parent, recursively,
     and while adjusting the inertial properties and child nodes.
  */
  class LinkFilter {
  public:
    virtual ~LinkFilter() {}
    virtual bool isFixed(urdf::Link const & urdf_link) const = 0;
  };
  
  
  /**
     The default implementation of LinkFilter simple checks whether
     the URDF model tags the link's parent joint as
     urdf::Joint::FIXED. Anything else, including links without
     parents, is considered non-fixed.
  */
  class DefaultLinkFilter: public LinkFilter {
  public:
    virtual bool isFixed(urdf::Link const & urdf_link) const;
  };
  
  
  /**
     An implementation of LinkFilter that considers links "fixed"
     unless they have been explicitly "activated".  In addition to the
     default action, this link filter checks whether a link has been
     previously registered using the AddLink() method.
  */
  class ActiveLinkFilter: public DefaultLinkFilter {
  public:
    /**
       Add a link to the set of "active" links, which are the only
       ones considered non-fixed.
    */
    void AddLink(std::string const & link_name);
    
    /**
       \return True if a link is in the set of active links.
    */
    inline bool HaveLink(std::string const & link_name) const
    { return m_active.end() != m_active.find(link_name); }
    
    /**
       \return True if the link is tagged urdf::Joint::FIXED or it has
       NOT been previously registered with AddLink().
    */
    virtual bool isFixed(urdf::Link const & urdf_link) const;
    
  protected:
    std::set<std::string> m_active;
  };
  
  
  /**
     An ActiveLinkFilter that loads a list of active link names from a
     flat text file.
  */
  class FlatFileLinkFilter: public ActiveLinkFilter {
  public:
    /**
       Register each whitespace-delimited string in the given file
       using the ActiveLinkFilter::AddLink() method. The first name in
       the file is considered the "root" name, which can be used to
       initialize the TAO root to that link. The \c # character
       denotes a comment, anything from the \c # to the end of that
       line gets ignored.
       
       \note Throws an exception if there is no name defined in the
       file.
    */
    void Load(std::string const & filename) throw(std::runtime_error);
    
    /**
       \return The first name that appeared in the file passed to Load(), or the empty string.
    */
    std::string const & GetRootName() const;
    
  protected:
    std::string m_root_name;
  };
  
  
  /**
     Convert the given URDF model to a TAO tree. Use the URDF link
     that has the name \c tao_root_name as the root of the TAO
     tree.
       
     The conversion fuses all subtrees that are connected via fixed
     links, adding their inertias and repatriating their children to
     descend from the fused subtree. The link specified as \c
     tao_root_name will be considered as fixed. For all remaining
     links, you can specify a custom function by providing \c
     link_filter which is a subclass of LinkFilter that informs the
     conversion algorithm about which links (actually the joint
     connecting them to their parent) should be considered fixed. You
     can use the DefaultLinkFilter in order to strictly adhere to what
     is labelled as fixed in the URDF tree.
     
     \return A structure containung the root of a freshly created TAO
     tree, along with some additional information that should make it
     easier to work with the TAO tree structure. An exception is
     thrown in case of errors. Typical errors are about invalid joint
     types. E.g. URDF has a notion of planar joint, which is lacking
     in TAO, and the joint axes in URDF can be arbitrary, whereas TAO
     is limited to the principal coordinate axes. These limitations of
     TAO might very well be removed in the future though.
  */
  tao_tree_info_s * convert_urdf_to_tao(urdf::Model const & urdf_model,
					std::string const & tao_root_name,
					LinkFilter const & link_filter) throw(std::runtime_error);
  
  
  /**
     Convert a URDF to several (identical) TAO trees. This is the same
     as convert_urdf_to_tao() but repeats the whole process \c
     n_tao_roots times.
  */
  void convert_urdf_to_tao_n(urdf::Model const & urdf_model,
			     std::string const & tao_root_name,
			     LinkFilter const & link_filter,
			     std::vector<tao_tree_info_s*> & tao_trees,
			     size_t n_tao_trees) throw(std::runtime_error);
  
  }
}

#endif // JSPACE_ROS_URDF_TO_TAO_HPP
