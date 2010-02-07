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

#ifndef WBC_URDF_TO_TAO_HPP
#define WBC_URDF_TO_TAO_HPP

#include <stdexcept>
#include <vector>
#include <set>


class taoNodeRoot;

namespace urdf {
  class Model;
  class Link;
}

namespace urdf_to_tao {
  

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
       
     The optional \c tao_id_to_link_name_map parameter, if non-NULL,
     is filled in with the names of the URDF links that reside at
     given TAO node IDs, which are numbered from 0 onward. The TAO
     root node has an ID of -1 and is not included in this map
     (because you provided it as the \c tao_root_name anyway).
     
     Similarly to \c tao_id_to_link_name_map, the optional \c
     tao_id_to_joint_name_map, if non-NULL, is filled in with the
     names of the URDF joints that reside at given TAO node IDs. Note
     that TAO supports multiple joints per link, but URDF seems to
     require that a joint alsways sits between two links, so here we
     can treat joint names just as link names by stuffing them into a
     \c std::vector.
       
     \return The root of the freshly created TAO tree. An exception is
     thrown in case of errors. Typical errors are about invalid joint
     types. E.g. URDF has a notion of planar joint, which is lacking
     in TAO, and the joint axes in URDF can be arbitrary, whereas TAO
     is limited to the principal coordinate axes. These limitations of
     TAO might very well be removed in the future though.
  */
  taoNodeRoot * convert(urdf::Model const & urdf_model,
			std::string const & tao_root_name,
			LinkFilter const & link_filter,
			std::vector<std::string> * tao_id_to_link_name_map,
			std::vector<std::string> * tao_id_to_joint_name_map) throw(std::runtime_error);
  
}

#endif // WBC_URDF_TO_TAO_HPP
