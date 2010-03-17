/*
 * Copyright (c) 2010 Stanford University
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
   \file XMLRPCDirectoryServer.cpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2009 Roland Philippsen, released under a BSD license.
*/

#include "directory.hpp"
#include "XMLRPCDirectoryServer.hpp"
#include <XmlRpc.h>

#define WBC_DIRECTORY_DEBUG
#ifdef WBC_DIRECTORY_DEBUG
# include <iostream>
#endif // WBC_DIRECTORY_DEBUG

using namespace XmlRpc;
using namespace wbcnet;
using namespace wbc;
using namespace std;


namespace {
  
  
  static bool xmlrpc_to_vector(XmlRpcValue /*const?*/ & from, srv_code_t & to)
  {

#ifdef WBC_DIRECTORY_DEBUG
    std::cout << "  xmlrpc_to_vector():\n    from: ";
    from.write(std::cout);
    std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG

    int const nelem(from.size());
    if ((to.NElements() != nelem) && ( ! to.SetNElements(nelem)))
      return false;
    for (int ii(0); ii < nelem; ++ii)
      to[ii] = from[ii];
    return true;
  }
  
  
  // would be nice to have such a thing in XmlRpcValue.h
  template<typename from_t, typename to_t>
  to_t convert(from_t const & from) {
    to_t to(from);
    return to;
  }
  
  
  static bool xmlrpc_to_matrix(XmlRpcValue /*const?*/ & from, srv_matrix_t & to)
  {
#ifdef WBC_DIRECTORY_DEBUG
    std::cout << "  xmlrpc_to_matrix():\n    from: ";
    from.write(std::cout);
    std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
    
    int const nrows(from["nrows"]);
    int const ncols(from["ncols"]);
    XmlRpcValue /*const?*/ & data(from["data"]);

#ifdef WBC_DIRECTORY_DEBUG
    std::cout << "    nrows: " << nrows << "\n    ncols: " << ncols << "\n    data: ";
    data.write(std::cout);
    ////    std::cout << data.toXml();
    std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
    
    int const size(nrows * ncols);
    if (data.size() != size)
      return false;
    if (((nrows != to.NRows()) || (ncols != to.NColumns()))
	&& ( ! to.SetSize(nrows, ncols)))
      return false;
    for (int ii(0); ii < size; ++ii) {
      if (data[ii].getType() == XmlRpcValue::TypeInt) 
	to.GetElement(ii) = convert<int, double>(data[ii]);
      else
	to.GetElement(ii) = data[ii];
    }
    return true;
  }
  
  
  static bool xmlrpc_to_listing(XmlRpcValue /*const?*/ & from, listing_t & to)
  {
    int const nelem(from.size());
    
#ifdef WBC_DIRECTORY_DEBUG
    std::cout << "  xmlrpc_to_listing():\n    nelem: " << nelem << "\n    from: ";
    from.write(std::cout);
    std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
    
    to.clear();
    for (int ii(0); ii < nelem; ++ii)
      to.push_back( /*string(*/ from[ii] /*)*/ );
    return true;
  }
  
  
  static void vector_to_xmlrpc(srv_code_t const & from, XmlRpcValue & to)
  {
    int const nelem(from.NElements());
    to.clear();			// "probably" redundant
    to.setSize(nelem);
    for (int ii(0); ii < nelem; ++ii)
      to[ii] = from[ii];
  }
  
  
  static void matrix_to_xmlrpc(srv_matrix_t const & from, XmlRpcValue & to)
  {
    int const nrows(from.NRows());
    int const ncols(from.NColumns());
    to.clear(); 		// "probably" redundant
    to["nrows"] = nrows;
    to["ncols"] = ncols;
    XmlRpcValue data;
    int const size(nrows * ncols);
    data.setSize(size);
    for (int ii(0); ii < size; ++ii)
      data[ii] = from.GetElement(ii);
    to["data"] = data;
  }
  
  
  static void listing_to_xmlrpc(listing_t const & from, XmlRpcValue & to)
  {
    size_t const nelem(from.size());
    to.clear();			// "probably" redundant
    to.setSize(nelem);
    listing_t::const_iterator ifrom(from.begin());
    for (size_t ii(0); ii < nelem; ++ii) {
      to[ii] = *ifrom;
      ++ifrom;
    }
  }
  
  
  class Method: public XmlRpcServerMethod {
  protected:
    Directory * directory;
  public:
    Method(std::string const & name, XMLRPCDirectoryServer * dir)
      : XmlRpcServerMethod(name, dir->GetServer()),
	directory(dir->GetDirectory()) {}
  };
  
  
  class ListBehaviors: public Method {
  public:
    ListBehaviors(XMLRPCDirectoryServer * dir): Method("ListBehaviors", dir) {}
    
    // bool ListBehaviors(listing_t & behaviors)
    void execute(XmlRpcValue & params, XmlRpcValue & result) {
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
      
      result.clear();		// "probably" redundant
      listing_t behaviors;
      wbcnet::srv_result_t const retval(directory->ListBehaviors(behaviors));
      result["retval"] = wbcnet::srv_result_to_string(retval);
      if (SRV_SUCCESS == retval) {
	XmlRpcValue listing;
	size_t ii(0);
	for (listing_t::const_iterator bb(behaviors.begin()); bb != behaviors.end(); ++bb, ++ii)
	  listing[ii] = *bb;
	result["behaviors"] = listing;
      }
      
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
    }
  };
  
  
  class ListBehaviorCmds: public Method {
  public:
    ListBehaviorCmds(XMLRPCDirectoryServer * dir): Method("ListBehaviorCmds", dir) {}
    
    // bool ListBehaviorCmds(int behaviorID, request_list_t & requests)
    void execute(XmlRpcValue & params, XmlRpcValue & result) {
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
      
      result.clear();		// "probably" redundant
      int const behaviorID(params[0]);
      command_list_t requests;
      wbcnet::srv_result_t const retval(directory->ListBehaviorCmds(behaviorID, requests));
      result["retval"] = wbcnet::srv_result_to_string(retval);
      if (SRV_SUCCESS == retval) {
	XmlRpcValue listing;
	size_t ii(0);
	for (command_list_t::const_iterator rr(requests.begin()); rr != requests.end(); ++rr, ++ii)
	  listing[ii] = srv_command_to_string(*rr);
	result["requests"] = listing;
      }
      
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
    }
  };
  
  
  class HandleBehaviorCmd: public Method {
  public:
    HandleBehaviorCmd(XMLRPCDirectoryServer * dir): Method("HandleBehaviorCmd", dir) {}
    
    // wbcnet::srv_result_t HandleBehaviorCmd(int behaviorID,
    //                                 SRV_request_t request,
    //                                 srv_code_t const * code_in,
    //                                 srv_matrix_t const * data_in,
    //                                 srv_code_t * code_out,
    //                                 srv_matrix_t * data_out)
    void execute(XmlRpcValue & params, XmlRpcValue & result) {
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
      
      result.clear();		// "probably" redundant
      int const behaviorID(params[0]);
      int const request(string_to_srv_command(params[1]));
      if (0 > request) {
	result["retval"] = wbcnet::srv_result_to_string(SRV_INVALID_COMMAND);
	return;
      }
      srv_code_t code_in(0);
      srv_matrix_t data_in(0, 0);
      if (params.size() >= 3) {
	xmlrpc_to_vector(params[2]["code_in"], code_in);
	xmlrpc_to_matrix(params[2]["data_in"], data_in);
      }
      srv_code_t code_out(0);
      srv_matrix_t data_out(0, 0);
      wbcnet::srv_result_t const retval(directory->HandleBehaviorCmd(behaviorID,
							      static_cast<srv_command_t>(request),
							      &code_in, &data_in,
							      &code_out, &data_out));
      result["retval"] = wbcnet::srv_result_to_string(retval);
      if (SRV_SUCCESS == retval) {
	vector_to_xmlrpc(code_out, result["code_out"]);
	matrix_to_xmlrpc(data_out, result["data_out"]);
      }
      
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
    }
  };
  
  
  class HandleServoCmd: public Method {
  public:
    HandleServoCmd(XMLRPCDirectoryServer * dir): Method("HandleServoCmd", dir) {}
    
    // wbcnet::srv_result_t HandleServoCmd(srv_command_t request,
    //                                 srv_code_t const * code_in,
    //                                 srv_matrix_t const * data_in,
    //                                 srv_code_t * code_out,
    //                                 srv_matrix_t * data_out)
    void execute(XmlRpcValue & params, XmlRpcValue & result) {
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
      
      result.clear();		// "probably" redundant
      int const request(string_to_srv_command(params[0]));
      if (0 > request) {
	result["retval"] = wbcnet::srv_result_to_string(SRV_INVALID_COMMAND);
	return;
      }
      srv_code_t code_in(0);
      srv_matrix_t data_in(0, 0);
      listing_t str_in;
      if (params.size() >= 2) {
	xmlrpc_to_vector(params[1]["code_in"], code_in);
	xmlrpc_to_matrix(params[1]["data_in"], data_in);
	xmlrpc_to_listing(params[1]["str_in"], str_in);
      }
      srv_code_t code_out(0);
      srv_matrix_t data_out(0, 0);
      listing_t str_out;
      wbcnet::srv_result_t const retval(directory->HandleServoCmd(static_cast<srv_command_t>(request),
								  &code_in, &data_in, str_in,
								  &code_out, &data_out, str_out));
      result["retval"] = wbcnet::srv_result_to_string(retval);
      if (SRV_SUCCESS == retval) {
	vector_to_xmlrpc(code_out, result["code_out"]);
	matrix_to_xmlrpc(data_out, result["data_out"]);
	listing_to_xmlrpc(str_out, result["str_out"]);
      }
      
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
    }
  };
  
  
  class ListTasks: public Method {
  public:
    ListTasks(XMLRPCDirectoryServer * dir): Method("ListTasks", dir) {}
    
    // bool ListTasks(int behaviorID, listing_t & tasks)
    void execute(XmlRpcValue & params, XmlRpcValue & result) {
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
      
      result.clear();		// "probably" redundant
      int const behaviorID(params[0]);
      listing_t tasks;
      wbcnet::srv_result_t const retval(directory->ListTasks(behaviorID, tasks));
      result["retval"] = wbcnet::srv_result_to_string(retval);
      if (SRV_SUCCESS == retval) {
	XmlRpcValue listing;
	size_t ii(0);
	for (listing_t::const_iterator tt(tasks.begin()); tt != tasks.end(); ++tt, ++ii)
	  listing[ii] = *tt;
	result["tasks"] = listing;
      }
      
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
    }
  };
  
  
  class ListTaskCmds: public Method {
  public:
    ListTaskCmds(XMLRPCDirectoryServer * dir): Method("ListTaskCmds", dir) {}
    
    // bool ListTaskCmds(int behaviorID, int taskID, command_list_t & requests)
    void execute(XmlRpcValue & params, XmlRpcValue & result) {
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
      
      result.clear();		// "probably" redundant
      int const behaviorID(params[0]);
      int const taskID(params[1]);
      command_list_t requests;
      wbcnet::srv_result_t const retval(directory->ListTaskCmds(behaviorID, taskID, requests));
      result["retval"] = wbcnet::srv_result_to_string(retval);
      if (SRV_SUCCESS == retval) {
	XmlRpcValue listing;
	size_t ii(0);
	for (command_list_t::const_iterator rr(requests.begin()); rr != requests.end(); ++rr, ++ii)
	  listing[ii] = srv_command_to_string(*rr);
	result["requests"] = listing;
      }
      
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
    }
  };
  
  
  class HandleTaskCmd: public Method {
  public:
    HandleTaskCmd(XMLRPCDirectoryServer * dir): Method("HandleTaskCmd", dir) {}
    
    // wbcnet::srv_result_t HandleTaskCmd(int behaviorID,
    //                             int taskID,
    //                             srv_command_t request,
    //                             srv_code_t const * code_in,
    //                             srv_matrix_t const * data_in,
    //                             srv_code_t * code_out,
    //                             srv_matrix_t * data_out)
    void execute(XmlRpcValue & params, XmlRpcValue & result) {
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
      
      result.clear();		// "probably" redundant
      int const behaviorID(params[0]);
      int const taskID(params[1]);
      int const request(string_to_srv_command(params[2]));
      if (0 > request) {
	result["retval"] = wbcnet::srv_result_to_string(SRV_INVALID_COMMAND);
	return;
      }
      srv_code_t code_in(0);
      srv_matrix_t data_in(0, 0);
      if (params.size() >= 4) {
	xmlrpc_to_vector(params[3]["code_in"], code_in);
	xmlrpc_to_matrix(params[3]["data_in"], data_in);
      }
      srv_code_t code_out(0);
      srv_matrix_t data_out(0, 0);
      wbcnet::srv_result_t const retval(directory->HandleTaskCmd(behaviorID,
							  taskID,
							  static_cast<srv_command_t>(request),
							  &code_in, &data_in,
							  &code_out, &data_out));
      result["retval"] = wbcnet::srv_result_to_string(retval);
      if (SRV_SUCCESS == retval) {
	vector_to_xmlrpc(code_out, result["code_out"]);
	matrix_to_xmlrpc(data_out, result["data_out"]);
      }
      
#ifdef WBC_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBC_DIRECTORY_DEBUG
    }
  };
  
}


namespace wbc {
  
  
  XMLRPCDirectoryServer::
  XMLRPCDirectoryServer(Directory * dir)
    : m_dir(dir),
      m_server(new XmlRpcServer())
  {
    m_methods.push_back(new ListBehaviors(this));
    m_methods.push_back(new ListBehaviorCmds(this));
    m_methods.push_back(new HandleBehaviorCmd(this));
    m_methods.push_back(new HandleServoCmd(this));
    m_methods.push_back(new ListTasks(this));
    m_methods.push_back(new ListTaskCmds(this));
    m_methods.push_back(new HandleTaskCmd(this));
  }
  
  
  XMLRPCDirectoryServer::
  ~XMLRPCDirectoryServer()
  {
    delete m_server;
    for (size_t ii(0); ii < m_methods.size(); ++ii)
      delete m_methods[ii];
  }
  
  
  Directory * XMLRPCDirectoryServer::
  GetDirectory()
  {
    return m_dir;
  }
  
  
  XmlRpcServer * XMLRPCDirectoryServer::
  GetServer()
  {
    return m_server;
  }
  
  
  void XMLRPCDirectoryServer::
  RunForever(int port)
  {
    m_server->bindAndListen(port);
    m_server->work(-1.0);
  }
  
}
