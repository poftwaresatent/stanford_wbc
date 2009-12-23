/*
 * Copyright (c) 2009 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "directory.hpp"
#include <XmlRpc.h>

#define WBCRUN_DIRECTORY_DEBUG
#ifdef WBCRUN_DIRECTORY_DEBUG
# include <iostream>
#endif // WBCRUN_DIRECTORY_DEBUG

using namespace XmlRpc;
using namespace wbcnet;
using namespace wbcrun;
using namespace std;


namespace {
  
  
  static bool xmlrpc_to_vector(XmlRpcValue /*const?*/ & from, srv_code_t & to)
  {

#ifdef WBCRUN_DIRECTORY_DEBUG
    std::cout << "  xmlrpc_to_vector():\n    from: ";
    from.write(std::cout);
    std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG

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
#ifdef WBCRUN_DIRECTORY_DEBUG
    std::cout << "  xmlrpc_to_matrix():\n    from: ";
    from.write(std::cout);
    std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
    
    int const nrows(from["nrows"]);
    int const ncols(from["ncols"]);
    XmlRpcValue /*const?*/ & data(from["data"]);

#ifdef WBCRUN_DIRECTORY_DEBUG
    std::cout << "    nrows: " << nrows << "\n    ncols: " << ncols << "\n    data: ";
    data.write(std::cout);
    ////    std::cout << data.toXml();
    std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
    
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
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
      
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
      
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
    }
  };
  
  
  class ListBehaviorCmds: public Method {
  public:
    ListBehaviorCmds(XMLRPCDirectoryServer * dir): Method("ListBehaviorCmds", dir) {}
    
    // bool ListBehaviorCmds(int behaviorID, request_list_t & requests)
    void execute(XmlRpcValue & params, XmlRpcValue & result) {
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
      
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
      
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
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
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
      
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
      
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
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
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
      
      result.clear();		// "probably" redundant
      int const request(string_to_srv_command(params[0]));
      if (0 > request) {
	result["retval"] = wbcnet::srv_result_to_string(SRV_INVALID_COMMAND);
	return;
      }
      srv_code_t code_in(0);
      srv_matrix_t data_in(0, 0);
      if (params.size() >= 2) {
	xmlrpc_to_vector(params[1]["code_in"], code_in);
	xmlrpc_to_matrix(params[1]["data_in"], data_in);
      }
      srv_code_t code_out(0);
      srv_matrix_t data_out(0, 0);
      wbcnet::srv_result_t const retval(directory->HandleServoCmd(static_cast<srv_command_t>(request),
							   &code_in, &data_in,
							   &code_out, &data_out));
      result["retval"] = wbcnet::srv_result_to_string(retval);
      if (SRV_SUCCESS == retval) {
	vector_to_xmlrpc(code_out, result["code_out"]);
	matrix_to_xmlrpc(data_out, result["data_out"]);
      }
      
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
    }
  };
  
  
  class ListTasks: public Method {
  public:
    ListTasks(XMLRPCDirectoryServer * dir): Method("ListTasks", dir) {}
    
    // bool ListTasks(int behaviorID, listing_t & tasks)
    void execute(XmlRpcValue & params, XmlRpcValue & result) {
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
      
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
      
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
    }
  };
  
  
  class ListTaskCmds: public Method {
  public:
    ListTaskCmds(XMLRPCDirectoryServer * dir): Method("ListTaskCmds", dir) {}
    
    // bool ListTaskCmds(int behaviorID, int taskID, command_list_t & requests)
    void execute(XmlRpcValue & params, XmlRpcValue & result) {
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
      
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
      
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
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
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "DBG\n" << name() << "():\n  params:\n    ";
      params.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
      
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
      
#ifdef WBCRUN_DIRECTORY_DEBUG
      std::cout << "  result:\n    ";
      result.write(std::cout);
      std::cout << "\n" << std::flush;
#endif // WBCRUN_DIRECTORY_DEBUG
    }
  };
  
}


namespace wbcrun {
  
  
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
