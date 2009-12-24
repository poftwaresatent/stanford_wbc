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

#include "Service.hpp"
#include <limits>
#include <sstream>

using namespace std;


static uint8_t const msz(std::numeric_limits<uint8_t>::max());

static char const * srv_domain_name[] = {
  "SRV_SERVO_DOMAIN",
  "SRV_BEHAVIOR_DOMAIN",
  "SRV_TASK_DOMAIN",
  "SRV_OTHER_DOMAIN"
};

static char const * srv_command_name[] = {
  "SRV_KEY_PRESS",
  "SRV_TOGGLE_RECORDER",
  "SRV_GET_TASK_TYPE",
  "SRV_GET_DIMENSION",
  "SRV_GET_LINK_ANGLE",
  "SRV_GET_LINK_TRANSFORM",
  "SRV_SET_BEHAVIOR",
  "SRV_GET_BEHAVIOR",
  "SRV_SET_GOAL",
  "SRV_GET_GOAL",
  "SRV_GET_POSITIONS",
  "SRV_GET_VELOCITIES",
  "SRV_GET_TORQUES",
  "SRV_SET_PROP_GAIN",
  "SRV_GET_PROP_GAIN",
  "SRV_SET_DIFF_GAIN",
  "SRV_GET_DIFF_GAIN",
  "SRV_SET_MAX_VEL",
  "SRV_GET_MAX_VEL",
  "SRV_SET_MAX_ACCEL",
  "SRV_GET_MAX_ACCEL",
  "SRV_GET_BEHAVIOR_LIST",
  "SRV_GET_COMMAND_LIST",
  "SRV_GET_TASK_LIST",
  "SRV_OTHER_COMMAND"
};

static char const * srv_result_name[] = {
  "SRV_SUCCESS",
  "SRV_NOT_IMPLEMENTED",
  "SRV_TRY_AGAIN",
  "SRV_INVALID_DIMENSION",
  "SRV_INVALID_BEHAVIOR_ID",
  "SRV_INVALID_TASK_ID",
  "SRV_INVALID_COMMAND",
  "SRV_INVALID_CODE",
  "SRV_MISSING_CODE",
  "SRV_INVALID_DATA",
  "SRV_OUT_OF_RANGE",
  "SRV_OTHER_ERROR"
};


namespace wbcnet {
  
  
  string srv_domain_to_string(int domain)
  {
    if (0 > domain) {
      ostringstream os;
      os << "(negative domain: " << domain << ")";
      return os.str();
    }
    // note: SRV_OTHER_DOMAIN==domain gets handled using the srv_domain_name table
    if (SRV_OTHER_DOMAIN < domain) {
      ostringstream os;
      os << "SRV_OTHER_DOMAIN #" << domain - SRV_OTHER_DOMAIN;
      return os.str();
    }
    return srv_domain_name[domain];
  }
  
  
  int string_to_srv_domain(std::string const & name)
  {
    static map<string, int> nm;
    if (nm.empty())
      for (int ii(0); ii <= SRV_OTHER_DOMAIN; ++ii)
	nm[srv_domain_name[ii]] = ii;
    map<string, int>::const_iterator inm(nm.find(name));
    if (nm.end() == inm)
      return -1;
    return inm->second;
  }
  
  
  string srv_command_to_string(int command)
  {
    if (0 > command) {
      ostringstream os;
      os << "(negative command: " << command << ")";
      return os.str();
    }
    // note: SRV_OTHER_COMMAND==command gets handled using the srv_command_name table
    if (SRV_OTHER_COMMAND < command) {
      ostringstream os;
      os << "SRV_OTHER_COMMAND #" << command - SRV_OTHER_COMMAND;
      return os.str();
    }
    return srv_command_name[command];
  }
  
  
  int string_to_srv_command(std::string const & name)
  {
    static map<string, int> nm;
    if (nm.empty())
      for (int ii(0); ii <= SRV_OTHER_COMMAND; ++ii)
	nm[srv_command_name[ii]] = ii;
    map<string, int>::const_iterator inm(nm.find(name));
    if (nm.end() == inm)
      return -1;
    return inm->second;
  }
  
  
  string srv_result_to_string(int result)
  {
    if (0 > result) {
      ostringstream os;
      os << "(negative result: " << result << ")";
      return os.str();
    }
    // note: SRV_OTHER_ERROR==result gets handled using the srv_result_name table
    if (SRV_OTHER_ERROR < result) {
      ostringstream os;
      os << "SRV_OTHER_ERROR #" << result - SRV_OTHER_ERROR;
      return os.str();
    }
    return srv_result_name[result];
  }
  
  
  int string_to_srv_result(std::string const & name)
  {
    static map<string, int> nm;
    if (nm.empty())
      for (int ii(0); ii <= SRV_OTHER_ERROR; ++ii)
	nm[srv_result_name[ii]] = ii;
    map<string, int>::const_iterator inm(nm.find(name));
    if (nm.end() == inm)
      return -1;
    return inm->second;
  }
  
  
  namespace msg {
    
    
    Service::
    Service(unique_id_t id)
      : UserCommand< Vector<int32_t>, Matrix<double> >(id,
						       0, 0, 0, // initial sizes
						       msz, msz, msz // max sizes
						       )
    {
    }
    
    
    void Service::
    InitRequest(uint8_t _nCodes, uint8_t _nRows, uint8_t _nColumns)
    {
      ++requestID;
      clear();
      nCodes = _nCodes;
      nRows = _nRows;
      nColumns = _nColumns;
      code.SetNElements(_nCodes);
      code.SetZero();
      matrix.SetSize(_nRows, _nColumns);
      matrix.SetZero();
    }
    
    
    void Service::
    InitReply(Service const & request)
    {
      requestID = request.requestID;
      clear();
      nCodes = 1;
      nRows = 0;
      nColumns = 0;
      code.SetNElements(1);
      code[0] = SRV_NOT_IMPLEMENTED;
      matrix.SetSize(0, 0);
      matrix.SetZero();
    }
    
    
    void Service::
    InitListBehaviors()
    {
      InitRequest(2, 0, 0);
      code[0] = SRV_SERVO_DOMAIN;
      code[1] = SRV_GET_BEHAVIOR_LIST;
    }
    
    
    void Service::
    InitListBehaviorCmds(int behaviorID)
    {
      InitRequest(3, 0, 0);
      code[0] = SRV_BEHAVIOR_DOMAIN;
      code[1] = behaviorID;
      code[2] = SRV_GET_COMMAND_LIST;
    }
    
    
    void Service::
    InitServoCmd(int commandID,
		 srv_code_t const * code_in,
		 srv_matrix_t const * data_in)
    {
      InitRequest(code_in->NElements() + 2, 0, 0);
      code[0] = SRV_SERVO_DOMAIN;
      code[1] = commandID;
      code.Splice(2, *code_in, 0, std::numeric_limits<int>::max());
      matrix.Copy(*data_in);
    }
    
    
    void Service::
    InitBehaviorCmd(int behaviorID,
		    int commandID,
		    srv_code_t const * code_in,
		    srv_matrix_t const * data_in)
    {
      InitRequest(code_in->NElements() + 3, 0, 0);
      code[0] = SRV_BEHAVIOR_DOMAIN;
      code[1] = behaviorID;
      code[2] = commandID;
      code.Splice(3, *code_in, 0, std::numeric_limits<int>::max());
      matrix.Copy(*data_in);
    }
    
    
    void Service::
    InitListTasks(int behaviorID)
    {
      InitRequest(3, 0, 0);
      code[0] = SRV_BEHAVIOR_DOMAIN;
      code[1] = behaviorID;
      code[2] = SRV_GET_TASK_LIST;
    }
    
    
    void Service::
    InitListTaskCmds(int behaviorID,
		     int taskID)
    {
      InitRequest(4, 0, 0);
      code[0] = SRV_TASK_DOMAIN;
      code[1] = behaviorID;
      code[2] = taskID;
      code[3] = SRV_GET_COMMAND_LIST;
    }
    
    
    void Service::
    InitTaskCmd(int behaviorID,
		int taskID,
		int commandID,
		srv_code_t const * code_in,
		srv_matrix_t const * data_in)
    {
      InitRequest(code_in->NElements() + 4, 0, 0);
      code[0] = SRV_TASK_DOMAIN;
      code[1] = behaviorID;
      code[2] = taskID;
      code[3] = commandID;
      code.Splice(4, *code_in, 0, std::numeric_limits<int>::max());
      matrix.Copy(*data_in);
    }
    
    
    void Service::
    InitSetBehavior(int behaviorID)
    {
      InitRequest(3, 0, 0);
      code[0] = SRV_SERVO_DOMAIN;
      code[1] = SRV_SET_BEHAVIOR;
      code[2] = behaviorID;
    }
    
    
    void Service::
    InitSetGoal(double const * goal_coordinates, size_t n_coordinates)
    {
      InitRequest(2, n_coordinates, 1);
      code[0] = SRV_SERVO_DOMAIN;
      code[1] = SRV_SET_GOAL;
      for (size_t ii(0); ii < n_coordinates; ++ii)
	matrix.GetElement(ii, 0) = goal_coordinates[ii];
    }
    
    
    void Service::
    InitGetPositions()
    {
      InitRequest(2, 0, 0);
      code[0] = SRV_SERVO_DOMAIN;
      code[1] = SRV_GET_POSITIONS;
    }
    
    
    void Service::
    InitGetVelocities()
    {
      InitRequest(2, 0, 0);
      code[0] = SRV_SERVO_DOMAIN;
      code[1] = SRV_GET_VELOCITIES;
    }
    
    
    void Service::
    InitGetLinkTransform(int linkID)
    {
      InitRequest(3, 0, 0);
      code[0] = SRV_SERVO_DOMAIN;
      code[1] = SRV_GET_LINK_TRANSFORM;
      code[2] = linkID;
    }
    
    
    void Service::
    InitGetLinkTransform(std::string const & linkName)
    {
      InitRequest(2, 0, 0);
      code[0] = SRV_SERVO_DOMAIN;
      code[1] = SRV_GET_LINK_TRANSFORM;
      append(linkName);
    }
    
    
    void Service::
    InitGetTorques()
    {
      InitRequest(2, 0, 0);
      code[0] = SRV_SERVO_DOMAIN;
      code[1] = SRV_GET_TORQUES;
    }
    
    
    void Service::
    InitToggleRecorder()
    {
      InitRequest(2, 0, 0);
      code[0] = SRV_SERVO_DOMAIN;
      code[1] = SRV_TOGGLE_RECORDER;
    }
    
    
    void Service::
    InitKeyPress(int32_t keycode)
    {
      InitRequest(3, 0, 0);
      code[0] = SRV_SERVO_DOMAIN;
      code[1] = SRV_KEY_PRESS;
      code[2] = keycode;
    }
    
  }
  
}
