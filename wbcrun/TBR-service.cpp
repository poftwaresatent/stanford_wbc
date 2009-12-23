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

#include "service.hpp"
#include <limits>

using namespace std;

static uint8_t const msz(std::numeric_limits<uint8_t>::max());

namespace wbcrun {


  wbcnet::msg::Service::
  wbcnet::msg::Service(wbcnet::unique_id_t id)
    : wbcnet::msg::
      UserCommand<wbcnet::Vector<int32_t>, wbcnet::Matrix<double> >(id,
								    0, 0, 0, // initial sizes
								    msz, msz, msz // max sizes
								    )
  {
  }
  
  
  namespace srv {
    
    static char const * result_name[] = {
      "SUCCESS",
      "NOT_IMPLEMENTED",
      "TRY_AGAIN",
      "INVALID_DIMENSION",
      "INVALID_BEHAVIOR_ID",
      "INVALID_TASK_ID",
      "INVALID_REQUEST",
      "INVALID_CODE",
      "MISSING_CODE",
      "INVALID_DATA",
      "OUT_OF_RANGE",
      "OTHER_ERROR"
    };
    
    
    char const * srv_result_to_string(int result)
    {
      if ((0 > result) || (OTHER_ERROR < result))
	return "(invalid result)";
      return result_name[result];
    }
    
    
    int string_to_result(std::string const & name)
    {
      static map<string, int> nm;
      if (nm.empty())
	for (int ii(0); ii <= OTHER_ERROR; ++ii)
	  nm[result_name[ii]] = ii;
      map<string, int>::const_iterator inm(nm.find(name));
      if (nm.end() == inm)
	return -1;
      return inm->second;
    }
  
  
    char const * get_id_str(int id)
    {
      static char * nm[] = {
	"SET_GOAL",
	"GET_POS",
	"GET_VEL",
	"GET_TORQUES",
	"GET_END_POS",
	"TOGGLE_RECORDER",
	"KEY_PRESS",
	"CLOSEST_DISTANCES",
	"CHANGE_STATE",
	"FLOAT",
	"ACTIVATE",
	"SERVO_DIR",
	"BEHAVIOR_DIR",
	"TASK_DIR",
  "SET_GAINS" // IMPORTANT: also adapt the if-clause two lines below.
      };
      if ((id < 0) || (id > SET_GAINS))
	return "(invalid command id)";
      return nm[id];
    }
  
  
    void set_goal(wbcnet::msg::Service & request, double const * goal_coordinates, size_t n_coordinates)
    {
      ++request.requestID;
      request.code.SetNElements(1);
      request.code[0] = SET_GOAL;
      request.matrix.SetSize(n_coordinates, 1);
      for (size_t ii(0); ii < n_coordinates; ++ii)
        request.matrix.GetElement(ii, 0) = goal_coordinates[ii];
    }

    void set_closest_distance(wbcnet::msg::Service & request,
			      int n_points,int * nodeids, double* point_coords){
    
      ++request.requestID;
      request.code.SetNElements(1);
      request.code[0] = CLOSEST_DISTANCES;
      request.matrix.SetSize(n_points, 8);
      for (size_t ii(0); ii < static_cast<size_t>(n_points); ++ii){
	request.matrix.GetElement(ii, 0) = nodeids[2*ii];
	request.matrix.GetElement(ii, 1) = nodeids[2*ii+1];
	for (size_t j(0); j < 6 ; ++j)
	  request.matrix.GetElement(ii, j+2) = point_coords[6*ii+j];	
      }      
    }  
  
    void get_pos(wbcnet::msg::Service & request)
    {
      ++request.requestID;
      request.code.SetNElements(1);
      request.code[0] = GET_POS;
      request.matrix.SetSize(0, 0);
    }
  
     void change_state(wbcnet::msg::Service & request)
    {
      ++request.requestID;
      request.code.SetNElements(1);
      request.code[0] = CHANGE_STATE;
      request.matrix.SetSize(0, 0);
    } 
    void get_end_pos(wbcnet::msg::Service & request)
    {
      ++request.requestID;
      request.code.SetNElements(1);
      request.code[0] = GET_END_POS;
      request.matrix.SetSize(0, 0);
    }
  
  
    void get_vel(wbcnet::msg::Service & request)
    {
      ++request.requestID;
      request.code.SetNElements(1);
      request.code[0] = GET_VEL;
      request.matrix.SetSize(0, 0);
    }
  
  
    void get_torques(wbcnet::msg::Service & request)
    {
      ++request.requestID;
      request.code.SetNElements(1);
      request.code[0] = GET_TORQUES;
      request.matrix.SetSize(0, 0);
    }
  
  
    void toggle_recorder(wbcnet::msg::Service & request)
    {
      ++request.requestID;
      request.code.SetNElements(1);
      request.code[0] = TOGGLE_RECORDER;
      request.matrix.SetSize(0, 0);
    }
  
  
    void key_press(wbcnet::msg::Service & request, int32_t keycode)
    {
      ++request.requestID;
      request.code.SetNElements(2);
      request.code[0] = KEY_PRESS;
      request.code[1] = keycode;
      request.matrix.SetSize(0, 0);
    }
    void float_command(wbcnet::msg::Service & request)
    {
      ++request.requestID;
      request.code.SetNElements(1);
      request.code[0] = FLOAT;
      request.matrix.SetSize(0, 0);
    }


    void activate_command(wbcnet::msg::Service & request)
    {
      ++request.requestID;
      request.code.SetNElements(1);
      request.code[0] = ACTIVATE;
      request.matrix.SetSize(0, 0);
    }


    void set_gains(wbcnet::msg::Service & request, double const * gains, size_t sz)
    {
      ++request.requestID;
      request.code.SetNElements(1);
      request.code[0] = SET_GAINS;
      request.matrix.SetSize(sz, 1);
      for (size_t ii(0); ii < sz; ++ii)
	      request.matrix.GetElement(ii, 0) = gains[ii];
    }
    
    
  }
  
  
  void wbcnet::msg::Service::
  Reset()
  {
    // somewhat redundant in most cases
    requestID = 0;
    nCodes = 1;
    nRows = 0;
    nColumns = 0;
    code.SetNElements(1);
    if (code.NElements() > 0)
      code[0] = SRV_NOT_IMPLEMENTED;
    matrixptr->SetSize(0, 0);
  }
  
}
