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

#ifndef WBCRUN_SERVICE_HPP
#define WBCRUN_SERVICE_HPP

#include <wbcnet/msg/UserCommand.hpp>
#include <wbcnet/data.hpp>
#ifdef WIN32
#include "extras.h"
#else
#include <unistd.h>
#endif
#include <stdlib.h>

namespace wbcrun {
  
  
  class wbcnet::msg::Service
    : public wbcnet::msg::UserCommand< wbcnet::Vector<int32_t>, wbcnet::Matrix<double> >
  {
  public:
    explicit wbcnet::msg::Service(/** You will probably want to use
				msg::USER_REQUEST or msg::USER_REPLY
				depending on whether this is a request
				or a reply message. */
		   wbcnet::unique_id_t id);
    
    /** Clears out everything and sets the code vector to one element
	with the value SRV_NOT_IMPLEMENTED (see wbcnet::srv_result_t below). */
    void Reset();
  };
  
#undef TRY_AGAIN


  namespace srv {
    
    typedef wbcnet::msg::Service::vector_type vector_t;
    typedef wbcnet::msg::Service::matrix_type matrix_t;
    
    /**
       Application specific user command status IDs.
       
       \note If you add a status, do not forget to update the
       implementation of srv_result_to_string() and string_to_result().
    */
    typedef enum {
      SUCCESS,
      NOT_IMPLEMENTED,
      TRY_AGAIN,
      INVALID_DIMENSION,
      INVALID_BEHAVIOR_ID,
      INVALID_TASK_ID,
      INVALID_REQUEST,
      INVALID_CODE,
      MISSING_CODE,
      INVALID_DATA,
      OUT_OF_RANGE,
      OTHER_ERROR
      // If you add something to this enum, please *immediately* also
      // update srv_result_to_string() and string_to_result().
    } result_t;
    
    /** \return The string representation according to the result_t
	enum, or "(invalid result)" if it lies outside that range. */
    char const * srv_result_to_string(int result);
    
    /** \return -1 for invalid name */
    int string_to_result(std::string const & name);
    
    
    /**
       Application specific user command IDs.
       
       \note If you add a command ID, do not forget to update the
       implementation of get_id_str().
       
       \note There is a bit of a redundance with
       wbcrun::SRV_request_t (from directory.hpp) when the id is
       BEHAVIOR_DIR or TASK_DIR.
       
       \todo Implement as many of these as possible in
       wbcrun::ServoProcess, or at least provide pre-processing for
       subclass implementers.
    */
    typedef enum {
      SET_GOAL,
      GET_POS,
      GET_VEL,
      GET_TORQUES,
      GET_END_POS,
      TOGGLE_RECORDER,
      KEY_PRESS,
      CLOSEST_DISTANCES,
      CHANGE_STATE,		// XXXX rfct into BEHAVIOR_DIR
      FLOAT,			// XXXX rfct into BEHAVIOR_DIR
      ACTIVATE,			// XXXX rfct into BEHAVIOR_DIR
      SERVO_DIR,
      BEHAVIOR_DIR,
      TASK_DIR ,
      SET_GAINS // IMPORTANT: changed this enum? *immediately* update get_id_str()!
    } id_t;
    
    char const * get_id_str(int id);
    
    void set_goal(wbcnet::msg::Service & request,
		  double const * goal_coordinates, size_t n_coordinates);
    void set_closest_distance(wbcnet::msg::Service & request,
			      int n_points, int * nodeids, double* point_coords);
    void get_pos(wbcnet::msg::Service & request);
    void change_state(wbcnet::msg::Service & request);
    void get_vel(wbcnet::msg::Service & request);
    void get_torques(wbcnet::msg::Service & request);
    void get_end_pos(wbcnet::msg::Service & request);
    void toggle_recorder(wbcnet::msg::Service & request);
    void key_press(wbcnet::msg::Service & request, int32_t keycode);
    void float_command(wbcnet::msg::Service & request);
    void activate_command(wbcnet::msg::Service & request);
    void set_gains(wbcnet::msg::Service & request, double const * gains, size_t n_gains);
    
    //  void set_tol(wbcnet::msg::Service & request, double const * tolerances, size_t n_tolerances);
  }
  
}

#endif // WBCRUN_SERVICE_HPP
