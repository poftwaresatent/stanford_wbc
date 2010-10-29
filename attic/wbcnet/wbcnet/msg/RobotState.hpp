/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

#ifndef WBCNET_MSG_ROBOT_STATE_HPP
#define WBCNET_MSG_ROBOT_STATE_HPP

#include <wbcnet/proxy.hpp>

namespace wbcnet {
  
  namespace msg {

    /**
       A Proxy for sending and receiving a robot state.
       
       The workings of CheckHeader() and UnpackHeader(): If
       auto_resize is true, CheckHeader() always succeeds and
       UnpackHeader() resizes the payload fields. Otherwise,
       CheckHeader() fails if the incoming message mismatches the
       dimensions given at construction time (and thus UnpackHeader()
       never needs to resize anything).
    */
    class RobotStateWrap
      : public Proxy
    {
    public:
      RobotStateWrap(unique_id_t id,
		     bool auto_resize,
		     uint8_t npos,
		     uint8_t nvel,
		     uint8_t forces_nrows,
		     uint8_t forces_ncolumns,
		     VectorStorageAPI * jointAnglesPtr,
		     VectorStorageAPI * jointVelocitiesPtr,
		     MatrixStorageAPI * forcesPtr);
    
      /** Checks are only active if auto_resize is false. Yell if
	  check_npos != npos or check_nvel != nvel. */
      virtual proxy_status CheckHeader() const;
      
      /** Resizing is active only if auto_resize is true. */
      virtual proxy_status UnpackHeader(BufferAPI const & buffer, endian_mode_t endian_mode);
      
      bool Copy(timestamp const & tstamp,
		VectorStorageAPI const & joint_angles,
		VectorStorageAPI const & joint_velocities,
		MatrixStorageAPI const & contact_forces);
      
      bool const auto_resize;
      uint8_t const check_npos;
      uint8_t const check_nvel;
      uint8_t const check_forces_nrows;
      uint8_t const check_forces_ncolumns;
    
      // header
      uint8_t requestID;
      uint8_t npos;
      uint8_t nvel;
      uint8_t forces_nrows;
      uint8_t forces_ncolumns;
      timestamp acquisitionTime;
      
      // wrapped payload
      VectorStorageAPI * jointAnglesPtr;
      VectorStorageAPI * jointVelocitiesPtr;
      MatrixStorageAPI * forcesPtr;
    };
  
  
    /**
       A Proxy for sending and receiving a robot state.
    */
    template<typename vector_t, typename matrix_t>
    class RobotState
      : public RobotStateWrap
    {
    public:
      typedef vector_t vector_type;
      typedef matrix_t matrix_type;
      
      RobotState(unique_id_t id,
		 bool auto_resize,
		 uint8_t npos,
		 uint8_t nvel,
		 uint8_t forces_nrows,
		 uint8_t forces_ncolumns)
	: RobotStateWrap(id, auto_resize, npos, nvel, forces_nrows, forces_ncolumns,
			 &jointAngles, &jointVelocities, &forces),
	  jointAngles(npos),
	  jointVelocities(nvel),
	  forces(forces_nrows, forces_ncolumns)
      {
      }
      
      bool operator == (RobotState const & rhs) const
      {
	return (requestID == rhs.requestID)
	  &&   (acquisitionTime == rhs.acquisitionTime)
	  &&   (jointAngles == rhs.jointAngles)
	  &&   (jointVelocities == rhs.jointVelocities)
	  &&   (forces == rhs.forces);
      }
      
      bool operator != (RobotState const & rhs) const
      { return ! (*this == rhs); }
      
      template<typename ostream_t>
      void display(ostream_t & os, char const * prefix) const
      {
	os << prefix << "requestID:       " << (int) requestID << "\n"
	   << prefix << "auto_resize:     " << (auto_resize ? "true\n" : "false\n")
	   << prefix << "npos:            " << (int) npos << "\n"
	   << prefix << "nvel:            " << (int) nvel << "\n"
	   << prefix << "forces_nrows:    " << (int) forces_nrows << "\n"
	   << prefix << "forces_ncolumns: " << (int) forces_ncolumns << "\n"
	   << prefix << "time:            " << acquisitionTime.tv_sec << "s "
	   << acquisitionTime.tv_usec << "usec\n";
	for (int ii(0); ii < npos; ++ii)
	  os << prefix << "jointAngles[" << ii << "]: " << jointAngles.GetElement(ii) << "\n";
	for (int ii(0); ii < nvel; ++ii)
	  os << prefix << "jointVelocities[" << ii << "]: " << jointVelocities.GetElement(ii) << "\n";
	for (int irow(forces_nrows - 1); irow > 0; --irow) {
	  os << prefix << "forces[" << irow << "]:";
	  for (int icol(0); icol < forces_ncolumns; ++icol)
	    os << "  " << forces.GetElement(irow, icol);
	  os << "\n";
	}
      }
      
      // real payload
      vector_t jointAngles;
      vector_t jointVelocities;
      matrix_t forces;
    };

  }
  
}

#endif // WBCNET_MSG_ROBOT_STATE_HPP
