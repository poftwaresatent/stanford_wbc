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

#include "RobotState.hpp"
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));

namespace wbcnet {
  
  namespace msg {

    RobotStateWrap::
    RobotStateWrap(unique_id_t id,
		   uint8_t _npos,
		   uint8_t _nvel,
		   uint8_t _forces_nrows,
		   uint8_t _forces_ncolumns,
		   VectorStorageAPI * _jointAnglesPtr,
		   VectorStorageAPI * _jointVelocitiesPtr,
		   MatrixStorageAPI * _forcesPtr)
      : Proxy(id),
	check_npos(_npos),
	check_nvel(_nvel),
	check_forces_nrows(_forces_nrows),
	check_forces_ncolumns(_forces_ncolumns),
	npos(_npos),
	nvel(_nvel),
	forces_nrows(_forces_nrows),
	forces_ncolumns(_forces_ncolumns),
	jointAnglesPtr(_jointAnglesPtr),
	jointVelocitiesPtr(_jointVelocitiesPtr),
	forcesPtr(_forcesPtr)
    {
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("Servo::requestID", true, &requestID, 1));
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("Servo::npos", true, &npos, 1));
      m_header.AddField(new ValuePack<uint8_t, fixed_count>
			("Servo::nvel", true, &nvel, 1));
      m_header.AddField(new TimestampPack("Servo::acquisitionTime", true, &acquisitionTime));
      m_payload.AddField(new VectorPack("Servo::jointAngles", true, false, jointAnglesPtr));
      m_payload.AddField(new VectorPack("Servo::jointVelocities", true, false, jointVelocitiesPtr));
      m_payload.AddField(new MatrixPack("Servo::forces", true, false, &forcesPtr));
    }
  
  
    proxy_status RobotStateWrap::
    CheckHeader() const
    {
      if ((check_npos == npos) && (check_nvel == nvel)
	  && (check_forces_nrows == forces_nrows) && (check_forces_ncolumns == forces_ncolumns))
	return PROXY_OK;
      LOG_ERROR (logger,
		     "wbcnet::RobotStateWrap::CheckHeader(): size check failed:\n"
		     << "  npos is " << static_cast<unsigned int>(npos) << " (should be "
		     << static_cast<unsigned int>(check_npos) << ")\n"
		     << "  nvel is " << static_cast<unsigned int>(nvel) << " (should be "
		     << static_cast<unsigned int>(check_nvel) << ")\n"
		     << "  forces_nrows is " << static_cast<unsigned int>(forces_nrows)
		     << " (should be "
		     << static_cast<unsigned int>(check_forces_nrows) << ")\n"
		     << "  forces_ncolumns is " << static_cast<unsigned int>(forces_ncolumns)
		     << " (should be "
		     << static_cast<unsigned int>(check_forces_ncolumns) << ")");
      return PROXY_DIMENSION_MISMATCH;
    }
    
    
    bool RobotStateWrap::
    Copy(timestamp const & tstamp,
	 VectorStorageAPI const & joint_angles,
	 VectorStorageAPI const & joint_velocities,
	 MatrixStorageAPI const & contact_forces)
    {
      acquisitionTime = tstamp;
      return jointAnglesPtr->Copy(joint_angles)
	&& jointVelocitiesPtr->Copy(joint_velocities)
	&& forcesPtr->Copy(contact_forces);
    }
    
  }

}
