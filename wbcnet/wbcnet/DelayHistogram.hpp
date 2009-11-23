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

#ifndef WBCNET_DELAY_HISTOGRAM_HPP
#define WBCNET_DELAY_HISTOGRAM_HPP

#ifndef WIN32
#include <sys/time.h>
#else
#include "extras.h"
#endif


#include <time.h>
#include <iosfwd>
#include <vector>

namespace wbcnet {
  
  class DelayHistogram
  {
  public:
    DelayHistogram(size_t nsets,
		   size_t nbins,
		   double ms_floor,
		   double ms_ceiling,
		   size_t oskip = 1);
    virtual ~DelayHistogram();
    
    bool SetName(size_t iset, std::string const & name);
    
    bool Start(size_t iset);
    bool StartAll();
    
    bool Stop(size_t iset);
    bool StopAll();
    
    /** Like Start() followed by Stop(), but given an external measurement. */
    bool StartStop(size_t iset, struct ::timeval const * start, struct ::timeval const * stop);
    
    bool Dump(std::ostream & os, size_t iset) const;
    bool DumpAll(std::ostream & os) const;
    bool DumpTable(FILE * of) const;
    bool CheckDumpTable(FILE * of) const;
    
    bool Reset(size_t iset);
    bool ResetAll();
    
    double GetMsMin(size_t iset) const;
    double GetMsMinAll() const;
    
    double GetMsMax(size_t iset) const;
    double GetMsMaxAll() const;
    
    size_t const nsets;
    size_t const nbins;
    double const ms_floor;
    double const ms_ceiling;
    double const ms_range;
    size_t const oskip;
    
  protected:
    void Update(size_t iset, struct ::timeval const * stop);
    
    size_t * m_below;		// [nsets]
    size_t * m_above;		// [nsets]
    size_t * m_bin;		// [nsets * nbins]
    struct ::timeval * m_start;	// [nsets]
    double * m_ms_min;		// [nsets]
    double * m_ms_max;		// [nsets]
    double * m_bin_floor;	// [nbins]
    double m_ms_min_all;
    double m_ms_max_all;
    size_t m_lower;
    size_t m_upper;
    
    mutable size_t m_iteration;
    
    std::vector<std::string> m_name;
  };
  
}

#endif // WBCNET_DELAY_HISTOGRAM_HPP
