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
#include "wbcnet/win32/win32_compat.hpp"
#endif


#include <time.h>
#include <iosfwd>
#include <vector>

namespace wbcnet {
  
  
  /**
     Utility for measuring time and creating histograms on the
     fly. You can have several "sets" (things that you measure) that
     all share the same "bins" (histogram time slices). You can start
     and stop sets individually or all at the same time (see Start(),
     StartAll(), Stop(), StopAll()), and even inject your own external
     time measurements (see StartStop()). You can ask it to write the
     current histogram to std::ostream or FILE (see Dump(), DumpAll(),
     DumpTable()), and for convenience you can tell it only to print
     every \c oskip time so you do not have to implement a counter
     (see CheckDumpTable()).
     
     \note The floor and ceiling of the histogram have to be given at
     construction time, which kind of assumes that you already have a
     fair idea of the range of delays you encounter. In order to make
     it easier to find out if you are missing important samples above
     or below the range you are measuring, the absolute minimum and
     maximum delays are tracked as well. They are output along with
     the histogram, and you can also access them programmatically
     using GetMsMin(), GetMsMinAll(), GetMsMax(), and GetMsMaxAll().
  */
  class DelayHistogram
  {
  public:
    DelayHistogram(size_t nsets,
		   size_t nbins,
		   double ms_floor,
		   double ms_ceiling,
		   size_t oskip = 1);
    virtual ~DelayHistogram();
    
    /**
       Associate a string with a given set. This is used to make the
       output more human-readable when printing the histogram (see
       Dump(), DumpAll(), DumpTable(), and CheckDumpTable()).
    */
    bool SetName(size_t iset, std::string const & name);
    
    bool Start(size_t iset);
    bool StartAll();
    
    bool Stop(size_t iset);
    bool StopAll();
    
    /**
       Like Start() followed by Stop(), but given an external
       measurement.
    */
    bool StartStop(size_t iset, struct ::timeval const * start, struct ::timeval const * stop);
    
    bool Dump(std::ostream & os, size_t iset) const;
    bool DumpAll(std::ostream & os) const;
    
    /**
       Pretty print the histogram. The result will look somewhat like
       the following (produced using the testDelayHistogram program):
       \verbatim
         +----------------+--------------------+--------------------+--------------------+
         | HISTOGRAM      |              set 0 |              set 1 |              set 2 |
         +----------------+--------------------+--------------------+--------------------+
         | min     90.090 |             90.090 |            180.178 |            270.240 |
         | max    324.189 |            108.063 |            216.127 |            324.189 |
         +----------------+--------------------+--------------------+--------------------+
         | below  100.000 |                  5 |                  0 |                  0 |
         +----------------+--------------------+--------------------+--------------------+
         |  [ 0]  100.000 |                  5 |                  0 |                  0 |
         |  [ 1]  110.000 |                  0 |                  0 |                  0 |
       ...etc...
         |  [18]  280.000 |                  0 |                  0 |                  2 |
         |  [19]  290.000 |                  0 |                  0 |                  1 |
         +----------------+--------------------+--------------------+--------------------+
         | above  300.000 |                  0 |                  0 |                  5 |
         +----------------+--------------------+--------------------+--------------------+
       \endverbatim
     */
    bool DumpTable(FILE * of) const;
    
    /**
       Verifies that \c oskip is non-zero, sees if the current
       iteration is an integer multiple of \c oskip, and in that case
       prints the histogram in a nice table format using DumpTable().
       
       \return True in case something has been output, false
       otherwise.
    */
    bool CheckDumpTable(FILE * of) const;
    
    bool Reset(size_t iset);
    bool ResetAll();
    
    double GetMsMin(size_t iset) const;
    double GetMsMinAll() const;
    
    double GetMsMax(size_t iset) const;
    double GetMsMaxAll() const;
    
    double GetMsMean(size_t iset) const;
    
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
    double * m_ms_sum;		// [nsets]
    size_t * m_ms_count;	// [nsets]
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
