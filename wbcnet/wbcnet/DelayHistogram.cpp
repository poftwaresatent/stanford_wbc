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

#include "DelayHistogram.hpp"
#include <wbcnet/strutil.hpp>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <limits>

using namespace std;

namespace wbcnet {
  
  DelayHistogram::
  DelayHistogram(size_t _nsets,
		 size_t _nbins,
		 double _ms_floor,
		 double _ms_ceiling,
		 size_t _oskip)
    : nsets(_nsets),
      nbins(_nbins),
      ms_floor(_ms_floor),
      ms_ceiling(_ms_ceiling),
      ms_range(_ms_ceiling - _ms_floor),
      oskip(_oskip),
      m_below(new size_t[_nsets]),
      m_above(new size_t[_nsets]),
      m_bin(new size_t[_nsets * _nbins]),
      m_start(new ::timeval[_nsets]),
      m_ms_min(new double[_nsets]),
      m_ms_max(new double[_nsets]),
      m_bin_floor(new double[_nbins]),
      m_iteration(0)
  {
    for (size_t ii(0); ii < _nsets; ++ii)
      m_name.push_back("set " + sfl::to_string(ii));
    for (size_t ii(0); ii < _nbins; ++ii)
      m_bin_floor[ii] = ms_floor + ii * ms_range / nbins;
    ResetAll();
  }
  
  
  DelayHistogram::
  ~DelayHistogram()
  {
    delete[] m_below;
    delete[] m_above;
    delete[] m_bin;
    delete[] m_start;
    delete[] m_ms_min;
    delete[] m_ms_max;
    delete[] m_bin_floor;
  }
  
  
  bool DelayHistogram::
  Start(size_t iset)
  {
    if (nsets <= iset)
      return false;
    if (0 != gettimeofday(m_start + iset, 0))
      return false;
    return true;
  }
  
  
  bool DelayHistogram::
  StartAll()
  {
    if (nsets < 1)
      return false;
    if (0 != gettimeofday(m_start, 0))
      return false;
    struct timeval *dst(m_start + 1);
    for (size_t ii(1); ii < nsets; ++ii) {
      memcpy(dst, m_start, sizeof(*dst));
      ++dst;
    }
    return true;
  }
  
  
  void DelayHistogram::
  Update(size_t iset, struct timeval const * stop)
  {
    struct timeval const * start(m_start + iset);
    double const ms(1.0e3  * (stop->tv_sec  - start->tv_sec) +
		    1.0e-3 * (stop->tv_usec - start->tv_usec));
    
    double * ms_min(m_ms_min + iset);
    double * ms_max(m_ms_max + iset);
    if (ms < *ms_min) {
      *ms_min = ms;
      if (ms < m_ms_min_all)
	m_ms_min_all = ms;
    }
    if (ms > *ms_max) {
      *ms_max = ms;
      if (ms > m_ms_max_all)
	m_ms_max_all = ms;
    }
    
    double const bin(floor(nbins * (ms - ms_floor) / ms_range));
    if (bin < 0)
      ++m_below[iset];
    else if (bin >= nbins)
      ++m_above[iset];
    else {
      size_t const ibin(static_cast<size_t>(bin));
      ++(*(m_bin + iset * nbins + ibin));
      if (ibin < m_lower)
	m_lower = ibin;
      if (ibin > m_upper)
	m_upper = ibin;
    }
  }
  
  
  bool DelayHistogram::
  Stop(size_t iset)
  {
    if (nsets <= iset)
      return false;
    struct timeval stop;
    if (0 != gettimeofday(&stop, 0))
      return false;
    Update(iset, &stop);
    return true;
  }
  
  
  bool DelayHistogram::
  StopAll()
  {
    if (nsets < 1)
      return false;
    struct timeval stop;
    if (0 != gettimeofday(&stop, 0))
      return false;
    for (size_t iset(0); iset < nsets; ++iset)
      Update(iset, &stop);
    return true;
  }
  
  
  bool DelayHistogram::
  StartStop(size_t iset, struct ::timeval const * start, struct ::timeval const * stop)
  {
    if (nsets <= iset)
      return false;
    *(m_start + iset) = *start;
    Update(iset, stop);
    return true;
  }
  
  
  bool DelayHistogram::
  Dump(std::ostream & os, size_t iset) const
  {
    if (nsets <= iset)
      return false;
    os << "bounds:\n  " << m_ms_min[iset] << " to " << m_ms_max[iset] << "\n";
    if (0 < m_below[iset])
      os << "below\t" << m_below[iset] << "\n";
    size_t * bin(m_bin + iset * nbins);
    if (m_lower >= nbins)
      os << "all bins are empty\n";
    else {
      for (size_t ii(m_lower); ii <= m_upper; ++ii)
	os << "[" << ii << "]\t" << bin[ii] << "\n";
    }
    if (0 < m_above[iset])
      os << "above\t" << m_above[iset] << "\n";
    return true;
  }
  
  
  bool DelayHistogram::
  DumpAll(std::ostream & os) const
  {
    if (nsets < 1)
      return false;
    os << "bin floors:\n";
    for (size_t ibin(0); ibin < nbins; ++ibin)
      os << "  [" << ibin << "]: " << m_bin_floor[ibin] << "\n";
    os << "overall bounds:\n  " << m_ms_min_all << " to " << m_ms_max_all << "\n";
    for (size_t ii(0); ii < nsets; ++ii) {
      os << "\n" << m_name[ii] << "\n";
      Dump(os, ii);
    }
    return true;
  }
  
  
  bool DelayHistogram::
  DumpTable(FILE * of) const
  {
    if (nsets < 1)
      return false;
    fprintf(of, "+----------------+-------------------");
    for (size_t ii(1); ii < nsets; ++ii)
      fprintf(of, "-+-------------------");
    fprintf(of, "-+\n| HISTOGRAM      | %18.18s", m_name[0].c_str());
    for (size_t ii(1); ii < nsets; ++ii)
      fprintf(of, " | %18.18s", m_name[ii].c_str());
    fprintf(of, " |\n+----------------+-------------------");
    for (size_t ii(1); ii < nsets; ++ii)
      fprintf(of, "-+-------------------");
    fprintf(of, "-+\n| min   %8.3f |           %8.3f",
	    (m_ms_min_all == numeric_limits<double>::max() ? -1 : m_ms_min_all),
	    (m_ms_min[0] == numeric_limits<double>::max() ? -1 : m_ms_min[0]));
    for (size_t ii(1); ii < nsets; ++ii)
      fprintf(of, " |           %8.3f",
	      (m_ms_min[ii] == numeric_limits<double>::max() ? -1 : m_ms_min[ii]));
    fprintf(of, " |\n| max   %8.3f |           %8.3f",
	    (m_ms_max_all == numeric_limits<double>::min() ? -1 : m_ms_max_all),
	    (m_ms_max[0] == numeric_limits<double>::min() ? -1 : m_ms_max[0]));
    for (size_t ii(1); ii < nsets; ++ii)
      fprintf(of, " |           %8.3f",
	      (m_ms_max[ii] == numeric_limits<double>::min() ? -1 : m_ms_max[ii]));
    fprintf(of, " |\n+----------------+-------------------");
    for (size_t ii(1); ii < nsets; ++ii)
      fprintf(of, "-+-------------------");
    fprintf(of, "-+\n| below %8.3f | %18zu", m_bin_floor[0], m_below[0]);
    for (size_t ii(1); ii < nsets; ++ii)
      fprintf(of, " | %18zu", m_below[ii]);
    fprintf(of, " |\n+----------------+-------------------");
    for (size_t ii(1); ii < nsets; ++ii)
      fprintf(of, "-+-------------------");
    if (m_lower < nbins) {
      char * prf("-+");
      for (size_t jj(m_lower); jj <= m_upper; ++jj) {
	fprintf(of, "%s\n|  [%2zu] %8.3f | %18zu", prf, jj, m_bin_floor[jj], m_bin[jj]);
	prf = " |";
	for (size_t ii(1); ii < nsets; ++ii)
	  fprintf(of, " | %18zu", m_bin[ii * nbins + jj]);
      }
    }
    else
      fprintf(of, "-+\n|  (all bins are empty)");
    fprintf(of, " |\n+----------------+-------------------");
    for (size_t ii(1); ii < nsets; ++ii)
      fprintf(of, "-+-------------------");
    fprintf(of, "-+\n| above %8.3f | %18zu", m_bin_floor[nbins-1] + ms_range / nbins, m_above[0]);
    for (size_t ii(1); ii < nsets; ++ii)
      fprintf(of, " | %18zu", m_above[ii]);
    fprintf(of, " |\n+----------------+-------------------");
    for (size_t ii(1); ii < nsets; ++ii)
      fprintf(of, "-+-------------------");
    fprintf(of, "-+\n");
    return true;
  }
  
  
  bool DelayHistogram::
  Reset(size_t iset)
  {
    if (nsets <= iset)
      return false;
    memset(m_below + iset,         0, sizeof(*m_below));
    memset(m_above + iset,         0, sizeof(*m_above));
    memset(m_bin   + iset * nbins, 0, nbins * sizeof(*m_bin));
    memset(m_start + iset,         0, sizeof(*m_start));
    m_ms_min[iset] = numeric_limits<double>::max();
    m_ms_max[iset] = numeric_limits<double>::min();
    m_lower = nbins;
    m_upper = 0;
    return true;
  }
  
  
  bool DelayHistogram::
  ResetAll()
  {
    if (nsets < 1)
      return false;
    memset(m_below,  0, nsets * sizeof(*m_below));
    memset(m_above,  0, nsets * sizeof(*m_above));
    memset(m_bin,    0, nsets * nbins * sizeof(*m_bin));
    memset(m_start,  0, nsets * sizeof(*m_start));
    *m_ms_min = numeric_limits<double>::max();
    double *dst(m_ms_min + 1);
    for (size_t ii(1); ii < nsets; ++ii) {
      memcpy(dst, m_ms_min, sizeof(*dst));
      ++dst;
    }
    *m_ms_max = numeric_limits<double>::min();
    dst = m_ms_max + 1;
    for (size_t ii(1); ii < nsets; ++ii) {
      memcpy(dst, m_ms_max, sizeof(*dst));
      ++dst;
    }
    m_ms_min_all = numeric_limits<double>::max();
    m_ms_max_all = numeric_limits<double>::min();
    m_lower = nbins;
    m_upper = 0;
    return true;
  }
    
  
  double DelayHistogram::
  GetMsMin(size_t iset) const
  {
    if (nsets <= iset)
      return numeric_limits<double>::max();
    return m_ms_min[iset];
  }
  
  
  double DelayHistogram::
  GetMsMinAll() const
  {
    return m_ms_min_all;
  }
  
  
  double DelayHistogram::
  GetMsMax(size_t iset) const
  {
    if (nsets <= iset)
      return numeric_limits<double>::min();
    return m_ms_max[iset];
  }
  
  
  double DelayHistogram::
  GetMsMaxAll() const
  {
    return m_ms_max_all;
  }
  
  
  bool DelayHistogram::
  SetName(size_t iset, std::string const & name)
  {
    if (nsets <= iset)
      return false;
    m_name[iset] = name;
    return true;
  }
  
  
  bool DelayHistogram::
  CheckDumpTable(FILE * of) const
  {
    if (0 == oskip)
      return false;
    ++m_iteration;
    if (0 == (m_iteration % oskip))
      return DumpTable(of);
    return false;
  }
  
}
