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

#include "Process.hpp"
#include <wbcnet/strutil.hpp>
#include <wbcnet/log.hpp>
#include <signal.h>
#include <stdlib.h>
#include <algorithm>
#include <iostream>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcrun"));

using sfl::to_string;
using namespace std;

  
namespace wbcrun {
  
  
  Process::
  Process(std::string const & _name,
	  int bufsize,
	  int max_bufsize,
	  wbcnet::endian_mode_t endian_mode)
    : name(_name),
      m_muldex(bufsize, max_bufsize, endian_mode)
  {
  }
  
  
  Process::
  ~Process()
  {
  }
  
  
  void Process::
  Send() throw(std::exception)
  {
    LOG_TRACE (logger, "wbcrun::Process[" << name << "]::Send()");
    
    for (outgoing_t::iterator io(m_outgoing.begin()); io != m_outgoing.end(); ++io) {
      outgoing_s & out(io->second);
      out.done.clear();
      int count(0);
      for (proxylist_t::iterator ip(out.pending.begin()); ip != out.pending.end(); /*nop*/) {

	LOG_TRACE (logger,
		       "wbcrun::Process[" << name << "]::Send(): mux " << *ip << " over "
		       << out.sink);

	wbcnet::muldex_status const ms(m_muldex.Mux(out.sink, **ip));

	if (wbcnet::muldex_status::SUCCESS == ms.muldex) {
	  out.done.push_back(*ip);
	  ip = out.pending.erase(ip);
	  ++count;
	  if ((0 < out.max_n_snd) && (count >= out.max_n_snd)) {
	    LOG_TRACE (logger,
			   "wbcrun::Process[" << name << "]::Send(): reached max count " << count);
	    break;
	  }
	}

	else if (wbcnet::COM_TRY_AGAIN != ms.com)
	  throw runtime_error("wbcrun::Process[" + name + "]::Send(): m_muldex.Mux() said "
			      + string(wbcnet::muldex_status_str(ms)));
	
	else {
	  LOG_TRACE (logger,
			 "wbcrun::Process[" << name << "]::Send(): m_muldex.Mux() said TRY_AGAIN");
	  break;
	}
      }
    }
  }
  
  
  void Process::
  SendWait(int sleep_usecs) throw(std::exception)
  {
    LOG_TRACE (logger, "wbcrun::Process[" << name << "]::SendWait(" << sleep_usecs << ")");
    
    for (outgoing_t::iterator io(m_outgoing.begin()); io != m_outgoing.end(); ++io) {
      outgoing_s & out(io->second);
      out.done.clear();
      int count(0);
      for (proxylist_t::iterator ip(out.pending.begin()); ip != out.pending.end(); /*nop*/) {

	LOG_TRACE (logger,
		       "wbcrun::Process[" << name << "]::SendWait(" << sleep_usecs
		       << "): mux wait " << *ip << " over " << out.sink);
	
	wbcnet::muldex_status const ms(m_muldex.MuxWait(out.sink, **ip, sleep_usecs,
							(logger->isTraceEnabled() ? &cerr : 0)));
	
	if (wbcnet::muldex_status::SUCCESS == ms.muldex) {
	  out.done.push_back(*ip);
	  ip = out.pending.erase(ip);
	  // XXXX to do: should we disregard out.max_n_snd in SendWait()?
	  ++count;
	  if ((0 < out.max_n_snd) && (count >= out.max_n_snd)) {
	    LOG_TRACE (logger,
			   "wbcrun::Process[" << name << "]::SendWait(" << sleep_usecs
			   << "): reached max count " << count);
	    break;
	  }
	}
	
	else if (wbcnet::COM_TRY_AGAIN != ms.com)
	  throw runtime_error("wbcrun::Process[" + name + "]::SendWait(" + to_string(sleep_usecs)
			      + "): m_muldex.MuxWait() said "
			      + string(wbcnet::muldex_status_str(ms)));
	
	else {
	  LOG_TRACE (logger,
			 "wbcrun::Process[" << name << "]::SendWait(" << sleep_usecs
			 << "): m_muldex.MuxWait() said TRY_AGAIN");
	  break;
	}
      }
    }
  }
  
  
  void Process::
  Receive() throw(std::exception)
  {
    LOG_TRACE (logger, "wbcrun::Process[" << name << "]::Receive()");
    
    for (incoming_t::iterator ii(m_incoming.begin()); ii != m_incoming.end(); ++ii) {
      incoming_s & in(ii->second);
      int count(0);
      
      LOG_TRACE (logger,
		     "wbcrun::Process[" << name << "]::Receive(): demux from " << in.source);
      
      wbcnet::muldex_status const ms(m_muldex.Demux(in.source, in.max_n_rcv, &count));
      
      if ((ms.muldex != wbcnet::muldex_status::SUCCESS)
	  && (ms.muldex != wbcnet::muldex_status::TRY_AGAIN))
	throw runtime_error("wbcrun::Process[" + name + "]::Receive(): m_muldex.Demux() said "
			    + string(wbcnet::muldex_status_str(ms)));
      
      LOG_TRACE (logger, "wbcrun::Process[" << name << "]::Receive(): demuxed " << count
		     << " of max " << in.max_n_rcv
		     << " messages: " << wbcnet::muldex_status_str(ms));
    }
  }
  
  
  void Process::
  ReceiveWait(int sleep_usecs, int min_nmsg) throw(std::exception)
  {
    LOG_TRACE (logger,
		   "wbcrun::Process[" << name << "]::ReceiveWait(" << sleep_usecs
		   << ", " << min_nmsg << ")");
    
    for (incoming_t::iterator ii(m_incoming.begin()); ii != m_incoming.end(); ++ii) {
      incoming_s & in(ii->second);
      int count(0);
      
      LOG_TRACE (logger,
		     "wbcrun::Process[" << name << "]::ReceiveWait(" << sleep_usecs
		     << ", " << min_nmsg << "): demux wait from " << in.source);
      
      // not sure how DemuxWait reacts to max<min, so be careful
      int max_nmsg(min_nmsg);
      if (in.max_n_rcv > max_nmsg)
	max_nmsg = in.max_n_rcv;
      wbcnet::muldex_status const ms(m_muldex.DemuxWait(in.source, min_nmsg, max_nmsg,
							sleep_usecs,
							(logger->isTraceEnabled() ? &cerr : 0),
							&count));
      
      if ((ms.muldex != wbcnet::muldex_status::SUCCESS)
	  && (ms.muldex != wbcnet::muldex_status::TRY_AGAIN))
	throw runtime_error("wbcrun::Process[" + name + "]::ReceiveWait(" + to_string(sleep_usecs)
			    + ", " + to_string(min_nmsg) + "): m_muldex.Demux() said "
			    + string(wbcnet::muldex_status_str(ms)));
      
      LOG_TRACE (logger,
		     "wbcrun::Process[" << name << "]::ReceiveWait(" << sleep_usecs
		     << ", " << min_nmsg << "): demuxed " << count
		     << " of max " << max_nmsg
		     << " messages: " << wbcnet::muldex_status_str(ms));
    }
  }
  
  
  void Process::
  CreateHandler(wbcnet::unique_id_t msg_id,
		std::string const & name,
		wbcnet::Proxy * proxy)
  {
    m_muldex.SetHandler(msg_id, new wbcnet::ProxyHandler(name, *proxy, true, this));
  }
  
  
  void Process::
  AddSink(wbcnet::Sink * sink, int max_n_snd)
  {
    outgoing_t::iterator io(m_outgoing.find(sink));
    if (m_outgoing.end() == io)
      m_outgoing.insert(make_pair(sink, outgoing_s(sink, max_n_snd)));
    else
      io->second.max_n_snd = max_n_snd;
  }
  
  
  void Process::
  AddSource(wbcnet::Source * source, int max_n_rcv)
  {
    incoming_t::iterator ii(m_incoming.find(source));
    if (m_incoming.end() == ii)
      m_incoming.insert(make_pair(source, incoming_s(source, max_n_rcv)));
    else
      ii->second.max_n_rcv = max_n_rcv;
  }
  
  
  void Process::
  EnqueueMessage(wbcnet::Sink * sink, wbcnet::Proxy * proxy,
		 bool ensure_unique, bool auto_add) throw(std::exception)
  {
    outgoing_t::iterator io(m_outgoing.find(sink));
    if (m_outgoing.end() == io) {
      if ( ! auto_add)
	throw runtime_error("wbcrun::Process[" + name
			    + "]::EnqueueMessage(): unknown sink without auto_add");
      LOG_TRACE (logger,
		     "wbcrun::Process[" << name << "]::EnqueueMessage(): auto_add sink " << sink);
      m_outgoing.insert(make_pair(sink, outgoing_s(sink, 1)));
      io = m_outgoing.find(sink);
    }
    
    proxylist_t & pl(io->second.pending);
    if (ensure_unique && (pl.end() != find(pl.begin(), pl.end(), proxy)))
      return;
    pl.push_back(proxy);
    
    LOG_TRACE (logger,
		   "wbcrun::Process[" << name << "]::EnqueueMessage(): enqueued " << proxy
		   << " on " << sink);
  }
  
}
