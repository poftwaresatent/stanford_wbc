/*
 * Copyright (c) 2010 Stanford University
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file Process.cpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2008 Roland Philippsen, released under a BSD license.
*/

#include "Process.hpp"
#include <wbcnet/strutil.hpp>
#include <wbcnet/log.hpp>
#include <signal.h>
#include <stdlib.h>
#include <algorithm>
#include <iostream>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

using sfl::to_string;
using namespace std;

  
namespace wbc {
  
  
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
    LOG_TRACE (logger, "wbc::Process[" << name << "]::Send()");
    
    for (outgoing_t::iterator io(m_outgoing.begin()); io != m_outgoing.end(); ++io) {
      outgoing_s & out(io->second);
      out.done.clear();
      int count(0);
      for (proxylist_t::iterator ip(out.pending.begin()); ip != out.pending.end(); /*nop*/) {

	LOG_TRACE (logger,
		   "wbc::Process[" << name << "]::Send(): mux " << *ip << " over "
		   << out.sink);

	wbcnet::muldex_status const ms(m_muldex.Mux(out.sink, **ip));

	if (wbcnet::muldex_status::SUCCESS == ms.muldex) {
	  out.done.push_back(*ip);
	  ip = out.pending.erase(ip);
	  ++count;
	  if ((0 < out.max_n_snd) && (count >= out.max_n_snd)) {
	    LOG_TRACE (logger,
		       "wbc::Process[" << name << "]::Send(): reached max count " << count);
	    break;
	  }
	}

	else if (wbcnet::COM_TRY_AGAIN != ms.com)
	  throw runtime_error("wbc::Process[" + name + "]::Send(): m_muldex.Mux() said "
			      + string(wbcnet::muldex_status_str(ms)));
	
	else {
	  LOG_TRACE (logger,
		     "wbc::Process[" << name << "]::Send(): m_muldex.Mux() said TRY_AGAIN");
	  break;
	}
      }
    }
  }
  
  
  void Process::
  SendWait(int sleep_usecs) throw(std::exception)
  {
    LOG_TRACE (logger, "wbc::Process[" << name << "]::SendWait(" << sleep_usecs << ")");
    
    for (outgoing_t::iterator io(m_outgoing.begin()); io != m_outgoing.end(); ++io) {
      outgoing_s & out(io->second);
      out.done.clear();
      int count(0);
      for (proxylist_t::iterator ip(out.pending.begin()); ip != out.pending.end(); /*nop*/) {

	LOG_TRACE (logger,
		   "wbc::Process[" << name << "]::SendWait(" << sleep_usecs
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
		       "wbc::Process[" << name << "]::SendWait(" << sleep_usecs
		       << "): reached max count " << count);
	    break;
	  }
	}
	
	else if (wbcnet::COM_TRY_AGAIN != ms.com)
	  throw runtime_error("wbc::Process[" + name + "]::SendWait(" + to_string(sleep_usecs)
			      + "): m_muldex.MuxWait() said "
			      + string(wbcnet::muldex_status_str(ms)));
	
	else {
	  LOG_TRACE (logger,
		     "wbc::Process[" << name << "]::SendWait(" << sleep_usecs
		     << "): m_muldex.MuxWait() said TRY_AGAIN");
	  break;
	}
      }
    }
  }
  
  
  void Process::
  Receive() throw(std::exception)
  {
    LOG_TRACE (logger, "wbc::Process[" << name << "]::Receive()");
    
    for (incoming_t::iterator ii(m_incoming.begin()); ii != m_incoming.end(); ++ii) {
      incoming_s & in(ii->second);
      int count(0);
      
      LOG_TRACE (logger,
		 "wbc::Process[" << name << "]::Receive(): demux from " << in.source);
      
      wbcnet::muldex_status const ms(m_muldex.Demux(in.source, in.max_n_rcv, &count));
      
      if ((ms.muldex != wbcnet::muldex_status::SUCCESS)
	  && (ms.muldex != wbcnet::muldex_status::TRY_AGAIN))
	throw runtime_error("wbc::Process[" + name + "]::Receive(): m_muldex.Demux() said "
			    + string(wbcnet::muldex_status_str(ms)));
      
      LOG_TRACE (logger, "wbc::Process[" << name << "]::Receive(): demuxed " << count
		 << " of max " << in.max_n_rcv
		 << " messages: " << wbcnet::muldex_status_str(ms));
    }
  }
  
  
  void Process::
  ReceiveWait(int sleep_usecs, int min_nmsg) throw(std::exception)
  {
    LOG_TRACE (logger,
	       "wbc::Process[" << name << "]::ReceiveWait(" << sleep_usecs
	       << ", " << min_nmsg << ")");
    
    for (incoming_t::iterator ii(m_incoming.begin()); ii != m_incoming.end(); ++ii) {
      incoming_s & in(ii->second);
      int count(0);
      
      LOG_TRACE (logger,
		 "wbc::Process[" << name << "]::ReceiveWait(" << sleep_usecs
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
	throw runtime_error("wbc::Process[" + name + "]::ReceiveWait(" + to_string(sleep_usecs)
			    + ", " + to_string(min_nmsg) + "): m_muldex.Demux() said "
			    + string(wbcnet::muldex_status_str(ms)));
      
      LOG_TRACE (logger,
		 "wbc::Process[" << name << "]::ReceiveWait(" << sleep_usecs
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
	throw runtime_error("wbc::Process[" + name
			    + "]::EnqueueMessage(): unknown sink without auto_add");
      LOG_TRACE (logger,
		 "wbc::Process[" << name << "]::EnqueueMessage(): auto_add sink " << sink);
      m_outgoing.insert(make_pair(sink, outgoing_s(sink, 1)));
      io = m_outgoing.find(sink);
    }
    
    proxylist_t & pl(io->second.pending);
    if (ensure_unique && (pl.end() != find(pl.begin(), pl.end(), proxy)))
      return;
    pl.push_back(proxy);
    
    LOG_TRACE (logger,
	       "wbc::Process[" << name << "]::EnqueueMessage(): enqueued " << proxy
	       << " on " << sink);
  }
  
}
