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

#include "Muldex.hpp"
#include "proxy.hpp"
#include <wbcnet/log.hpp>
#include <iostream>

#ifndef WIN32
#include <unistd.h>
#else 
#include "win32/win32_compat.hpp"
#endif

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace wbcnet {
  
  
  char const * muldex_status_str(muldex_status const & ms)
  {
    switch (ms.muldex) {
    case muldex_status::SUCCESS:
      return "MULDEX_SUCCESS";
    case muldex_status::TRY_AGAIN:
      return "MULDEX_TRY_AGAIN";
    case muldex_status::COM_ERROR:
      return com_status_str(ms.com);
    case muldex_status::PROXY_ERROR:
      return proxy_status_str(ms.pack);
    case muldex_status::HANDLE_ERROR:
      return "MULDEX_HANDLE_ERROR";
    case muldex_status::NO_SINK:
      return "MULDEX_NO_SINK";
    case muldex_status::NO_SOURCE:
      return "MULDEX_NO_SOURCE";
    }
    return "(invalid muldex_status)";
  }
  
  
  Muldex::
  Muldex(int bufsize, int max_bufsize, endian_mode_t endian_mode)
    : m_endian_mode(endian_mode),
      m_buf(bufsize, max_bufsize)
  {
  }


  muldex_status Muldex::
  DemuxOne(Source * source)
  {
    muldex_status ms;
    
    if (0 == source) {
      LOG_ERROR (logger, "wbcnet::Muldex::DemuxOne(): source is NULL");
      ms.muldex = muldex_status::NO_SOURCE;
    }
    
    LOG_TRACE (logger, "wbcnet::Muldex::DemuxOne(): receiving...");
    
    ms.com = source->Receive(m_buf);
    if (COM_TRY_AGAIN == ms.com) {
      ms.muldex = muldex_status::TRY_AGAIN;
      return ms;
    }
    if (COM_OK != ms.com) {
      LOG_ERROR (logger,
		     "wbcnet::Muldex::DemuxOne(): source->Receive() failed with "
		     << com_status_str(ms.com));
      ms.muldex = muldex_status::COM_ERROR;
      return ms;
    }
    
    LOG_TRACE (logger, "wbcnet::Muldex::DemuxOne(): unpacking msg_id...");
    
    unique_id_t msg_id;
    ms.pack = proxy_unpack_msg_id(m_buf, msg_id, m_endian_mode);
    if (PROXY_OK != ms.pack) {
      LOG_ERROR (logger,
		     "wbcnet::Muldex::DemuxOne(): unpack_msg_id() failed with "
		     << proxy_status_str(ms.pack));
      ms.muldex = muldex_status::PROXY_ERROR;
      return ms;
    }
    
    LOG_TRACE (logger, "wbcnet::Muldex::DemuxOne(): calling Handle(" << int(msg_id) << ")");
    
    ms.handle = Handle(msg_id, m_buf);
    if (0 != ms.handle) {
      LOG_ERROR (logger,
		     "wbcnet::Muldex::DemuxOne(): Handle(" << int(msg_id)
		     << ") returned " << ms.handle);
      ms.muldex = muldex_status::HANDLE_ERROR;
      return ms;
    }

    if (logger->isTraceEnabled() && (COM_TRY_AGAIN != ms.com))
      LOG_TRACE (logger, "wbcnet::Muldex::DemuxOne(): returning " << muldex_status_str(ms));
    
    return ms;
  }
  
  
  muldex_status Muldex::
  Demux(Source * source, int max_nmsg, int * msgcount)
  {
    int count;
    if (0 == msgcount)
      msgcount = &count;
    
    muldex_status ms;
    for (*msgcount = 0; (0 > max_nmsg) || (*msgcount < max_nmsg); ++(*msgcount)) {
      ms = DemuxOne(source);
      if (muldex_status::SUCCESS != ms.muldex)
	break;
    }
    
    if (logger->isTraceEnabled() && (COM_TRY_AGAIN != ms.com))
      LOG_TRACE (logger,
		     "wbcnet::Muldex::Demux(): " << *msgcount << " of " << max_nmsg
		     << " message(s), returning " << muldex_status_str(ms));
    
    return ms;
  }
  
  
  muldex_status Muldex::
  DemuxWait(Source * source, int min_nmsg, int max_nmsg,
	    unsigned long usecsleep, std::ostream * progress,
	    int * msgcount)
  {
    LOG_TRACE (logger, "wbcnet::Muldex::DemuxWait(...)");
    
    int count;
    if (0 == msgcount)
      msgcount = &count;
    
    muldex_status ms;
    bool slept(false);
    
    for (*msgcount = 0; (0 > max_nmsg) || (*msgcount < max_nmsg); /* ++ only if SUCCESS */) {
      
      ms = DemuxOne(source);
      if ((muldex_status::SUCCESS != ms.muldex)
	  && (muldex_status::TRY_AGAIN != ms.muldex))
	break;			// an error occurred
      
      if (muldex_status::SUCCESS == ms.muldex)
	++(*msgcount);
      if (*msgcount >= min_nmsg)
	break;			// we've demuxed enough
      
      if (muldex_status::TRY_AGAIN == ms.muldex) {
	// dry source, wait a bit and check again
	if (progress)
	  *progress << "." << std::flush;
	usleep(usecsleep);
	slept = true;
      }
    }
    if ((0 != progress) && slept)
      *progress << "\n";
    
    LOG_TRACE (logger,
		   "wbcnet::Muldex::DemuxWait(): " << *msgcount << " of " << min_nmsg
		   << " (max " << max_nmsg << ") message(s), returning "
		   << muldex_status_str(ms));
    
    return ms;
  }
  
  
  muldex_status Muldex::
  Mux(Sink * sink, Proxy & proxy)
  {
    muldex_status ms;
    if (0 == sink) {
      LOG_ERROR (logger, "wbcnet::Muldex::Mux(): sink is NULL");
      ms.muldex = muldex_status::NO_SINK;
      return ms;
    }
    
    ms.pack = proxy.Pack(m_buf, m_endian_mode);
    if (PROXY_OK != ms.pack) {
      LOG_ERROR (logger,
		     "wbcnet::Muldex::Mux(): proxy.Pack() failed with "
		     << proxy_status_str(ms.pack));
      ms.muldex = muldex_status::PROXY_ERROR;
      return ms;
    }
    
    ms.com = sink->Send(m_buf);
    if (COM_OK != ms.com) {
      if (COM_TRY_AGAIN != ms.com) // suppress printing of TRY_AGAIN, it's often OK
	LOG_ERROR (logger,
		       "wbcnet::Muldex::Mux(): sink->Send() failed with "
		       << com_status_str(ms.com));
      ms.muldex = muldex_status::COM_ERROR;
    }
    
    return ms;
  }
  
  
  muldex_status Muldex::
  MuxWait(Sink * sink, Proxy & proxy, unsigned long usecsleep, std::ostream * progress)
  {
    muldex_status ms;
    bool slept(false);
    while (true) {
      ms = Mux(sink, proxy);
      if (wbcnet::COM_TRY_AGAIN == ms.com) {
	if (progress)
	  *progress << "." << std::flush;
	usleep(usecsleep);
	slept = true;
      }
      else
	break;			// returns error or OK
    }
    if ((0 != progress) && slept)
      *progress << "\n";
    return ms;
  }
  
  
  MdxListener::
  ~MdxListener()
  {
  }
  
  
  int MdxListener::
  HandleMessageHeader(unique_id_t msg_id)
  {
    return 0;
  }
  
  
  MdxHandler::
  MdxHandler(bool _dispatcher_owned, MdxListener * listener)
    : dispatcher_owned(_dispatcher_owned),
      m_listener(listener)
  {
  }
  

  MdxHandler::
  ~MdxHandler()
  {
  }
  
  
  int MdxHandler::
  HandleMessage(unique_id_t msg_id, BufferAPI const & buf, endian_mode_t endian_mode)
  {
    // first try the header, and notify any listener
    int status(DoHandleMessageHeader(msg_id, buf, endian_mode));
    if (0 != status)
      return status;
    if (0 != m_listener) {
      status = m_listener->HandleMessageHeader(msg_id);
      if (0 != status)
	return status;
    }
    // then try the payload, again notifying any listeners
    status = DoHandleMessagePayload(msg_id, buf, endian_mode);
    if (0 != status)
      return status;
    if (0 != m_listener)
      return m_listener->HandleMessagePayload(msg_id);
    return 0;
  }
  
  
  MdxDispatcher::
  MdxDispatcher(int bufsize, int max_bufsize, endian_mode_t endian_mode)
    : Muldex(bufsize, max_bufsize, endian_mode)
  {
  }
  
  
  MdxDispatcher::
  ~MdxDispatcher()
  {
    for (handler_map::iterator ih(m_handler.begin());
	 ih != m_handler.end(); ++ih)
      if (ih->second->dispatcher_owned)
	delete ih->second;
  }
  
  
  void MdxDispatcher::
  SetHandler(unique_id_t msg_id, MdxHandler * handler)
  {
    LOG_TRACE (logger, "wbcnet::MdxDispatcher::SetHandler(" << int(msg_id) << ", ...)");
    
    handler_map::iterator ih(m_handler.find(msg_id));
    if ((m_handler.end() != ih) && ih->second->dispatcher_owned) {
      LOG_TRACE (logger, "  deleting previously registered handler");
      delete ih->second;
    }
    
    m_handler[msg_id] = handler;
  }
  
  
  int MdxDispatcher::
  Handle(unique_id_t msg_id, BufferAPI const & buf)
  {
    LOG_TRACE (logger, "wbcnet::MdxDispatcher::Handler(" << int(msg_id) << ", ...)");
    
    handler_map::iterator ih(m_handler.find(msg_id));
    if (m_handler.end() == ih) {
      LOG_ERROR (logger, "MdxDispatcher::Handle(): no handler for msg_id " << int(msg_id));
      return -1;
    }
    return ih->second->HandleMessage(msg_id, buf, m_endian_mode);
  }
  

  ProxyHandler::
  ProxyHandler(std::string const & _name,
	       Proxy & proxy,
	       bool dispatcher_owned,
	       MdxListener * listener)
    : MdxHandler(dispatcher_owned, listener),
      name(_name),
      m_proxy(proxy)
  {
  }
  
  
  int ProxyHandler::
  DoHandleMessageHeader(unique_id_t msg_id, BufferAPI const & buf, endian_mode_t endian_mode)
  {
    proxy_status const ps(m_proxy.UnpackHeader(buf, endian_mode));
    if (PROXY_OK != ps) {
      LOG_ERROR (logger,
		     "wbcnet::ProxyHandler " << name
		     << ": wbcnet::Proxy::UnpackHeader() failed: " << proxy_status_str(ps));
      return -2;
    }
    return 0;
  }
  
  
  int ProxyHandler::
  DoHandleMessagePayload(unique_id_t msg_id, BufferAPI const & buf, endian_mode_t endian_mode)
  {
    proxy_status const ps(m_proxy.UnpackPayload(buf, endian_mode));
    if (PROXY_OK != ps) {
      LOG_ERROR (logger,
		     "wbcnet::ProxyHandler " << name
		     << ": wbcnet::Proxy::UnpackPayload() failed: " << proxy_status_str(ps));
      return -3;
    }
    return 0;
  }
  
}
