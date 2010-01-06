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
   \file Process.hpp
   \author Roland Philippsen
*/

#ifndef WBC_PROCESS_HPP
#define WBC_PROCESS_HPP

#include <wbcnet/Muldex.hpp>
#include <stdexcept>
#include <map>
#include <list>

namespace wbcnet {
  class Proxy;
  class Sink;
  class Source;
}


namespace wbc {
  
  
  /** Has to be a list because we remove items from it while iterating
      over its elements. */
  typedef std::list<wbcnet::Proxy *> proxylist_t;
  
  
  /** Contains pending outgoing messages, where they are to be sent,
      and how many should be sent. */
  struct outgoing_s {
    outgoing_s(wbcnet::Sink * _sink, int _max_n_snd)
      : sink(_sink), max_n_snd(_max_n_snd) {}
    
    wbcnet::Sink * sink;
    int max_n_snd; /** maximum number of messages to send (use "-1" for unlimited) */
    proxylist_t pending;
    proxylist_t done;
  };
  
  
  /** Contains an incoming channel and how many messages should be
      attempted to be read from it each time. */
  struct incoming_s {
    incoming_s(wbcnet::Source * _source, int _max_n_rcv)
      : source(_source), max_n_rcv(_max_n_rcv) {}
    
    wbcnet::Source * source;
    int max_n_rcv; /** maximum number of messages to receive (use "-1"
		       for unlimited, which creates an infinite loop,
		       so maybe do not use it that way) */
  };
  
  
  /**
     A process is something that can be run, possibly repeatedly, and
     that can listen to and/or send messages. See Process::Step() for
     information on how to send and receive messages.
     
     Subclasses have to implement the abstract
     wbcnet::MdxListener::HandleMessagePayload() method, whose
     signature is...

     \code
     int HandleMessagePayload(unique_id_t msg_id)
     \endcode

     ...and has to return 0 on success. Optionally, subclasses can
     also override the default
     wbcnet::MdxListener::HandleMessageHeader() implementation, in
     case they need to to stuff before the payload gets unpacked.
  */
  class Process
    : public wbcnet::MdxListener
  {
  public:    
    std::string const name;
    
    Process(std::string const & name,
	    /** Communication buffer (initial) size. You can just use
		0 and let it grow eccording to usage. */
	    int bufsize,
	    /** Maximum size of the communication buffer. Say "-1" if
		you want it to grow to whatever size it needs. */
	    int max_bufsize,
	    /** The recommended value is wbcnet::ENDIAN_DETECT. */
	    wbcnet::endian_mode_t endian_mode);
    
    virtual ~Process();
    
    /**
       You should probably call Send() or SendWait() as well as
       Receive() or ReceiveWait() in order to actually handle incoming
       and outgoing messages. The easiest way to implement a periodic
       process with non-blocking communication is probably like this:
       
       \code
       bool MyProcess::Step() throw(std::exception) {
	 Receive();
	 DoStuffAndEnqueueMessages();
         Send();
	 return true;
       }
       \endcode
       
       For a process that blocks e.g. on user input, maybe the
       following would be a good way:
       
       \code
       bool MyInteractiveProcess::Step() throw(std::exception) {
         if ( ! GetUserInputAndEnqueueMessages())
	   return false;
	 SendWait(10000);
	 ReceiveWait(10000, 1);
	 return true;
       }
       \endcode
       
       \return true if you want to be called again (i.e. periodic
       processes).
    */
    virtual bool Step() throw(std::exception) = 0;
    
    /**
       Non-blocking sending of messages previously passed to
       EnqueueMessage().  Goes through the outgoing message queue, and
       for each Sink that has pending messages try to send as many as
       possible or up to the ceiling specified that was specified to
       AddSink() when you registered that sink. Messages that could
       not be sent remain on the queue for the next time that you call
       Send().
    */
    void Send() throw(std::exception);
    
    /**
       Blocking sending of messages previously passed to
       EnqueueMessage().  Similar to Send(), but when a message cannot
       be sent it will repeatedly sleep for the specified amount and
       then retry, until either the given Sink has no more enqueued
       messages or the ceiling configured through AddSink() has been
       hit.
    */
    void SendWait(int sleep_usecs) throw(std::exception);
    
    /**
       Non-blocking reading of messages from sources previously
       registered through AddSource(). Loops over all registered
       sources and attempts to read up to the configured maximum
       number of messages from it.
    */
    void Receive() throw(std::exception);
    
    /**
       Blocking reading of messages from sources previously registered
       through AddSource(). Similar to Receive(), but it will
       repeatedly sleep and then retry until it has read the given
       minimum number of messages from each source in turn. As sources
       are processed in turn, this means that if one of them does not
       receive the expected amount of messages, the others are never
       processed, so use with caution. In any case, the number of
       messages gets limited (upper bound) by the value of max_n_rcv
       that was passed to AddSource().
    */
    void ReceiveWait(int sleep_usecs, int min_nmsg) throw(std::exception);
    
    /**
       Convenience method for adding message handlers. Called by
       subclasses (probably during doInit()) in order to let the
       m_muldex know which messages they handle. You can also do this
       manually, of course, by calling
       wbcnet::MdxDispatcher::SetHandler(). Whichever way you choose,
       in the end you have to override HandleMessagePayload() and --
       if you want -- even HandleMessageHeader() in order to implement
       the message callback.
       
       \note If you override an existing message_id, the old handler
       gets de-registered (and probably deleted, depending on who owns
       it).
    */
    void CreateHandler(/** The message ID you want to listen to. */
		       wbcnet::unique_id_t msg_id,
		       /** Name of this handler, mostly for debugging
			   and error messages. */
		       std::string const & name,
		       /** The proxy that this handler fills in for you. */
		       wbcnet::Proxy * proxy);
    
    /**
       Register a Sink and configure the maximum number of messages
       that should be sent at any given time you call Send() or
       SendWait(). If max_n_send is smaller than one, it is
       interpreted as "no limit".
    */
    void AddSink(wbcnet::Sink * sink,
		 /** Maximum number of messages to send each time you
		     call Send() or SendWait(). You can use numbers <=
		     0 to mean "no limit". */
		 int max_n_snd);
    
    /**
       Register a Source and configure the maximum number of messages
       that should be read from it when you call Receive() or
       ReceiveWait(). Use max_n_rcv smaller than one if you want no
       limit on the number of messages that should be read.
    */
    void AddSource(wbcnet::Source * source,
		   /** Maximum number of messages to read each time
		       you call Receive() or ReceiveWait(). You can
		       use numbers <= 0 to mean "no limit". */
		   int max_n_rcv);
    
    /**
       Put a pointer to something you want to send on the outgoing
       queue.  If the given Sink has not been previously registered
       through AddSink() and the parameter auto_add is true, then the
       sink will be configured with a max_n_send of 1 (one).
       
       \note <b>IMPORTANT</b> We only store the pointer to the proxy
       on the queue, message creation (the serialization) is deferred
       until a call to Send() or SendWait() hits that proxy (which might not
       even necessarily be the next time you call it, but possibly some
       later time). This means that if you change the value of the
       proxy after enqueueing but before sending, the new value gets
       sent, not the old one. Also, enqueueing the same proxy more
       than once at different moments will just result in the
       same message being sent multiple times. The rationale for this
       is that we want to avoid spurious copying (allocation,
       initialization, destruction) for as many things as possible.
       
       \todo The non-copying semantics are error prone, think about
       providing a flag to switch on proxy copying, or do the
       serialization right away and just store the data
       buffer.
    */
    void EnqueueMessage(wbcnet::Sink * sink, wbcnet::Proxy * proxy,
			bool ensure_unique, bool auto_add) throw(std::exception);
    
  protected:
    typedef std::map<wbcnet::Sink *, outgoing_s> outgoing_t;
    typedef std::map<wbcnet::Source *, incoming_s> incoming_t;
    
    /** Subclasses need not directly access the muldex. You can load
	it the easy way with CreateHandler(). Or, if you need more
	fine-grained control, you can use
	wbcnet::MdxDispatcher::SetHandler(). */
    wbcnet::MdxDispatcher m_muldex;
    
    /** Subclasses should never have to access this directly. Use
	AddSink() and EnqueueMessage() instead. */
    outgoing_t m_outgoing;
    
    /** Subclasses should never have to access this directly. Use
	AddSource() instead. */
    incoming_t m_incoming;
  };
  
}

#endif // WBC_PROCESS_HPP
