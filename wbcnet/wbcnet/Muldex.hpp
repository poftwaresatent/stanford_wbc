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

/** \file Muldex.hpp Multiplexing / demultiplexing for Proxy subclasses. */

#ifndef WBCNET_MULDEX_HPP
#define WBCNET_MULDEX_HPP

#include <wbcnet/com.hpp>
#include <wbcnet/proxy.hpp>
#include <wbcnet/data.hpp>
#include <iosfwd>
#include <map>


namespace wbcnet {
  
  
  /**
     Return value of several muldexing functions. As multiplexing /
     demultiplexing involves communication and serialization, in
     addition to passing messages to handlers, the muldex_status is
     composed of com_status, proxy_status, and its own status.
     
     \todo muldex_status::pack should be renamed
     muldex_status::proxy... this is a historical leftover that was
     removed when packing (marshalling) was made more generic and lots
     of stuff moved from there into the proxy interface.
  */

// Windows defines this in the preprocessor somewhere
#undef TRY_AGAIN

	struct muldex_status {
    typedef enum {
      SUCCESS,
      /**
	 BEWARE: non-blocking communication status is signaled
	 inconsistently, sometimes as COM_ERROR and sometimes as
	 TRY_AGAIN. The best way to check for TRY_AGAIN in
	 non-blocking communication modes is to see if \c
	 muldex_status::com==COM_TRY_AGAIN
	 
	 Example code:
	 \code
	 wbcnet::muldex_status const ms(m_muldex.DemuxOne(m_channel));
	 if (ms.com == wbcnet::COM_TRY_AGAIN) {
	   cout << "We have a COM_TRY_AGAIN status on our muldex!\n";
	 }
	 else if (ms.muldex != wbcnet::muldex_status::SUCCESS) {
	   cout << "Our muldex is seriously shot!\n";
	 }
	 else {
	   cout << "Check out these freshly demuxed data!\n";
	 }
	 \endcode
	 
	 \todo Find a better way of fusing things into muldex_status,
	 or at least introduce two kinds of TRY_AGAIN, namely
	 COM_TRY_AGAIN and OTHER_TRY_AGAIN.
      */
      TRY_AGAIN,
      /** look at muldex_status::com for more details */
      COM_ERROR,
      /** look at muldex_status::pack for more details */
      PROXY_ERROR,
      /** look at muldex_status::handle for more details */
      HANDLE_ERROR,
      NO_SINK,
      NO_SOURCE
    } status;
    
    muldex_status()
      : muldex(SUCCESS), handle(0), com(COM_OK), pack(PROXY_OK) {}
    
    /** summarizing status */
    status muldex;
    
    /** status of Muldex::Handle() */
    int handle;
    
    /** status of any attempted Source::Receive() or Sink::Send() */
    com_status com;
    
    /** status of any Proxy-related functions,
	i.e. proxy_unpack_msg_id() or Proxy::Pack() */
    proxy_status pack;
  };
  
  /**
     \return A static string describing the muldex_status.
  */
  char const * muldex_status_str(muldex_status const & ms);
  
  
  /**
     Message multiplexer / demultiplexer for Proxy
     instances. Multiplexing is straightforward and directly
     implemented here. Demultiplexing involves a delegation through
     the abstract Handle() method, which can decide what to do with a
     message based on its ID.
     
     Either you subclass Muldex and do manual handling, or you use the
     provided MdxDispatcher class and register some functors that know
     how to handle specific messages.
  */
  class Muldex
  {
  public:
    Muldex(/** Passed to an underlying Buffer instance, see its
	       constructor arguments. Basically, this is the initial
	       buffer size, and you can use max_bufsize < 0 to say
	       "grow indefinitely if needed". */
	   int bufsize,
	   /** Passed to an underlying Buffer instance, see its
	       constructor arguments. This is the maximum size to
	       which the buffer will grow. Say -1 (or some other
	       negative value) to say "unlimited". */
	   int max_bufsize,
	   /** if networking, use ENDIAN_DETECT, otherwise
	       ENDIAN_NEVER_SWAP to avoid swapping when staying within
	       the same computer */
	   endian_mode_t endian_mode);
    virtual ~Muldex() {}
    
    /**
       Demultiplex one message from the given source, if any is
       available. The actual handling of the message is delegated to
       the abstract Handle() method which is implemented in
       subclasses of Muldex. For example, use MdxDispatcher, to which
       you can register a (unique) mapping from proxy_msg_id to
       functors that have the same signature as Handle().
    */
    muldex_status DemuxOne(Source * source);
    
    /**
       THIS DOC IS OUT OF DATE
       \todo Update this doc!!!!!
       
       Demultiplex messages until we hit a condition. Conditions can be:
       - muldex_status::TRY_AGAIN if the source is (temporarily) dry
       - one of the muldex_status error codes
       - we have processed max_nmsg messages, provided max_nmsg>=0
       
       \note
       - Use max_nmsg<0 to say "as many as possible". This effectively
         enters an infinite demux loop (until we hit an error). This
         might be fine if all your processing happens inside
         Muldex::Handle(), but you probably should use DemuxWait() to
         insert a small sleep after attempting to read from a dry
         source.
       - max_nmsg==0 would be a kinda expensive NO-OP which returns
         SUCCESS but does not do much
       - If you want to keep waiting on a non-blocking sink until
         the data has been transmitted, use DemuxWait().
    */
    muldex_status Demux(Source * source, int max_nmsg,
			/** optional: if non-null it will contain the
			    number of messages handled during this
			    call */
			int * msgcount);
    
    /**
       Like Demux() but instead of returning on TRY_AGAIN, it'll sleep
       and then try again for you, for as long as min_nmsg has not
       been reached. As soon as max_nmsg messages have been handled,
       it returns even if there might be more data waiting in the
       source. If max_nmsg is smaller than zero, we enter an infinite
       loop a bit like calling Demux() with max_nmsg==-1. This can be
       used to implement data-driven processing using the Handle()
       method.
    */
    muldex_status DemuxWait(Source * source, int min_nmsg, int max_nmsg,
			    /** amount of time to usleep() between
				attempts to Demux() */
			    unsigned long usecsleep,
			    /** optional, set it to zero if you don't
				want '....' output */
			    std::ostream * progress,
			    /** optional: if non-null it will contain
				the number of messages handled during
				this call */
			    int * msgcount);
    
    /** \note If you want to keep waiting on a non-blocking sink until
	the data has been transmitted, use MuxWait(). */
    muldex_status Mux(Sink * sink, Proxy & proxy);
    
    /** Attempts to Mux() until the data has been sent or an error
	occurs. */
    muldex_status MuxWait(Sink * sink, Proxy & proxy,
			  /** amount of time to usleep() between
			      attempts to Mux() */
			  unsigned long usecsleep,
			  /** optional, set it to zero if you don't
			      want '....' output */
			  std::ostream * progress);
    
    /** \return 0 on success. It gets stored in the
	muldex_status::handle field of the Demux() return value. The
	meaning of non-zero return values depends on the subclass. */
    virtual int Handle(unique_id_t msg_id, BufferAPI const & buf) = 0;
    
  protected:
    endian_mode_t m_endian_mode;
    Buffer m_buf;
  };
  
  
  /** Just an interface for things that want to get notified when a
      message has arrived. Used from within MdxHandler. */
  class MdxListener
  {
  public:
    virtual ~MdxListener();
    
    /** Default implementation returns 0. In most cases, you just
	worry about messages once they have been completely unpacked,
	so you only need to implement HandleMessagePayload(). However,
	in case you need to prepare for the payload before it arrives,
	you can override HandleMessageHeader(). For example, suppose
	you want to directly unpack matrix messages into an instance
	that depends on the header information: you would use
	HandleMessageHeader() to adjust a pointer to the matrix. */
    virtual int HandleMessageHeader(unique_id_t msg_id);
    
    virtual int HandleMessagePayload(unique_id_t msg_id) = 0;
  };
  
  
  /**
     Abstract message handling functor. You register instances of this
     handler using MdxDispatcher::SetHandler(). Then,
     MdxDispatcher::Handle() will call the appropriate mdx_handler
     when it receives a message with matching ID. Optionally, you can
     have it call mdx_listener::HandleMsg() on an interested party.
  */
  class MdxHandler
  {
  public:
    MdxHandler(/** Set this flag to true if you want MdxDispatcher to
		   delete this functor for you when it is no longer
		   needed. This is the typical usage scenario and is
		   more efficient than some generic smart pointer
		   implementation. */
	       bool dispatcher_owned,
	       /** Optional pointer to a listener that gets notified
		   each time a message has been successfully
		   handled. */
	       MdxListener * listener);
    
    virtual ~MdxHandler();
    
    /**
       Delegates to DoHandleMessageHeader() then
       DoHandleMessagePayload(). If there is a listener, upon success
       of each delegated call it notifies
       MdxListener::HandleMessageHeader() then
       MdxListener::HandleMessagePayload().
       
       \return 0 on success.
    */
    int HandleMessage(unique_id_t msg_id, BufferAPI const & buf, endian_mode_t endian_mode);
    
    bool const dispatcher_owned;

  protected:
    /**
       Implement your message header handler here. The msg_id allows
       you to intercept more than one message ID, should that fit your
       needs.
       
       \note You can probably just use a ProxyHandler instead of
       subclassing MdxHandler yourself.
       
       \return 0 on success. Note that MdxDispatcher::Handle() returns
       -1 if there is no handler for a given msg_id, so maybe you
       should avoid returning -1 from your DoHandleMessageHeader() in order to
       avoid confusion.
    */
    virtual int DoHandleMessageHeader(unique_id_t msg_id, BufferAPI const & buf,
				      endian_mode_t endian_mode) = 0;
    
    /**
       Implement your message payload handler here. The msg_id allows
       you to intercept more than one message ID, should that fit your
       needs.
       
       \note You can probably just use a ProxyHandler instead of
       subclassing MdxHandler yourself.
       
       \return 0 on success. Note that MdxDispatcher::Handle() returns
       -1 if there is no handler for a given msg_id, so maybe you
       should avoid returning -1 from your DoHandleMessagePayload() in order to
       avoid confusion.
    */    
    virtual int DoHandleMessagePayload(unique_id_t msg_id, BufferAPI const & buf,
				       endian_mode_t endian_mode) = 0;
    
  private:
    MdxListener * m_listener;
  };
  
  
  /**
     Callback-based Muldex implementation. Simply register a
     mdx_handler for each type of message that you're interested in.
  */
  class MdxDispatcher
    : public Muldex
  {
  public:
    MdxDispatcher(int bufsize, int max_bufsize,
		  /** if networking, use ENDIAN_DETECT, otherwise
		      ENDIAN_NEVER_SWAP to avoid swapping when staying
		      within the same computer */
		  endian_mode_t endian_mode);
    
    /**
       \note Deletes any registered mdx_handler instances which have
       their dispatcher_owned flag set to true.
    */
    ~MdxDispatcher();
    
    void SetHandler(unique_id_t msg_id,
		    /** Note that you can set
			mdx_handler::dispatcher_owned to true if you
			want to "fire and forget" a newly created
			mdx_handler. */
		    MdxHandler * handler);
    
    /**
       Delegate handling to user-defined mdx_handler subclasses.
       
       \return -1 if there is no handler for that msg_id, or the value
       returned by mdx_handler::operator() (which should return 0 on
       success). For example, if you use ProxyHandler, then you will
       receive -2 if the header could not be de-serialized, and -2 if
       the payload could not be de-serialized.
    */
    virtual int Handle(unique_id_t msg_id, BufferAPI const & buf);
    
  private:
    typedef std::map<unique_id_t, MdxHandler*> handler_map;
    handler_map m_handler;
  };
  
  
  /**
     All Proxy subclasses can be handled in the same way by this
     handler. It simply unpacks the proxy and tells the registered
     MdxListener about it. So you can simply subclass MdxListener,
     instantiate a couple of ProxyHandler instances (each with a
     reference to one of the Proxy subclasses which you are interested
     in), and get notified each time one of them is dropped in your
     mailbox, so to speak.  
  */
  class ProxyHandler
    : public MdxHandler
  {
  public:
    ProxyHandler(/** Name of this handler, mostly for debugging and
		     error messages. */
		 std::string const & name,
		 /** The proxy that this handler fills in for you. */
		 Proxy & proxy,
		 /** Whether this instance should be automatically
		     deleted by the MdxMuldex destructor. */
		 bool dispatcher_owned,
		 /** The interested party, if any (probably the owner
		     of the proxy in question). */
		 MdxListener * listener);
    
    std::string const name;
    
  protected:
    /**
       Unpack the header from the buffer into the registered proxy.
       
       \return 0 on success, -2 if Proxy::UnpackHeader() failed. We do
       not return -1 because that is used by MdxDispatcher::Handle()
       when there is no registered handler.
    */
    virtual int DoHandleMessageHeader(unique_id_t msg_id,
				      BufferAPI const & buf,
				      endian_mode_t endian_mode);
    
    /**
       Unpack the payload from the buffer into the registered proxy.
       
       \return 0 on success, -3 if Proxy::UnpackPayload() failed. We
       do not return -1 because that is used by
       MdxDispatcher::Handle() when there is no registered handler,
       nor -2 because that is used by DoHandleMessageHeader() if
       Proxy::UnpackHeader() failed.
    */
    virtual int DoHandleMessagePayload(unique_id_t msg_id,
				       BufferAPI const & buf,
				       endian_mode_t endian_mode);
    
    Proxy & m_proxy;
  };
  
}

#endif // WBCNET_MULDEX_HPP
