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
   \file UserProcess.cpp
   \author Roland Philippsen
*/

#include "UserProcess.hpp"
#include <wbcnet/message_id.hpp>
#include <wbcnet/NetConfig.hpp>
#include <wbcnet/log.hpp>
#include <iostream>
#include <sstream>
#include <limits>
#include <cmath>

#ifdef WIN32
#define M_PI 3.141592653
#endif

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

using namespace std; 

#ifdef HAVE_CURSES
# include <curses.h>
#endif // HAVE_CURSES


#ifdef HAVE_XMLRPC

#include "XMLRPCDirectoryServer.hpp"
#include "XmlRpcServer.h"
#include <cstring>
#include <signal.h>
#include <err.h>

static wbc::XMLRPCDirectoryServer * xmlrpc_directory(0);

static void handle_SIGTSTP(int signum)
{
  // could check if we really got SIGTSTP...
  if (xmlrpc_directory) {
    warnx("handle_SIGTSTP(): exiting xmlrpc_directory");
    xmlrpc_directory->GetServer()->exit();
  }
  else
    warnx("handle_SIGTSTP(): no xmlrpc_directory found");
}

#endif // HAVE_XMLRPC


static bool ncurses_active(false);


namespace {
  
  class MyServiceTransaction: public wbc::ServiceTransaction {
  public:
    wbc::UserProcess * m_user;
    
    explicit MyServiceTransaction(wbc::UserProcess * user): m_user(user) {}
    
    virtual wbcnet::msg::Service * GetRequest()
    { return &m_user->m_user_request; }
    
    virtual wbcnet::msg::Service * GetReply()
    { return &m_user->m_user_reply; }
    
    virtual void SendWaitReceive() {
      m_user->EnqueueMessage(m_user->m_channel, GetRequest(), false, false);
      m_user->SendWait(10000);
      GetReply()->code.SetNElements(0);
      GetReply()->matrix.SetSize(0, 0);
      m_user->ReceiveWait(10000, 1);
    }
  };
  
}


namespace wbc {
  
  
  bool UserProcess::
  Step() throw(std::exception)
  {
    static const int buflen(1024);
    static char buffer[buflen];
    
    cout << "user> " << flush;
    if ( ! cin.getline(buffer, buflen)) {
      cout << "FATAL ERROR reading standard input\n";
      return false;
    }
    istringstream line(buffer);
    string token;
    line >> token;
    if ( ! line) {
      cout << "SYNTAX ERROR reading first token\n";
      return true;
    }
    try {

      if ("pos" == token) {
	m_user_request.InitGetPositions();
	cout << "wbc::UserProcess::Step(): sending user request\n";
	m_user_request.Dump(cout, "    ");
        EnqueueMessage(m_channel, &m_user_request, false, false);
        SendWait(10000);
        ReceiveWait(10000, 1);
        return true;
      }

      if ("vel" == token) {
	m_user_request.InitGetVelocities();
	cout << "wbc::UserProcess::Step(): sending user request\n";
	m_user_request.Dump(cout, "    ");
        EnqueueMessage(m_channel, &m_user_request, false, false);
        SendWait(10000);
        ReceiveWait(10000, 1);
        return true;
      }

      if ("endpos" == token) {
	m_user_request.InitGetLinkTransform("End_Effector"); // would be nice to give custom name...
	cout << "wbc::UserProcess::Step(): sending user request\n";
	m_user_request.Dump(cout, "    ");
        EnqueueMessage(m_channel, &m_user_request, false, false);
        SendWait(10000);
        ReceiveWait(10000, 1);
        return true;
      }
      
      if ("tau" == token) {
	m_user_request.InitGetTorques();
	cout << "wbc::UserProcess::Step(): sending user request\n";
	m_user_request.Dump(cout, "    ");
        EnqueueMessage(m_channel, &m_user_request, false, false);
        SendWait(10000);
        ReceiveWait(10000, 1);
        return true;
      }

      if ("go" == token) {
        InteractiveGoalRequest();
	cout << "wbc::UserProcess::Step(): sending user request\n";
	m_user_request.Dump(cout, "    ");
        EnqueueMessage(m_channel, &m_user_request, false, false);
        SendWait(10000);
        ReceiveWait(10000, 1);
        return true;
      }
      
      if ("float" == token) {
	m_user_request.InitFloat();
	cout << "wbc::UserProcess::Step(): sending user request\n";
	m_user_request.Dump(cout, "    ");
        EnqueueMessage(m_channel, &m_user_request, false, false);
        SendWait(10000);
        ReceiveWait(10000, 1);
        return true;
      }
      
      if ("activate" == token) {
	m_user_request.InitActivate();
	cout << "wbc::UserProcess::Step(): sending user request\n";
	m_user_request.Dump(cout, "    ");
        EnqueueMessage(m_channel, &m_user_request, false, false);
        SendWait(10000);
        ReceiveWait(10000, 1);
        return true;
      }
      
      if ("setgoal" == token) {
        double goal[7];
        line >> goal[0] >>goal[1] >>goal[2] >>goal[3] >>goal[4] >>goal[5] >>goal[6] ;
        if ( ! line) {
          cout << "SYNTAX ERROR reading goal pos and orientation\n";
          return true;
        }
        m_user_request.InitSetGoal(goal, 7);
        EnqueueMessage(m_channel, &m_user_request, false, false);
        SendWait(10000);
        ReceiveWait(10000, 1);
        return true;
      }     
      
      if ("setgains" == token) {
        double gains[4];
        line >> gains[0] >>gains[1] >>gains[2] >>gains[3];
	if ( ! line) {
          cout << "SYNTAX ERROR reading gains\n";
          return true;
        }
        m_user_request.InitSetGains(gains, 4);
        EnqueueMessage(m_channel, &m_user_request, false, false);
        SendWait(10000);
        ReceiveWait(10000, 1);
        return true;
      }        
      
      if ("b?" == token) {
        cout << "available behaviors:\n";
        size_t ii(0);
        for (listing_t::const_iterator bb(GetBehaviorList().begin()), bend(GetBehaviorList().end());
	     bb != bend; ++bb, ++ii)
          cout << "  [" << ii << "] " << *bb << "\n";
        return true;
      }

      if (("b" == token) || ("B" == token)) {
        int bnum;
        line >> bnum;
        if ( ! line) {
          cout << "SYNTAX ERROR reading behavior number\n";
          return true;
        }
        if (0 > bnum) {
          cout << "ERROR behavior number " << bnum << " is negative\n";
          return true;
        }
        int const nbehaviors(static_cast<int>(GetBehaviorList().size()));
        if (nbehaviors <= bnum) {
          cout << "ERROR behavior number " << bnum << " is too large (max "
	       << nbehaviors - 1 << ")\n";
          return true;
        }
        if (("b" == token) && (bnum == m_task_spec.behaviorID)) {
          cout << "already running behavior " << bnum << ", skipping (use capital B to override)\n";
          return true;
        }
        ++m_task_spec.requestID;
        m_task_spec.behaviorID = bnum;
        EnqueueMessage(m_channel, &m_task_spec, false, false);
        SendWait(10000);
        // do NOT ReceiveWait(), because task specs are fire-and-forget
        return true;
      }
      
      if ("r" == token) {
	m_user_request.InitToggleRecorder();
        SendWait(10000);
        ReceiveWait(10000, 1);
        return true;
      }

      if ("k" == token) {
        InteractiveKeyPressLoop();
        return true;
      }

      if ("xmlrpc" == token) {
        XmlRpcLoop();
        return true;
      }
      
      cout << "SYNTAX ERROR: unknown command \"" << token << "\"\n"
	"\n"
	" generic commands:\n"
	"  pos      -  request current joint positions\n"
	"  vel      -  request current joint velocities\n"
	"  tau      -  request current command torques\n"
	"  endpos   -  show end effector position end orientation\n"
	"  go       -  enter and send goal\n"
	"  b?       -  list available behaviors\n"
	"  b   <N>  -  switch to behavior number <N>\n"
	"  r        -  toggle recorder state (writes them to file after the 2nd time)\n"
	"  k        -  enter interactive_key_press mode (use 'q' to leave it again)\n"
	"  xmlrpc   -  spawn XMLRPC loop, runs until SIGTSTP (i.e. Ctrl-Z)\n"
	"\n"
	" experimental and behavior-specific commands:\n"
	"  float    -  send a FLOAT request (not understood by all behaviors)\n"
	"  activate -  send an ACTIVATE request (not understood by all behaviors)\n"
	"  setgoal  -  enter goal position (hardcoded 7-D vector -- x y z axis angle)\n"
	"  setgains -  enter control gains (type (1(x) or 2(y) or 3(yaw)) kp kd ki)\n";
    }
    catch (exception const & ee) {
      cout << "EXCEPTION " << ee.what() << "\n";
      return false;
    }
    return true;
  }
  
  
  void UserProcess::
  InteractiveKeyPressLoop()
  {
#ifndef HAVE_CURSES

    cout <<
      "Sorry, but curses headers were not found on your system,"
      "so the UserProcess::InteractiveKeyPressLoop() is not available.\n"
      "If you want it, install curses (e.g. libncurses5-dev or so), wipe\n"
      "away your build dir, and rebuild.\n";

#else // HAVE_CURSES (rest of method)

    if (ncurses_active) 		// "never" happens though
      endwin();
    
    // race condition with ncurses_active flag, which is not a real mutex...
    initscr();
    ncurses_active = true;
    
    cbreak();
    noecho();
    nonl();
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);
    
    ostringstream tmp_error_os;
    
    bool ok(true);
    if (ERR == mvaddstr(0, 0,
			"---> interactive_key_press mode <---\n"
			"each key code is sent to the servo process and echoed here\n"
			"press 'q' or 'Q' to quit this mode\n")) {
      ok = false;
      tmp_error_os << "mvaddstr() failed for title\n";
    }
    
    while (ok) {
      
      int const ch(getch());
      if (ERR == ch) {
	usleep(10000);
	continue;
      }
      if (('q' == ch) || ('Q' == ch))
	break;
      
      m_user_request.InitKeyPress(-1, ch);
      EnqueueMessage(m_channel, &m_user_request, false, false);
      ostringstream os_status;
      os_status << "key code: " << ch << "\n"
		<< "  sending request:\n";
      m_user_request.Dump(os_status, "    ");
      try {
	SendWait(10000);
	ReceiveWait(10000, 1);
	os_status << "  received reply:\n";
	m_user_reply.Dump(os_status, "    ");
	if (m_user_reply.code.NElements() == 0) {
	  os_status << "weird, no status code in reply\n";
	}
	else {
	  os_status << "reply status: " << m_user_reply.code[0] << " "
		    << wbcnet::srv_result_to_string(m_user_reply.code[0]) << "\n";
	}
      }
      catch (exception const & ee) {
	ok = false;
	tmp_error_os << "EXCEPTION during send or receive\n  " << ee.what() << "\n";
	os_status << "EXCEPTION during send or receive\n  " << ee.what();
	break;
      }
      
      if (ERR == mvaddstr(5, 0, os_status.str().c_str())) {
	ok = false;
	tmp_error_os << "mvaddstr() failed\n";
	break;
      }
      if (ERR == refresh()) {
	ok = false;
	tmp_error_os << "refresh() failed\n";
	break;
      }
      
    }
    
    // another race condition...
    endwin();
    ncurses_active = false;
    
    if (logger->isErrorEnabled() && ( ! tmp_error_os.str().empty()))
      LOG_ERROR (logger, "errors during ncurses mode:\n" << tmp_error_os.str());
    
#endif // HAVE_CURSES
  }
  
  
  UserProcess::
  UserProcess()
    : Process("user", 0, -1, wbcnet::ENDIAN_DETECT),
      m_channel(0),
      m_user_request(wbcnet::msg::USER_REQUEST),
      m_user_reply(wbcnet::msg::USER_REPLY),
      m_task_spec(wbcnet::msg::TASK_SPEC),
      m_directory_client(0)
  {
  }
  
  
  UserProcess::
  ~UserProcess()
  {
    delete m_channel;
    delete m_directory_client;
  }
  
  
  void UserProcess::
  Init(wbcnet::NetConfig const & netconf) throw(std::exception)
  {
    if (m_channel) {
      // We could also just return, but maybe people think they can
      // re-configure us by calling Init() with a different NetConfig,
      // but that's trickier than it might seem because of the
      // incoming and outgoing message queues.
      throw runtime_error("wbc::UserProcess::Init(): already initialized");
    }
    
    m_directory_client = new DirectoryCmdClient(new MyServiceTransaction(this), true);
    
    m_channel = netconf.CreateChannel(wbcnet::NetConfig::USER, wbcnet::NetConfig::SERVO);
    if ( ! m_channel)
      throw runtime_error("wbc::UserProcess::Init(): could not create channel");
    AddSink(m_channel, 100);
    AddSource(m_channel, 100);
    
    CreateHandler(wbcnet::msg::USER_REPLY, "UserReply", &m_user_reply);
    
    m_task_spec.requestID = 0;
    m_task_spec.behaviorID = numeric_limits<uint8_t>::max();
  }
  
  
  int UserProcess::
  HandleMessagePayload(wbcnet::unique_id_t msg_id)
  {
    if (ncurses_active) {
      return 0;
    }
    
    cout << "wbc::UserProcess::HandleMessagePayload():\n"
	 << "  message:\n";
    m_user_reply.Dump(cout, "    ");
    
    if (wbcnet::msg::USER_REPLY != msg_id) {
      cout << "  ERROR unknown message ID " << (int) msg_id << "\n";
      return 0;
    }
    
    if (m_user_request.requestID != m_user_reply.requestID) {
      cout << "  WARNING requestID mismatch: expected " << (int) m_user_request.requestID
	   << " but got " << (int) m_user_reply.requestID << "\n";
    }
    
    if (1 < m_user_reply.nCodes) {
      cout << "  WARNING no status info in message\n";
    }
    else {
      cout << "  status " << m_user_reply.code[0] << ": "
	   << wbcnet::srv_result_to_string(m_user_reply.code[0]) << "\n";
    }
    
    return 0;
  }
  
  
  void UserProcess::
  InteractiveGoalRequest()
  {
    static const int buflen(1024);
    char buffer[buflen];
    static char const * field[] =
      { "x [m]", "y [m]", "z [m]", "psi [deg]", "theta [deg]", "phi [deg]" };
    double goal[6];
    
    cout << "interactive_goal_request()\n";
    for (size_t ii(0); ii < 6; ++ii) {
      cout << "  enter " << field[ii] << ": ";
      if ( ! cin.getline(buffer, buflen)) {
	cout << "ERROR reading standard input\n";
	return;
      }
      istringstream line(buffer);
      line >> goal[ii];
      if ( ! line) {
	cout << "ERROR reading field\n";
	--ii;
	continue;
      }
    }
    // deg -> rad conversion
    for (size_t ii(3); ii < 6; ++ii)
      goal[ii] *= M_PI / 180;
    
    m_user_request.InitSetGoal(goal, 6);
  }
  
  
  void UserProcess::
  Cleanup()
  {
#ifdef HAVE_CURSES
    if (ncurses_active) {
      endwin();
      cout << "cleanup up curses\n";
    }
#endif // HAVE_CURSES
    
#ifdef HAVE_XMLRPC
    if (xmlrpc_directory) {
      cout << "cleanup up XmlRpc\n";
      handle_SIGTSTP(SIGTSTP);
      usleep(500000);
      delete xmlrpc_directory;
      xmlrpc_directory = 0;
    }
#endif // HAVE_XMLRPC
    
    cout << "see you later\n";
  }
  
  
  void UserProcess::
  XmlRpcLoop()
  {
#ifndef HAVE_XMLRPC
    cout << "Sorry, XMLRPC not available in this build\n"
	 << "  You need xmlrpc++, which you can get by going into the wbc/xmlrpc++ directory\n"
	 << "  and typing './buildme.sh' at the prompt.  At the next configure / build it\n"
	 << "  should get picked up\n";
#else // HAVE_XMLRPC
    
    cout << "Installing new signal handler for SIGTSTP\n";
    struct sigaction sig;
    memset(&sig, 0, sizeof(sig));
    sig.sa_handler = handle_SIGTSTP;
    struct sigaction save_sig;
    memset(&save_sig, 0, sizeof(save_sig));
    if (0 != sigaction(SIGTSTP, &sig, &save_sig)) {
      warn("wbc::UserProcess::XmlRpcLoop(): sigaction()");
      return;
    }
    
    cout << "Spawning XmlRpc server on port 8080.\n"
	 << "  Press Ctrl-Z to quit.\n";
    xmlrpc_directory = new XMLRPCDirectoryServer(m_directory_client);
    xmlrpc_directory->RunForever(8080);

    cout << "XmlRpc server has exited.\n";
    usleep(200000);		// needed?
    delete xmlrpc_directory;
    xmlrpc_directory = 0;
    
    cout << "Restoring old signal handler for SIGTSTP\n";
    if (0 != sigaction(SIGTSTP, &save_sig, 0))
      warn("wbc::UserProcess::XmlRpcLoop(): sigaction()");
    
#endif // HAVE_XMLRPC
  }
  
  
  listing_t const & UserProcess::
  GetBehaviorList() const throw(std::exception)
  {
    if (m_lazy_behavior_list.empty()) {
      wbcnet::srv_result_t const stat(m_directory_client->ListBehaviors(m_lazy_behavior_list));
      if (wbcnet::SRV_SUCCESS != stat)
	throw runtime_error("wbc::UserProcess::GetBehaviorList():\n"
			    "  m_directory_client->ListBehaviors() failed: "
			    + string(wbcnet::srv_result_to_string(stat)));
    }
    return m_lazy_behavior_list;
  }

}
