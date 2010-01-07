#include <wbcnet/misc/StreamBufMgr.hpp>
#include <iostream>
#include <deque>
#include <err.h>
#include <stdlib.h>
#include <unistd.h>

using namespace wbcnet;
using namespace std;

class DelayLine
{
public:
  size_t const minsize;
  size_t const lineskip;
  
  DelayLine(size_t _minsize, size_t _lineskip): minsize(_minsize), lineskip(_lineskip) {
    while (line.size() < minsize)
      line.push_front('.');
  }
  
  void Send() {
    outbuf.push_front('o');
  }
  
  bool Receive() {
    if (inbuf.empty())
      return false;
    inbuf.pop_back();
    return true;
  }
  
  void Tick() {
    while (line.size() < minsize)
      line.push_front('.');
    ++tick;
    if (0 != (tick % lineskip))
      return;
    if (outbuf.empty())
      line.push_front('.');
    else {
      outbuf.pop_back();
      line.push_front('*');
    }
    char const in(line.back());
    line.pop_back();
    if ('.' != in)
      inbuf.push_front('i');
  }
  
  void Dump() const {
    deque<char>::const_iterator ii;
    for (ii = outbuf.begin(); ii != outbuf.end(); ++ii)
      cout << *ii;
    cout << "\t";
    for (ii = line.begin(); ii != line.end(); ++ii)
      cout << *ii;
    cout << "\t";
    for (ii = inbuf.begin(); ii != inbuf.end(); ++ii)
      cout << *ii;
  }
  
  deque<char> inbuf, line, outbuf;
  size_t tick;
};


static void simul_sync()
{
  DelayLine foo(10, 2);
  foo.Send();
  while (true) {
    if (foo.Receive())
      foo.Send();
    foo.Tick();
    foo.Dump();
    cout << "\n";
    {
      string blah;
      getline(cin, blah);
      if ( ! cin)
	errx(EXIT_FAILURE, "simul_sync(): broken cin");
      if ((!blah.empty()) && ('q' == blah[0]))
	errx(EXIT_SUCCESS, "bye bye");
    }
  }
}


static void test_mgr(ostream * dbgos, bool prompt)
{
  StreamBufMgr mgr(0, 5, 4, 15, 25);
  DelayLine net(30, 2);
  
  bool please_send(true);
  while (true) {
    
    if (please_send) {
      net.Send();
      mgr.IncOutgoing();
    }
    
    cout << "lag " << mgr.ComputeLag() << ":\t";
    net.Dump();
    
    StreamBufMgr::mode_t mode;
    please_send = mgr.CheckSend(&mode);
    switch (mode) {
    case StreamBufMgr::FULLSPEED:
      cout << "\tfull speed\n";
      break;
    case StreamBufMgr::TRICKLE:
      cout << "\ttrickle\n";
      break;
    case StreamBufMgr::STOP:
      cout << "\tSTOP\n";
      break;
    default:
      cout << "\t????\n";
    }
    
    if (net.Receive())
      mgr.IncIncoming();
    
    net.Tick();
    
    if (prompt) {
      string blah;
      getline(cin, blah);
      if ( ! cin)
	errx(EXIT_FAILURE, "simul_sync(): broken cin");
      if ((!blah.empty()) && ('q' == blah[0]))
	errx(EXIT_SUCCESS, "bye bye");
    }
    else
      usleep(100000);
  }
}


int main(int argc, char ** argv)
{
  //simul_sync();
  test_mgr(0, false);
}
