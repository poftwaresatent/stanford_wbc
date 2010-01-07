#include <wbcnet/misc/DelayHistogram.hpp>
#include <mqueue.h>
#include <signal.h>
#include <err.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

using namespace std;

static bool go(true);

static void handle(int signum)
{
  warnx("%s", sys_siglist[signum]);
  if (go) {
    go = false;
  }
  else {
    errx(EXIT_FAILURE, "oopsie");
  }
}

int main(int argc, char ** argv)
{
  if (3 > argc)
    errx(EXIT_FAILURE, "usage: %s wr_name rd_name", argv[0]);
  
  vector<int> signums;
  signums.push_back(SIGHUP);
  signums.push_back(SIGINT);
  signums.push_back(SIGTERM);
  signums.push_back(SIGPIPE);
  for (size_t ii(0); ii < signums.size(); ++ii)
    if (SIG_ERR == signal(signums[ii], handle))
      err(EXIT_FAILURE, "signal(%s)", sys_siglist[signums[ii]]);
  
  wbcnet::DelayHistogram dhist(3, 10, 0, 0.5, 5000);
  dhist.SetName(0, "send");
  dhist.SetName(1, "receive");
  dhist.SetName(2, "total");
  
  static int const nbytes(1024);
  char buffer[nbytes];
  for (int ii(0); ii < nbytes; ++ii)
    buffer[ii] = ii % 256;
  
  mq_attr attr;
  attr.mq_curmsgs = 0;
  attr.mq_flags = O_NONBLOCK;
  attr.mq_msgsize = 1024;
  attr.mq_maxmsg = 1;
  
  string const wr_name(string("/") + argv[1]);
  mqd_t wr;
  wr = mq_open(wr_name.c_str(), O_CREAT | O_NONBLOCK | O_WRONLY, S_IRUSR | S_IWUSR, &attr);
  if (-1 == wr)
    err(EXIT_FAILURE, "mq_open %s", wr_name.c_str());
  
  string const rd_name(string("/") + argv[2]);
  mqd_t rd;
  rd = mq_open(rd_name.c_str(), O_CREAT | O_NONBLOCK | O_RDONLY, S_IRUSR | S_IWUSR, &attr);
  if (-1 == rd)
    err(EXIT_FAILURE, "mq_open %s", rd_name.c_str());
  
  while (go) {
    dhist.Start(2);
    
    dhist.Start(0);
    if ((0 != mq_send(wr, buffer, nbytes, 0)) && (EAGAIN != errno)) {
      go = false;
      warn("mq_send %s", wr_name.c_str());
    }
    dhist.Stop(0);
    
    if (go) {
      dhist.Start(1);
      if ((-1 == mq_receive(rd, buffer, nbytes, 0)) && (EAGAIN != errno)) {
	go = false;
	warn("mq_receive %s", rd_name.c_str());
      }
      dhist.Stop(1);
    }
    
    dhist.Stop(2);
    dhist.CheckDumpTable(stdout);
    
    usleep(1000);		// 1kHz
  }
  
  if (0 != mq_close(wr))
    warn("mq_close %s", wr_name.c_str());
  if (0 != mq_unlink(wr_name.c_str()))
    warn("mq_unlink %s", wr_name.c_str());
  
  if (0 != mq_close(rd))
    warn("mq_close %s", rd_name.c_str());
  if (0 != mq_unlink(rd_name.c_str()))
    warn("mq_unlink %s", rd_name.c_str());
  
  dhist.DumpTable(stdout);
  printf("byebye\n");
}
