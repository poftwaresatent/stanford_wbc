/*
 * Copyright (c) 2009 Stanford University
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
   \file wbcmodel.cpp
   \author Roland Philippsen
*/

#include <wbc/bin/attributes.hpp>
#include <wbcnet/log.hpp>
#include <wbc/bin/ModelProcess.hpp>
#include <wbc/bin/options.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/Plugin.hpp>
#include <wbc/core/TaskModelFactory.hpp>
#include <wbc/core/TaskModelBase.hpp>
#include <iostream>
#include <sstream>
#include <cstring>
#include <err.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


using namespace wbc;
using namespace std;


static options opt;
static attributes * attr;
static ModelProcess * proc;
static TaskModelBase * task_model;


static void handle(int signum)
{
  errx(SIGTERM == signum ? EXIT_SUCCESS : EXIT_FAILURE,
       "signal %d (%s)",
       signum, sys_siglist[signum]);
}


static void cleanup()
{
  delete proc;
  delete task_model;
  delete attr;
}


int main(int argc, char*argv[])
{
  if (0 != atexit(cleanup))
    err(EXIT_FAILURE, "atexit()");
  if (SIG_ERR == signal(SIGTERM, handle))
    err(EXIT_FAILURE, "signal(SIGTERM, ...)");
  if (SIG_ERR == signal(SIGPIPE, handle))
    err(EXIT_FAILURE, "signal(SIGPIPE, ...)");
  
  opt.parse(argc, argv);
  
  wbcnet::configure_logging();
  wbcnet::manual_logging_verbosity(opt.verbosity);
  
  proc = new wbc::ModelProcess();
  try {
    attr = attributes::create(opt);
    static wbcnet::endian_mode_t const endian_mode(wbcnet::ENDIAN_DETECT);
    task_model = attr->extensions->task_model_registry->Create(opt.motion_type,
							       attr->robmodel->branching(),
							       endian_mode);
    wbc::ModelImplementation *
      imp(new wbc::ModelImplementation(task_model, attr->robmodel, attr->behavior));
    size_t const ndof(attr->robmodel->branching()->numJoints());
    size_t contact_nrows, contact_ncols;
    attr->robmodel->getForceDimension(contact_nrows, contact_ncols);
    // XXXX to do: the last 4 parameters can be retrieved from robmodel inside the Init() method
    proc->Init(imp, true, *attr->netcfg, ndof, /*nvel=ndof*/ ndof, contact_nrows, contact_ncols);
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION %s", ee.what());
  }
  
  warnx("entering loop");
  timeval t0, t1;
  while (true) {
    gettimeofday( &t0, NULL ); 
    if ( ! proc->Step())
      errx(EXIT_FAILURE, "wbc::ModelProcess::Step() failed");
    gettimeofday( &t1, NULL );     
  }
}
