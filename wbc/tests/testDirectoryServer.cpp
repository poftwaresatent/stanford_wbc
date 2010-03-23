/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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
   \file testProcess.cpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2008 Roland Philippsen, released under a BSD license.
*/

#include <wbc/bin/XMLRPCDirectoryServer.hpp>
#include "TestDirectory.hpp"
#include <stdexcept>
#include <err.h>
#include <stdlib.h>

using namespace wbc;
using namespace std;


int main(int argc, char ** argv)
{
  try {
    TestDirectory directory;
    XMLRPCDirectoryServer server(&directory);
    server.RunForever(8080);
  }
  catch (exception const & ee) {
    errx(EXIT_FAILURE, "exception: %s", ee.what());
  }
  errx(EXIT_SUCCESS, "SUCCESS");
}
