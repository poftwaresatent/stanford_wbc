Stanford Whole-Body Control Framework
=====================================

    Core project copyright:
        Copyright (C) 1997-2010 Stanford University. All rights reserved.

    Other copyright holders:
        Copyright (C) 2005 Arachi Inc. All rights reserved.
        Copyright (C) 2008-2009 Roland Philippsen. All rights reserved.
    	Copyright (C) 2000-2006 Lee Thomason. All rights reserved.

    Core project license:
        GNU Lesser General Public License, Version 3

    Other licenses:
        MIT license
        BSD license (new and simplified)
        zlib/libpng license

The Whole-Body Control framework developed at the Stanford Robotics
and AI Lab http://ai.stanford.edu/groups/manips/ provides advanced
control for human-centered robotics and mobile manipulation. This
project provides a framework for developing robot behaviors that use
operational-space hierarchical task decompositions, based on the
thesis of Luis Sentis
http://www.me.utexas.edu/directory/faculty/sentis/luis/. The software
provides the foundations for flexibly controlling motions that
simultaneously take into account multiple objectives, such as
achieving end-effector positions while avoiding obstacles and choosing
appropriate postures for redundant degrees of freedom.


Download
--------

Clone our GIT repository or download a tarball.

* Original GIT repos on Sourceforge: this contains several
  sub-projects that could use a bit of cleanup and are thus not easily
  reused by others. It is recommended that you try our Github clone
  (see next item) instead.

        git clone git://stanford-wbc.git.sourceforge.net/gitroot/stanford-wbc/stanford-wbc.git

* Minimal clone of Github: this repos contains the same code, but many
  sub-projects have been moved into an attic area from which they will
  re-emerge when they get cleaned up and made ready for general
  consumption. So you will only be exposed to things that have reached
  higher levels of maturity and reusability.

        git://github.com/poftwaresatent/stanford_wbc.git

* Grab a tarball from Sourceforge:
  http://sourceforge.net/projects/stanford-wbc/files/ contains
  official releases of the Stanford-WBC project. Up to version 0.8 the
  releases are based on the Sourceforge repository.


Build
-----

The dependencis for the minimal version from Github are:

* a C++ compiler (we use GCC for development http://gcc.gnu.org/)
* CMake cross-platform build system version 2.6 http://www.cmake.org/
* Eigen matrix library version 2 http://eigen.tuxfamily.org/
* Google Testing Framework http://code.google.com/p/googletest/
  _(optional but highly recommended)_

The full version from Sourceforge has some additional dependencies:
GNU ncurses, LAPACK and BLAS, expat, log4cxx (optional), XmlRpc++
(optional).

    mkdir build
    cd build
    cmake ..
    make

There are CMake variables that influence the configure step. They get
passed to `cmake` using its `-DVARIABLE=VALUE` command-line
syntax. Note that reasonable guesses are made according to your
operating system, but if you have installed some dependencies in
non-standard locations, you will have to tell CMake where to find
them.

<table>
 <tr><th>variable name</th><th>meaning</th><th>example</th></tr>
 <tr>
  <td><pre>EIGEN2_DIR</pre></td>
  <td>Path where the Eigen matrix library is installed. This will end
      up adding several directories to the header search path:
      <pre>EIGEN2_DIR</pre> itself, <pre>EIGEN2_DIR/include</pre>,
      <pre>EIGEN2_DIR/eigen2</pre>, and
      <pre>EIGEN2_DIR/include/eigen2</pre>.  Note that CMake will fail
      with an error message if it cannot find Eigen2, and <strong>you
      have to clear its cache</strong> before reconfiguring with a
      different <pre>EIGEN2_DIR</pre> setting. The easiest way to
      achieve that is to remove the entire <pre>build</pre> directory.</td>
  <td>If you installed Eigen2 underneath <pre>/home/toto/eigen2</pre>,
      then you have to pass <pre>-DEIGEN2_DIR=/home/toto/eigen2</pre>
      to the CMake command.</td>
 </tr>
 <tr>
  <td><pre>GTEST_DIR</pre></td>
  <td>Path where the Google Testing Framework is installed. This will
      end up adding two directories to the header search path, and two
      directories to the library search path: <pre>GTEST_DIR</pre>
      itself is added to the header and library path,
      <pre>GTEST_DIR/include</pre> is added to the header path, and
      <pre>GTEST_DIR/lib</pre> is added to the library path.  CMake
      will not fail when Gtest is not found (even if you explicitly
      pass <pre>GTEST_DIR</pre>), although a warning is issued. Again,
      if you add or change <pre>GTEST_DIR</pre> after a CMake run, you
      should wipe its cache (e.g. the entire <pre>build</pre>
      directory) before re-running CMake. Note that CMake will fail
      with an error if the Gtest headers are found but the library is
      not.</td>
  <td>Suppose you have ROS cturtle installed underneath
      <pre>/opt/ros/cturtle</pre>, then all you have to do is pass
      <pre>-DGTEST_DIR=/opt/ros/cturtle/ros/3rdparty/gtest/gtest</pre>
      to the CMake command for Stanford-WBC.</td>
 </tr>
</table>


Test
----

If you have Gtest (and CMake found it) then you will be able to run
unit tests after building. At the time of writing, there are two test
programs:

* Basic (very basic) tests for the TAO dynamics engine are available
  by running:

        ./tao/testTAO

* A fairly complete suite of unit tests for the joint-space dynamics
  model (which is built on TAO) is available by running:

        ./jspace/tests/testJspace

If any of these tests fail, there is a regression that should be
fixed. Please let us know:

* Check if the issue is already known, and possibly resolved in a more
  recent version, by looking at our ticketing system on the
  project TRAC on Sourceforge
  http://sourceforge.net/apps/trac/stanford-wbc/report
* You can open a new ticket on our project TRAC on Sourceforge (you
  will need a Sourceforge account to do so).
* You can send an email to `stanford-wbc-devel@lists.sourceforge.net`


Documentation
-------------

If you have Doxygen http://doxygen.org/ then you can generate code
documentation for the various sub-projects. There is a little
`rundox.sh` script in the `jspace/doc` and `tao/doc` subdirectories
(look for the same pattern in other parts of the project as well, we
might forget to update this README file when adding sub-projects and
documentation).

    cd /path/to/stanford_wbc/jspace/doc
    ./rundox
    then open html/index.html in a web browser
