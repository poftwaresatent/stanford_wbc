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

The Whole-Body Control framework developed at the [Stanford Robotics
and AI Lab][] provides advanced control for human-centered robotics
and mobile manipulation.  This project provides a framework for
developing robot behaviors that use operational-space hierarchical
task decompositions, based on the work of many contributors over many
years, under the guidance of and in collaboration with [Oussama
Khatib][]: most notably [Jaeheung Park][], K. C. Chang, Diego Ruspini,
[Roy Featherstone][], Bob Holmberg, [François Conti][], and [Luis
Sentis][].

The core [stanford-wbc][] library provides a joint-space dynamics
model for branching structures of rigid bodies.  It has two main
components: TAO and jspace. The TAO dynamics engine, developed by
K. C. Chang and Diego Ruspini, was released under an MIT licence and
integrated into the [SimTK][] simulation framework. In the
stanford-wbc project, we forked it (from [here][tao-on-simtk]) and
added an easy-to-use facade in form of the jspace library (designed
and implemented by [Roland Philippsen][]).

[Stanford Robotics and AI Lab]: http://ai.stanford.edu/groups/manips/
[Oussama Khatib]: http://cs.stanford.edu/groups/manips/people/oussama-khatib
[Jaeheung Park]: http://plaza4.snu.ac.kr/~park73/wiki/index.php5/People
[Roy Featherstone]: http://users.cecs.anu.edu.au/~roy/
[François Conti]: http://cs.stanford.edu/groups/manips/people/francois-conti
[stanford-wbc]: https://github.com/poftwaresatent/stanford_wbc
[SimTK]: https://simtk.org/xml/index.xml
[tao-on-simtk]: https://simtk.org/home/tao_de
[Roland Philippsen]: http://cs.stanford.edu/groups/manips/people/roland-philippsen


Download
--------

Clone our GIT repository or download a tarball.

* Original GIT repos on Sourceforge: this contains several
  sub-projects that could use a bit of cleanup and are thus not easily
  reused by others. It is recommended that you try our Github clone
  (see next item) instead.

        git clone git://stanford-wbc.git.sourceforge.net/gitroot/stanford-wbc/stanford-wbc.git

* Minimal clone on Github: this repos contains the same code, but many
  sub-projects have been moved into an attic area from which they will
  re-emerge when they get cleaned up and made ready for general
  consumption. So you will only be exposed to things that have reached
  higher levels of maturity and reusability.

        git://github.com/poftwaresatent/stanford_wbc.git

* Grab a tarball from Sourceforge: the [download area][] contains
  official releases of the Stanford-WBC project. Up to version 0.8 the
  releases are based on the Sourceforge repository, later ones are
  (probably) based on the Github clone.

[download area]: http://sourceforge.net/projects/stanford-wbc/files/


Build
-----

The dependencis for the minimal version from Github are:

* a C++ compiler (we use [GCC][] for development)
* [CMake][] cross-platform build system version 2.6 (or higher)
* [Eigen][] matrix library version 2 
* [Google Testing][gtest] Framework _(optional but highly recommended)_

[gcc]: http://gcc.gnu.org/
[cmake]: http://www.cmake.org/
[eigen]: http://eigen.tuxfamily.org/
[gtest]: http://code.google.com/p/googletest/

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
  <td valign="top"><code>EIGEN2_DIR</code></td>
  <td valign="top">Path where the Eigen matrix library is installed. This will end
      up adding several directories to the header search path:
      <code>EIGEN2_DIR</code> itself, <code>EIGEN2_DIR/include</code>,
      <code>EIGEN2_DIR/eigen2</code>, and
      <code>EIGEN2_DIR/include/eigen2</code>.  Note that CMake will fail
      with an error message if it cannot find Eigen2, and <strong>you
      have to clear its cache</strong> before reconfiguring with a
      different <code>EIGEN2_DIR</code> setting. The easiest way to
      achieve that is to remove the entire <code>build</code> directory.</td>
  <td valign="top">If you installed Eigen2 underneath <code>/home/toto/eigen2</code>,
      then you have to pass <code>-DEIGEN2_DIR=/home/toto/eigen2</code>
      to the CMake command.</td>
 </tr>
 <tr>
  <td valign="top"><code>GTEST_DIR</code></td>
  <td valign="top">Path where the Google Testing Framework is installed. This will
      end up adding two directories to the header search path, and two
      directories to the library search path: <code>GTEST_DIR</code>
      itself is added to the header and library path,
      <code>GTEST_DIR/include</code> is added to the header path, and
      <code>GTEST_DIR/lib</code> is added to the library path.  CMake
      will not fail when Gtest is not found (even if you explicitly
      pass <code>GTEST_DIR</code>), although a warning is issued. Again,
      if you add or change <code>GTEST_DIR</code> after a CMake run, you
      should wipe its cache (e.g. the entire <code>build</code>
      directory) before re-running CMake. Note that CMake will fail
      with an error if the Gtest headers are found but the library is
      not.</td>
  <td valign="top">Suppose you have ROS cturtle installed underneath
      <code>/opt/ros/cturtle</code>, then all you have to do is pass
      <code>-DGTEST_DIR=/opt/ros/cturtle/ros/3rdparty/gtest/gtest</code>
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
  recent version, by looking at our [ticketing system][] on the
  project TRAC on Sourceforge
* You can open a new ticket on our project TRAC on Sourceforge (you
  will need a Sourceforge account to do so).
* You can send an email to `stanford-wbc-devel@lists.sourceforge.net`

[ticketing system]: http://sourceforge.net/apps/trac/stanford-wbc/report


Documentation
-------------

If you have [Doxygen][] then you can generate code documentation for
the various sub-projects. There is a little `rundox.sh` script in the
`jspace/doc` and `tao/doc` subdirectories (look for the same pattern
in other parts of the project as well, we might forget to update this
README file when adding sub-projects and documentation).

    cd /path/to/stanford_wbc/jspace/doc
    ./rundox
    then open html/index.html in a web browser

[doxygen]: http://doxygen.org/
