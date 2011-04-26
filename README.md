Stanford Whole-Body Control Framework
=====================================

    Copyright (C) 2008-2011 The Board of Trustees of The Leland Stanford Junior University. All Rights Reserved.

    This program is free software: you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public License
    as published by the Free Software Foundation, either version 3 of
    the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    

The Whole-Body Control framework developed at the [Stanford Robotics
and AI Lab][] provides advanced control for human-centered robotics
and mobile manipulation.  This project provides a framework for
developing robot behaviors that use operational-space hierarchical
task decompositions, based on the work of many contributors over many
years, under the guidance of and in collaboration with [Oussama
Khatib][]: most notably [Jaeheung Park][], K. C. Chang, Diego Ruspini,
[Roy Featherstone][], Bob Holmberg, [François Conti][], [Roland
Philippsen][] and [Luis Sentis][].

[Stanford Robotics and AI Lab]: http://ai.stanford.edu/groups/manips/
[Oussama Khatib]: http://cs.stanford.edu/groups/manips/people/oussama-khatib
[Jaeheung Park]: http://plaza4.snu.ac.kr/~park73/wiki/index.php5/People
[Roy Featherstone]: http://users.cecs.anu.edu.au/~roy/
[François Conti]: http://cs.stanford.edu/groups/manips/people/francois-conti
[Roland Philippsen]: http://cs.stanford.edu/groups/manips/people/roland-philippsen
[Luis Sentis]: http://www.me.utexas.edu/directory/faculty/sentis/luis/


Download
--------

Clone our GIT repository...

    git clone git://github.com/poftwaresatent/stanford_wbc.git

...or grab a tarball from Sourceforge: the [download area][] contains
official releases.

[download area]: http://sourceforge.net/projects/stanford-wbc/files/


Build
-----

The dependencis are:

* a C++ compiler (we use [GCC][] for development)
* [CMake][] cross-platform build system version 2.6 (or higher)
* [Eigen][] matrix library version 2 
* [Fast Light Toolkit][fltk] version 1.1.x _(optional but required for tutorials)_
* [Google Testing][gtest] Framework _(optional but highly recommended)_

[gcc]: http://gcc.gnu.org/
[cmake]: http://www.cmake.org/
[eigen]: http://eigen.tuxfamily.org/
[fltk]: http://www.fltk.org/
[gtest]: http://code.google.com/p/googletest/

After you cloned our repos, or downloaded and unpacked a release, go
into the top-level source directory and build it:

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
 <tr>
  <td valign="top"><code>GTEST_SKIP</code></td>
  <td valign="top">If set to true, this will skip the unit tests even
      if gtest is installed.  Useful mainly as a workaround for
      macports, where the fltk and gtest packages have incompatible
      build flags (which means that on OS X, unless you build fltk
      and/or gtest from source on your own, you can either have the
      tutorials or the unit tests, but not both... very
      annoying).</td>
  <td valign="top">&nbsp;</td>
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

* A budding collection of semi-functioning unit tests for the
  operational-space control library is available by running:

        ./opspace/tests/testTask

If any of these tests fail (well... the opspace tests are a tad
fragile at the time of writing (April 2011)), there is a regression
that should be fixed. Please let us know by sending an email to
`stanford-wbc-users@lists.sourceforge.net`


Documentation
-------------

If you have [Doxygen][] then you can generate code documentation for
the various sub-projects. There is a little `rundox.sh` script in the
`jspace/doc`, `opspace/doc`, and `tao/doc` subdirectories (look for the same pattern
in other parts of the project as well, we might forget to update this
README file when adding sub-projects and documentation).

    cd /path/to/stanford_wbc/jspace/doc
    ./rundox
    then open html/index.html in a web browser

[doxygen]: http://doxygen.org/
