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
[Roland Philippsen]: http://poftwaresatent.net/r/
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

[gcc]: http://gcc.gnu.org/
[cmake]: http://www.cmake.org/
[eigen]: http://eigen.tuxfamily.org/
[fltk]: http://www.fltk.org/

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
</table>


Test
----

We use the [Google Testing][gtest] Framework (included in the
_3rdparty_ subdirectory). At the time of writing, we provide the
following test programs.  Also note that there is a `runtests.sh`
script in the top-level project directory, which you can run from
within the build directory by issuing `../runtests.sh` _(assuming that
your build directory is one level below the top-level source
directory, as implied in this README)_.

[gtest]: http://code.google.com/p/googletest/

* Basic (very basic) tests for the TAO dynamics engine are available
  by running:

        ./tao/testTAO

* A fairly complete suite of unit tests for the joint-space dynamics
  model (which is built on TAO) is available by running:

        ./jspace/tests/testJspace

* A budding collection of semi-functioning unit tests for the
  operational-space control library is available by running:

        ./opspace/tests/testTask

* A quick test of the mechanism that builds controllers from YAML
  files:

        ./opspace/tests/testFactory

If any of these tests fail, there is a regression that should be
fixed. Please let us know via our [Issue Tracker][github_issues] or
sending an email to `stanford-wbc-users@lists.sourceforge.net`

[github_issues]: https://github.com/poftwaresatent/stanford_wbc/issues


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
