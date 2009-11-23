AC_INIT([wbcnet], [0.0pre], [stanford-wbc-devel AT lists DOT sourceforge DOT net])
AC_CONFIG_AUX_DIR([.])
AC_CONFIG_MACRO_DIR([m4])
AC_PREREQ(2.57)
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

AC_LANG(C++)
AC_PROG_CXX
AC_PROG_LIBTOOL
AC_PROG_INSTALL
AC_PROG_AWK
AC_PROG_SED

if test "x${GXX}" != "xyes"; then
   AC_MSG_WARN([Looks like a non-GNU C++ compiler, build might break.])
   AC_MSG_WARN([Update configure.ac with compiler-vendor detection.])
fi

CPPFLAGS=""
CXXFLAGS="-pipe -Wall"
LDFLAGS=""

AC_CANONICAL_HOST
case $host_os in
  linux*)
    AC_MSG_NOTICE([detected Linux])
    CPPFLAGS="$CPPFLAGS -DLINUX"
    AC_DISABLE_STATIC;;
  *)
    AC_MSG_WARN([Unknown host_os $host_os, build will probably break.])
    AC_MSG_WARN([Please update configure.ac.]);;
esac

AC_ARG_ENABLE(debug,
  AS_HELP_STRING([--enable-debug], [GCC options -g -O0 (else -O3)]),
  [ CXXFLAGS="$CXXFLAGS -g -O0" ],
  [ CXXFLAGS="$CXXFLAGS -O3" ])

AC_CHECK_HEADER([mqueue.h],
  [ AM_CONDITIONAL([HAVE_MQUEUE], [true]) ],
  [ AC_MSG_NOTICE([POSIX message queue support disabled])
    AM_CONDITIONAL([HAVE_MQUEUE], [false]) ])

AC_ARG_WITH(log4cxx,
  AC_HELP_STRING([--with-log4cxx=PATH], [Apache log4cxx install dir]),
  [ if test "x$withval" != "x" ; then
      CPPFLAGS="$CPPFLAGS -I${withval}/include -I${withval}"
      LDFLAGS="-L${withval}/lib -L${withval} $LDFLAGS"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
if test "$prefix" != "NONE" ; then
  CPPFLAGS="$CPPFLAGS -I$prefix/include"
fi
AC_CHECK_HEADER([log4cxx/logger.h],
  [ CPPFLAGS="$CPPFLAGS -DHAVE_LOG4CXX"
    AM_CONDITIONAL([HAVE_LOG4CXX], [true]) ],
  [ AC_MSG_NOTICE([log4cxx support disabled])
    AM_CONDITIONAL([HAVE_LOG4CXX], [false]) ] )
CPPFLAGS="$oldCPPFLAGS"

AC_ARG_WITH(netwrapper,
  AC_HELP_STRING([--with-netwrapper=PATH], [Force Dimension NetWrapper install dir]),
  [ if test "x$withval" != "x" ; then
      CPPFLAGS="$CPPFLAGS -I${withval}/include"
      LDFLAGS="-L${withval}/lib $LDFLAGS"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
if test "$prefix" != "NONE" ; then
  CPPFLAGS="$CPPFLAGS -I$prefix/include"
fi
AC_CHECK_HEADER([NetWrapper.h],
  [ CPPFLAGS="$CPPFLAGS -DHAVE_NETWRAP"
    AM_CONDITIONAL([HAVE_NETWRAP], [true]) ],
  [ AC_MSG_NOTICE([netwrapper support disabled])
    AM_CONDITIONAL([HAVE_NETWRAP], [false]) ] )
CPPFLAGS="$oldCPPFLAGS"

if test "$prefix" != "NONE" ; then
  CPPFLAGS="$CPPFLAGS -I$prefix/include"
  LDFLAGS="$LDFLAGS -I$prefix/lib"
fi

AC_SUBST(PACKAGE_VERSION)
AC_SUBST(CPPFLAGS)
AC_SUBST(CXXFLAGS)
AC_SUBST(LDFLAGS)

AC_CONFIG_HEADERS([wbcnet_config.h])
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
