# these should come from a configure script or so...
TOP_BUILD_DIR= $(CURDIR)
VPATH= $(CURDIR)/..
TOP_SRC_DIR= $(CURDIR)/..
LIB_DIR= $(TOP_BUILD_DIR)/lib

CXX= g++
CPPFLAGS= -DLINUX -DDISABLE_NETWORKING -DHAVE_NETWRAP
CXXFLAGS= $(CPPFLAGS) -O0 -g -Wall -pipe -I$(TOP_BUILD_DIR)/include -I$(NETWRAP_DIR)/include

LD= $(CXX) -shared
LDFLAGS= -L$(LIB_DIR) \
         -Wl,-rpath,$(LIB_DIR) \
         -L$(NETWRAP_DIR)/lib \
         -Wl,-rpath,$(NETWRAP_DIR)/lib

# variables for building the lib
SRCS= wbcnet/msg/StringList.cpp \
      wbcnet/msg/Matrix.cpp \
      wbcnet/msg/TaskSpec.cpp \
      wbcnet/msg/UserCommand.cpp \
      wbcnet/msg/Status.cpp \
      wbcnet/msg/RobotState.cpp \
      wbcnet/msg/TaskMatrix.cpp \
      wbcnet/msg/ServoCommand.cpp \
      wbcnet/SPQueue.cpp \
      wbcnet/log.cpp \
      wbcnet/Muldex.cpp \
      wbcnet/TaskAtomizer.cpp \
      wbcnet/proxy.cpp \
      wbcnet/pack.cpp \
      wbcnet/DelayHistogram.cpp \
      wbcnet/StreamBufMgr.cpp \
      wbcnet/strutil.cpp \
      wbcnet/data.cpp \
      wbcnet/com.cpp \
      wbcnet/endian.cpp \
      wbcnet/id.cpp \
      wbcnet/NetWrapperWrap.cpp \
      wbcnet/NetConfig.cpp \
      wbcnet/imp/MQNetConfig.cpp \
      wbcnet/imp/SPQNetConfig.cpp \
      wbcnet/imp/NetWrapNetConfig.cpp
OBJS= $(SRCS:.cpp=.o) 

# variables for building programs recursively
EXE= NONE
OBJ= NONE
LIBS= NONE
PROGS= test/testTaskAtomizer.cpp \
       test/testMuldex.cpp \
       test/testEndian.cpp \
       test/testStreamBufMgr.cpp \
       test/testDelayHistogram.cpp \
       test/testLogWithoutLog4cxx.cpp \
       test/testLogDisabled.cpp \
       test/testPack.cpp \
       test/testProxy.cpp \
       test/testID.cpp \
       test/testNetWrapper.cpp

all: $(LIB_DIR)/libwbcnet.so

##all: programs

$(LIB_DIR)/libwbcnet.so: symlink $(OBJS)
	mkdir -p $(LIB_DIR)
	$(LD) -o $(LIB_DIR)/libwbcnet.so $(LDFLAGS) $(OBJS)

.PHONY: symlink
symlink:
	mkdir -p $(TOP_BUILD_DIR)/include
	test -L $(TOP_BUILD_DIR)/include/wbcnet || ln -s $(TOP_SRC_DIR) $(TOP_BUILD_DIR)/include/wbcnet

.PHONY: programs
programs: $(LIB_DIR)/libwbcnet.so
	for spec in $(PROGS); do \
	  prog=`echo $$spec | sed 's/:.*//'` ;\
	  libs=`echo $$spec | sed 's/[^:]*//' | sed 's/:/ /g'`; \
	  $(MAKE) -f $(TOP_SRC_DIR)/Makefile \
	          VPATH="$(VPATH)" \
	          CXX="$(CXX)" \
	          CXXFLAGS="$(CXXFLAGS)" \
	          LDFLAGS="$(LDFLAGS)" \
	          EXE=`basename $$prog .cpp` \
                  OBJ=`echo $$prog | sed s:\.cpp:.o:` \
	          LIBS="-lwbcnet -lnetwrapper $$libs" \
	          program; \
	done

.PHONY: program
program:
	$(MAKE) $(OBJ)
	$(CXX) -o $(EXE) $(OBJ) $(LDFLAGS) $(LIBS)

.SUFFIXES: .o .cpp

.cpp.o:
	mkdir -p `dirname $@`
	$(CXX) $(CXXFLAGS) -o $@ -c $<
