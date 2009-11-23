### Minimalist GNU Makefile for the Stanford Whole-Body Control Framework
#
# RTFM! (Yes, that means you: read on!)
#
# This Makefile builds a subset of the functionality. It does not
# detect some of the optional features. It does not take care of
# plugin location and other goodies.
#
# This Makefile is written such that it works out-of-the box only in a
# build directory that is a subdirectory of the top source
# directory. Why? Because it allows us to easily blow away a stale
# build. So, use the following sequence of command to set up and run
# the build:
#   $ mkdir build
#   $ cd build
#   $ make -f ../Makefile
#

### Variables pointing make to the correct places.
# You can also set these in the environment or on the make command
# line, in case you're not following the recommended setup described
# above.
TOP_BUILD_DIR= $(CURDIR)
VPATH= $(CURDIR)/..
TOP_SRC_DIR= $(VPATH)

### with GNU toolchain:
# Note that the WBCNET_HAVE_MQUEUE and WBCRUN_HAVE_MQUEUE are useless for the "minimal Windows DLL" setup.
CXX= g++
CPPFLAGS= -DLINUX -DWBCNET_HAVE_MQUEUE -DWBCRUN_HAVE_MQUEUE -DDISABLE_PLUGINS -DDISABLE_NETWORKING
CXXFLAGS= $(CPPFLAGS) -O0 -g -Wall -pipe -I$(TOP_SRC_DIR)

### this seems to work rather generally:
AR= ar
ARFLAGS= r


all: programs


### Sources that implement actual networking.
# Only use these if you do NOT pass -DDISABLE_NETWORKING to the
# compilation.
NET_SRCS= wbcnet/AutoSocket.cpp \
          wbcnet/udp_util.cpp \
          wbcnet/MQWrap.cpp \
          wbcnet/SockWrap.cpp \
          wbcnet/net.cpp

### List of all the source files we build.
# Depending on whether we want networking, the NET_SRCS should be
# included or not. All of these source files will simply be compiled
# and archived into one big (currently static) lib.
SRCS= wbc/robarch/osimarchitect/parser/CTransformAxisObj.cpp \
      wbc/robarch/osimarchitect/parser/CJointNew.cpp \
      wbc/robarch/osimarchitect/parser/CCoordObj.cpp \
      wbc/robarch/osimarchitect/parser/CMuscleDefinition.cpp \
      wbc/robarch/osimarchitect/parser/CSkeletonLinkNew.cpp \
      wbc/robarch/osimarchitect/parser/CWrapCylinderNew.cpp \
      wbc/robarch/osimarchitect/parser/CSkeletonModelNew.cpp \
      wbc/robarch/osimarchitect/parser/SkeletonMarker.cpp \
      wbc/robarch/osimarchitect/parser/CWrapEllipsoidNew.cpp \
      wbc/robarch/osimarchitect/parser/SkeletonCoordinate.cpp \
      wbc/robarch/osimarchitect/COsimArchitect.cpp \
      wbc/robarch/CRobotArchitect.cpp \
      plugins/fake/plugin.cpp \
      plugins/fake/DebugBehavior.cpp \
      plugins/fake/RobotFake.cpp \
      plugins/robotlog/plugin.cpp \
      plugins/robotlog/writelog.cpp \
      plugins/robotlog/readlog.cpp \
      wbcnet/msg/StringList.cpp \
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
      tao/utility/TaoDeMassProp.cpp \
      tao/utility/TaoDeLogger.cpp \
      tao/matrix/TaoDeVector6.cpp \
      tao/matrix/TaoDeMatrix6.cpp \
      tao/matrix/TaoDeQuaternionf.cpp \
      tao/matrix/TaoDeMatrix3f.cpp \
      tao/matrix/TaoDeTransform.cpp \
      tao/dynamics/taoDynamics.cpp \
      tao/dynamics/taoGroup.cpp \
      tao/dynamics/taoABDynamics.cpp \
      tao/dynamics/taoJoint.cpp \
      tao/dynamics/taoNode.cpp \
      tao/dynamics/taoABJoint.cpp \
      tao/dynamics/taoWorld.cpp \
      tao/dynamics/taoCNode.cpp \
      tao/dynamics/taoABNode.cpp \
      wbc/core/TaskDescription.cpp \
      wbc/core/BranchingRepresentation.cpp \
      wbc/core/BehaviorDescription.cpp \
      wbc/core/TaskModelBase.cpp \
      wbc/core/TaskSet.cpp \
      wbc/core/Kinematics.cpp \
      wbc/core/Plugin.cpp \
      wbc/core/MobileManipulatorTaskModel.cpp \
      wbc/core/Dynamics.cpp \
      wbc/core/MobileManipulatorServoBehaviors.cpp \
      wbc/core/RobotControlModel.cpp \
      wbc/core/RobotFactory.cpp \
      wbc/core/BehaviorFactory.cpp \
      wbc/core/SAIVectorAPI.cpp \
      wbc/core/SAIMatrixAPI.cpp \
      wbc/core/Contact.cpp \
      wbcrun/service.cpp \
      wbcrun/message_id.cpp \
      wbcrun/directory.cpp \
      wbcrun/NetConfig.cpp \
      wbcrun/UserProcess.cpp \
      wbcrun/util.cpp \
      wbcrun/Process.cpp \
      saimatrix/SAIVector.cpp \
      saimatrix/SAIMatrix.cpp \
      saimatrix/SAITransform.cpp \
      saimatrix/SAIQuaternion.cpp \
      saimatrix/SAIVector3.cpp \
      saimatrix/SAIMatrix3.cpp \
      saimatrix/SAIVector6.cpp \
      saimatrix/SAIMatrix6.cpp \
      saimatrix/SAILapack.cpp \
      wbc/motion/FrictionPosture.cpp \
      wbc/motion/COMTask.cpp \
      wbc/motion/JointTask.cpp \
      wbc/motion/WholeBodyPosture.cpp \
      wbc/motion/OrientationTask.cpp \
      wbc/motion/PostureBehavior.cpp \
      wbc/motion/FloatBehavior.cpp \
      wbc/motion/JointLimitConstraint.cpp \
      wbc/motion/PositionTask.cpp \
      wbc/motion/ObstacleAvoidanceTask.cpp \
      wbc/bin/ModelProcess.cpp \
      wbc/bin/builtin.cpp \
      wbc/bin/ServoModelProcess.cpp \
      wbc/bin/BehaviorDirectory.cpp \
      wbc/bin/attributes.cpp \
      wbc/bin/TaskModelListener.cpp \
      wbc/bin/options.cpp \
      wbc/bin/ServoProcess.cpp \
      wbc/bin/example_wbc_add_builtin_plugins.cpp \
      wbc_tinyxml/wbc_tinyxmlparser.cpp \
      wbc_tinyxml/wbc_tinystr.cpp \
      wbc_tinyxml/wbc_tinyxmlerror.cpp \
      wbc_tinyxml/wbc_tinyxml.cpp \
      wbc/parse/taoRepCreator/CTaoRepCreator.cpp \
      wbc/parse/TiXmlBRParser.cpp \
      wbc/parse/BRParser.cpp \
      wbc/parse/OsimBRParser.cpp \
      wbc/parse/BRBuilder.cpp \
      wbc/parse/BehaviorParser.cpp \
      wbc/util/ShuffleVector.cpp \
      wbc/util/tao_util.cpp \
      wbc/util/RecorderImpl.cpp \
      wbc/util/DataLog.cpp \
      wbc/util/saiTime.cpp

OBJS= $(SRCS:.cpp=.o) 


### All network-related programs.
# Add these to PROGS if you enable networking (i.e. you do NOT define -DDISABLE_NETWORKING)

NET_PROGS= wbcnet/testMQWrap.cpp:-lrt \
           wbcnet/testSockWrapMuldex.cpp \
           wbc/bin/servo.cpp:-lrt:-lexpat:-llapack:-lblas \
           wbc/bin/user.cpp:-lrt \
           wbc/bin/model.cpp:-lrt:-lexpat:-llapack:-lblas


### Variables for building all programs (test and such).
#
# If you need to link with external libs, just specify the
# corresponding linker flags appended to the source filename,
# replacing spaces with colons. The programs target parses them and
# passes it into the recursive make via the LIBS variable.

EXE= NONE
OBJ= NONE
LIBS= NONE
PROGS= wbcnet/testTaskAtomizer.cpp \
       wbcnet/testMuldex.cpp \
       wbcnet/testEndian.cpp \
       wbcnet/testStreamBufMgr.cpp \
       wbcnet/testDelayHistogram.cpp \
       wbcnet/testLogWithoutLog4cxx.cpp \
       wbcnet/testLogDisabled.cpp \
       wbcnet/testPack.cpp \
       wbcnet/testProxy.cpp \
       wbcnet/testID.cpp \
       saimatrix/test_lapack_svd.cpp:-llapack:-lblas \
       saimatrix/test_SAILapack.cpp:-llapack:-lblas


.PHONY: programs
programs: libStanford_WBC.a
	for spec in $(PROGS); do \
	  prog=`echo $$spec | sed 's/:.*//'` ;\
	  libs=`echo $$spec | sed 's/[^:]*//' | sed 's/:/ /g'`; \
	  $(MAKE) -f $(TOP_SRC_DIR)/Makefile \
	          VPATH="$(VPATH)" \
	          CXX="$(CXX)" \
	          CXXFLAGS="$(CXXFLAGS)" \
	          EXE=`basename $$prog .cpp` \
                  OBJ=`echo $$prog | sed s:\.cpp:.o:` \
	          LIBS="$$libs" \
	          program; \
	done

.PHONY: program
program:
	$(MAKE) $(OBJ)
	$(CXX) -o $(EXE) $(OBJ) -L. -lStanford_WBC $(LIBS)

libStanford_WBC.a: $(OBJS)
	- rm libStanford_WBC.a
	$(AR) $(ARFLAGS) libStanford_WBC.a $(OBJS)

.PHONY: clean
clean:
	- rm $(OBJS) libStanford_WBC.a
	for prog in $(PROGS); do \
	  rm $$prog $$prog.o; done

.SUFFIXES: .o .cpp

.cpp.o:
	mkdir -p `dirname $@`
	$(CXX) $(CXXFLAGS) -o $@ -c $<


sinclude deps.mk
