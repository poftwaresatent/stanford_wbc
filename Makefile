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

### If you have your own implementation of non-dynamic plugins,
### override this variable.
WBC_ADD_BUILTIN_PLUGINS_SRC= wbc/bin/example_wbc_add_builtin_plugins.cpp

### You can add further library sources by overriding EXTRA_SRCS
EXTRA_SRCS= 

### You can add further preprocessor flags by overriding EXTRA_CPPFLAGS
EXTRA_CPPFLAGS= 

### with GNU toolchain:
# The -DDISABLE_PLUGINS ends up telling the plugin mechanism to not
# actually look for dynamically loadable modules, but to call
# wbc_add_builtin_plugins() instead. See wbc/bin/builtin.cpp and
# wbc/bin/example_wbc_add_builtin_plugins.cpp (which is used in this
# minimal build setup).
CXX= g++
CPPFLAGS= -DLINUX -DWBCNET_HAVE_MQUEUE -DDISABLE_PLUGINS -I$(TOP_SRC_DIR) -I$(TOP_SRC_DIR)/wbcnet $(EXTRA_CPPFLAGS)
CXXFLAGS= $(CPPFLAGS) -O0 -g -Wall -pipe

### this seems to work rather generally:
AR= ar
ARFLAGS= r


all: programs


### Sources that implement actual networking.
# Only use these if you do NOT pass -DDISABLE_NETWORKING to the
# compilation.
NET_SRCS= wbcnet/wbcnet/misc/AutoSocket.cpp \
          wbcnet/wbcnet/misc/udp_util.cpp \
          wbcnet/wbcnet/imp/MQWrap.cpp \
          wbcnet/wbcnet/imp/SockWrap.cpp \
          wbcnet/wbcnet/net.cpp

### List of all the source files we build.
# Depending on whether we want networking, the NET_SRCS should be
# included or not. All of these source files will simply be compiled
# and archived into one big (currently static) lib.
#
# In case you want plugin support, remove the -DDISABLE_PLUGINS from
# the CPPFLAGS and add the DLModule.cpp file to the SRCS list. But
# then you'll also have to correctly recurse into the plugin
# directories etc... You'll also need to add -ldl to the executables
# (see below)
#      wbcnet/wbcnet/DLModule.cpp \
#
# The Lotus architect is problematic because it expects a tinyxml that
# is different from the rest of WBC:
#      wbc/robarch/lotusarchitect/CLotusArchitect.cpp \
#      wbc/robarch/lotusarchitect/tixml_parser/CLotusTiXmlParser.cpp \
#
SRCS= wbcnet/wbcnet/msg/TaskMatrix.cpp \
      wbcnet/wbcnet/msg/Matrix.cpp \
      wbcnet/wbcnet/msg/Status.cpp \
      wbcnet/wbcnet/msg/RobotState.cpp \
      wbcnet/wbcnet/msg/ServoCommand.cpp \
      wbcnet/wbcnet/msg/Service.cpp \
      wbcnet/wbcnet/msg/StringList.cpp \
      wbcnet/wbcnet/msg/UserCommand.cpp \
      wbcnet/wbcnet/msg/TaskSpec.cpp \
      wbcnet/wbcnet/strutil.cpp \
      wbcnet/wbcnet/NetConfig.cpp \
      wbcnet/wbcnet/imp/MQNetConfig.cpp \
      wbcnet/wbcnet/imp/NetWrapperWrap.cpp \
      wbcnet/wbcnet/imp/SPQueue.cpp \
      wbcnet/wbcnet/imp/TCPNetConfig.cpp \
      wbcnet/wbcnet/imp/NetWrapNetConfig.cpp \
      wbcnet/wbcnet/imp/SPQNetConfig.cpp \
      wbcnet/wbcnet/log.cpp \
      wbcnet/wbcnet/endian.cpp \
      wbcnet/wbcnet/com.cpp \
      wbcnet/wbcnet/pack.cpp \
      wbcnet/wbcnet/proxy.cpp \
      wbcnet/wbcnet/misc/id.cpp \
      wbcnet/wbcnet/misc/message_id.cpp \
      wbcnet/wbcnet/misc/DelayHistogram.cpp \
      wbcnet/wbcnet/misc/TaskAtomizer.cpp \
      wbcnet/wbcnet/misc/StreamBufMgr.cpp \
      wbcnet/wbcnet/data.cpp \
      wbcnet/wbcnet/Muldex.cpp \
      tao/dynamics/taoCNode.cpp \
      tao/dynamics/taoABDynamics.cpp \
      tao/dynamics/taoABJoint.cpp \
      tao/dynamics/taoWorld.cpp \
      tao/dynamics/taoGroup.cpp \
      tao/dynamics/taoDynamics.cpp \
      tao/dynamics/taoJoint.cpp \
      tao/dynamics/taoNode.cpp \
      tao/dynamics/taoABNode.cpp \
      tao/utility/TaoDeMassProp.cpp \
      tao/utility/TaoDeLogger.cpp \
      tao/matrix/TaoDeQuaternionf.cpp \
      tao/matrix/TaoDeMatrix6.cpp \
      tao/matrix/TaoDeVector6.cpp \
      tao/matrix/TaoDeMatrix3f.cpp \
      tao/matrix/TaoDeTransform.cpp \
      wbc_tinyxml/wbc_tinyxmlerror.cpp \
      wbc_tinyxml/wbc_tinyxmlparser.cpp \
      wbc_tinyxml/wbc_tinyxml.cpp \
      wbc_tinyxml/wbc_tinystr.cpp \
      wbc/parse/taoRepCreator/CTaoRepCreator.cpp \
      wbc/parse/TiXmlBRParser.cpp \
      wbc/parse/BehaviorParser.cpp \
      wbc/parse/OsimBRParser.cpp \
      wbc/parse/BRBuilder.cpp \
      wbc/parse/BRParser.cpp \
      wbc/motion/JointTask.cpp \
      wbc/motion/OrientationTask.cpp \
      wbc/motion/WholeBodyPosture.cpp \
      wbc/motion/PositionTask.cpp \
      wbc/motion/PostureBehavior.cpp \
      wbc/motion/FrictionPosture.cpp \
      wbc/motion/JointLimitConstraint.cpp \
      wbc/motion/COMTask.cpp \
      wbc/motion/FloatBehavior.cpp \
      wbc/motion/ObstacleAvoidanceTask.cpp \
      wbc/bin/TaskModelListener.cpp \
      wbc/bin/ServoModelProcess.cpp \
      wbc/bin/ModelProcess.cpp \
      wbc/bin/UserProcess.cpp \
      wbc/bin/options.cpp \
      wbc/bin/attributes.cpp \
      wbc/bin/Process.cpp \
      wbc/bin/DirectoryCmdServer.cpp \
      wbc/bin/ServoProcess.cpp \
      wbc/bin/directory.cpp \
      wbc/bin/builtin.cpp \
      wbc/util/DataLog.cpp \
      wbc/util/File.cpp \
      wbc/util/saiTime.cpp \
      wbc/util/dump.cpp \
      wbc/util/utc.cpp \
      wbc/util/dtor_check.cpp \
      wbc/util/ShuffleVector.cpp \
      wbc/util/tao_util.cpp \
      wbc/util/RecorderImpl.cpp \
      wbc/core/Plugin.cpp \
      wbc/core/MobileManipulatorTaskModel.cpp \
      wbc/core/TaskSet.cpp \
      wbc/core/TaskModelBase.cpp \
      wbc/core/TaskDescription.cpp \
      wbc/core/BranchingRepresentation.cpp \
      wbc/core/SAIMatrixAPI.cpp \
      wbc/core/SAIVectorAPI.cpp \
      wbc/core/Contact.cpp \
      wbc/core/MobileManipulatorServoBehaviors.cpp \
      wbc/core/RobotControlModel.cpp \
      wbc/core/Kinematics.cpp \
      wbc/core/Dynamics.cpp \
      wbc/core/BehaviorFactory.cpp \
      wbc/core/BehaviorDescription.cpp \
      wbc/core/RobotFactory.cpp \
      plugins/fake/RobotFake.cpp \
      plugins/fake/RawController.cpp \
      plugins/fake/DebugBehavior.cpp \
      plugins/robotlog/readlog.cpp \
      plugins/robotlog/writelog.cpp \
      wbc/robarch/CRobotArchitect.cpp \
      wbc/robarch/osimarchitect/COsimArchitect.cpp \
      wbc/robarch/osimarchitect/parser/SkeletonCoordinate.cpp \
      wbc/robarch/osimarchitect/parser/CTransformAxisObj.cpp \
      wbc/robarch/osimarchitect/parser/SkeletonMarker.cpp \
      wbc/robarch/osimarchitect/parser/CWrapCylinderNew.cpp \
      wbc/robarch/osimarchitect/parser/CJointNew.cpp \
      wbc/robarch/osimarchitect/parser/CMuscleDefinition.cpp \
      wbc/robarch/osimarchitect/parser/CSkeletonLinkNew.cpp \
      wbc/robarch/osimarchitect/parser/CWrapEllipsoidNew.cpp \
      wbc/robarch/osimarchitect/parser/CCoordObj.cpp \
      wbc/robarch/osimarchitect/parser/CSkeletonModelNew.cpp \
      saimatrix/SAIMatrix.cpp \
      saimatrix/SAIVector6.cpp \
      saimatrix/SAIQuaternion.cpp \
      saimatrix/SAILapack.cpp \
      saimatrix/SAIMatrix3.cpp \
      saimatrix/SAIMatrix6.cpp \
      saimatrix/SAITransform.cpp \
      saimatrix/SAIVector.cpp \
      saimatrix/SAIVector3.cpp \
      jspace/State.cpp \
      jspace/Model.cpp \
      $(WBC_ADD_BUILTIN_PLUGINS_SRC) \
      $(NET_SRCS) \
      $(EXTRA_SRCS) \

OBJS= $(SRCS:.cpp=.o) 


### All network-related programs.
# Add these to PROGS if you enable networking (i.e. you do NOT define -DDISABLE_NETWORKING)

NET_PROGS= applications/wbcservo.cpp:-lrt:-lexpat:-llapack:-lblas \
           applications/wbcmodel.cpp:-lrt:-lexpat:-llapack:-lblas \
           applications/wbcuser.cpp:-lrt


### Variables for building all programs (test and such).
#
# If you need to link with external libs, just specify the
# corresponding linker flags appended to the source filename,
# replacing spaces with colons. The programs target parses them and
# passes it into the recursive make via the LIBS variable.

EXE= NONE
OBJ= NONE
LIBS= NONE
PROGS= applications/rawservo.cpp:-llapack:-lblas:-lexpat:-lrt \
       applications/robot-bridge.cpp:-llapack:-lblas \
       $(NET_PROGS)


.PHONY: programs
programs: libStanford_WBC.a
	for spec in $(PROGS); do \
	  prog=`echo $$spec | sed 's/:.*//'` ;\
	  libs=`echo $$spec | sed 's/[^:]*//' | sed 's/:/ /g'`; \
	  mkdir -p `dirname $$prog`; \
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
