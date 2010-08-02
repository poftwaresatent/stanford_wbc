#!/bin/sh

##################################################
# HOW TO CREATE COVERAGE REPORTS WITH THIS SCRIPT
# 
#  cd build
#  rm -rf * && cmake -DCOVERAGE=true .. && make -j 4
#  cd ..
#  lcov --directory build --zerocounters
#  cd build
#  ../runtests.sh 
#  cd ..
#  lcov --directory build --capture --output-file wbc.info
#  genhtml wbc.info -o stanford-wbc-lcov/

## to do: make these standalone?
# wbcnet/testStreamBufMgr
# wbcnet/testMQSpeed
# wbcnet/testSockWrapMuldex
# wbc/tests/testDirectoryServer
# wbc/tests/testBehaviorParser

## to do: needs to find the test module's .so
# wbcnet/testTestModule

## segfaults on purpose
# wbc/tests/testDtorCheck

## segfaults
# wbc_plugins/wbc_plugins/robotlog/test

## lcov did not like these (truncated output files?)
#    wbcnet/testLogWithoutLog4cxx \
#    wbcnet/testDelayHistogram \
#    wbcnet/testLogDisabled \
#    wbcnet/testLogWithLog4cxx \
#    saimatrix/test_SAILapack \

MSG=""
FAIL=""
NOTFOUND=""

for test in \
    jspace/tests/testServoProxy \
    wbcnet/testPack \
    wbcnet/testTaskAtomizer \
    wbcnet/testMQWrap \
    wbcnet/testMuldex \
    wbcnet/testEndian \
    wbcnet/testID \
    wbcnet/testProxy \
    wbcnet/testFactory \
    jspace/tests/testJspace \
    tao/testTAO \
    wbc/tests/testProcess \
    wbc_tinyxml/xmltest; do
    if [ -x $test ]; then
	$test 2>&1
	if [ $? -eq 0 ]; then
	    MSG="$MSG\n$test OK"
	else
	    MSG="$MSG\n$test failed"
	    FAIL="$FAIL $test"
	fi
    else
	MSG="$MSG\n$test not found"
	NOTFOUND="$NOTFOUND $test"
    fi
done

echo -e $MSG
if [ -n "$NOTFOUND" ]; then
    echo -e "\nNot found:$NOTFOUND"
else
    echo -e "\nAll tests were found."
fi
if [ -n "$FAIL" ]; then
    echo -e "\nFailures in:$FAIL"
    exit 1
else
    if [ -n "$MSG" ]; then
	echo -e "\nAll tests passed."
    else
	echo -e "\nNo tests were run."
    fi
fi
