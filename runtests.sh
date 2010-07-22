#!/bin/sh

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

for test in \
    jspace/testServoProxy \
    wbcnet/testPack \
    wbcnet/testTaskAtomizer \
    wbcnet/testMQWrap \
    wbcnet/testMuldex \
    wbcnet/testEndian \
    wbcnet/testID \
    wbcnet/testProxy \
    wbcnet/testFactory \
    jspace-tests/testJspace \
    tao/testTAO \
    wbc/tests/testProcess \
    wbc_tinyxml/xmltest; do
    if [ -x $test ]; then
	$test 2>&1 | tee $test.log
	if [ $? -eq 0 ]; then
	    MSG="$MSG\n$test OK"
	else
	    MSG="$MSG\n$test failed, see $foo.log for details"
	    FAIL="$FAIL $test"
	fi
    else
	MSG="$MSG\n$test not found"
    fi
done

echo $MSG
if [ -n "$FAIL" ]; then
    echo "There were failures in:$FAIL"
    exit 1
else
    echo "All tests passed."
fi
