#!/bin/bash

UNIT_TESTS="testEndian testID testMQWrap testPack testProxy testTaskAtomizer testMuldex"
USER_TESTS="testSockWrapMuldex testStreamBufMgr"

rm -f run-success.log
rm -f run-failure.log
for test in $UNIT_TESTS; do
    rm -f run.$test.log
    echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    echo "running $test"
    if ./$test; then
	echo "  $test succeeded" >> run-success.log
    else
	echo "  $test FAILED, see run.$test.log for details" >> run-failure.log
    fi 2>&1 | tee run.$test.log
done

echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
echo "finished testing"

if [ -f run-success.log ]; then
    echo "successes:"
    cat run-success.log
fi

if [ ! -z "$USER_TESTS" ]; then
    echo "skipped:"
    for test in $USER_TESTS; do
	echo "  $test requires user interaction"
    done
fi

if [ -f run-failure.log ]; then
    echo "FAILURES:"
    cat run-failure.log
    exit 1
fi
echo "CONGRATULATIONS, no failures"
exit 0
