#!/bin/bash

BASE_DIR=`pwd`
WBC_ROOT="~/wbc"
WBC_SOURCE="${BASE_DIR}"

if [ ! -z $1 ]; then
    if [ foo`echo $1 | sed 's:\(.\).*:\1:'` = "foo/" ]; then
        WBC_ROOT=$1
    else
        WBC_ROOT=`pwd`/$1
    fi
fi
echo "Bootstrapping Stanford WBC development setup for ${WBC_ROOT}"
echo "=================================================="

if [ -z "${EXTRA_MAKE_FLAGS}" ]; then
    EXTRA_MAKE_FLAGS=-j4
fi

if [ ! -f ${WBC_SOURCE}/bash-env-wbc ]; then
    echo "${WBC_SOURCE}/bash-env-wbc does not exist, have you checked out the stanford-wbc code?"
    exit 42
fi

if [ ! -d ${WBC_ROOT} ]; then
    mkdir ${WBC_ROOT}
    if [ ! -d ${WBC_ROOT} ]; then
	echo "failed to mkdir ${WBC_ROOT}"
	exit 42
    fi
fi

source ${WBC_SOURCE}/bash-env-wbc ${WBC_ROOT}

cd ${WBC_ROOT}

cmake ${WBC_SOURCE} -DCMAKE_INSTALL_PREFIX=${WBC_ROOT}
if [ $? -ne 0 ]; then
    cd ${BASE_DIR}
    echo "failed to run cmake"
    exit 42
fi

make ${EXTRA_MAKE_FLAGS}
if [ $? -ne 0 ]; then
    cd ${BASE_DIR}
    echo "failed to make ${EXTRA_MAKE_FLAGS}"
    exit 42
fi

SYM_THESE_LIBS="wbc/libStanford_WBC.so tao/libTAO_Dynamics_Engine.so saimatrix/libSAI_Matrix.so wbcnet/libwbcnet.so wbcrun/libwbcrun.so wbc_tinyxml/libwbc_tinyxml.so"

if [ ! -e lib ]; then
    mkdir lib
    if [ $? -ne 0 ]; then
	cd ${BASE_DIR}
	echo "failed to create lib directory (for symlinks to libraries)"
	exit 42
    fi
fi

if [ -d lib ]; then
    for foo in ${SYM_THESE_LIBS}; do
	bar=`basename ${foo}`
	if [ -e lib/${bar} ]; then
	    echo "INFO: There already is a file lib/${bar}"
	    echo "      It should point to ${WBC_ROOT}/${foo}"
	else
	    ln -s ${WBC_ROOT}/${foo} lib/${bar}
	    if [ $? -ne 0 ]; then
		echo "WARNING failed to create symlink lib/${bar} --> ${WBC_ROOT}/${foo}"
	    fi
	fi
    done
else
    cd ${BASE_DIR}
    echo "lib exists and is not a directory, cannot use it for symlinks to libraries"
    exit 42
fi

if [ ! -e bin ]; then
    ln -s ${WBC_ROOT}/applications bin
    if [ $? -ne 0 ]; then
	cd ${BASE_DIR}
	echo "failed to create symlink from bin to appliations"
	exit 42
    fi
else
    echo "INFO: There already is a bin directory"
    echo "      It should contain symnlinks to the various apps..."
fi

if [ ! -e share ]; then
    mkdir share
    if [ $? -ne 0 ]; then
	cd ${BASE_DIR}
	echo "failed to create share directory"
	exit 42
    fi
fi

if [ ! -d share ]; then
    cd ${BASE_DIR}
    echo "share is not a directory"
    exit 42
fi
if [ ! -e share/wbc.cmake ]; then
    ln -s ${WBC_SOURCE}/wbc.cmake share/wbc.cmake
    if [ $? -ne 0 ]; then
	cd ${BASE_DIR}
	echo "failed to create symlink share/wbc.cmake -> ${WBC_SOURCE}/wbc.cmake"
	exit 42
    fi
fi

if [ -e include ]; then
    echo "INFO: There already is an include"
    echo "      It should be a symnlink to the sources, or maybe a directory of symlinks..."
else
    ln -s ${WBC_SOURCE} include
    if [ $? -ne 0 ]; then
	cd ${BASE_DIR}
	echo "failed to create symlink include -> ${WBC_SOURCE}"
	exit 42
    fi
fi

echo "=================================================="
echo "Successfully bootstrapped your Stanford WBC development setup."
echo ""
echo "You can also install to a fixed location.  For instructions read"
echo "https://sourceforge.net/apps/trac/stanford-wbc/wiki/Installation"
echo ""
echo " IMPORTANT! read the following notice"
echo ""
echo "  Run the following command in each shell that you use"
echo "  to work on this project, in order to get the environment"
echo "  set up correctly:"
echo ""
echo "  source ${WBC_SOURCE}/bash-env-wbc ${WBC_ROOT}"
echo ""
echo " IMPORTANT! read the above notice"
