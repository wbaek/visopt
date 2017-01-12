#!/bin/sh
DEVELOPER_NAME="google"
PROJECT_NAME="googletest"
TAG_NAME="release-1.8.0"
BUILD_SCRIPT="mkdir build; cd build; cmake -DCMAKE_INSTALL_PREFIX=\${TARGET_PATH}/release ..; make install"

CURRENT_DIRECTORY=`cd "\`dirname \"$0\"\`";pwd`
source ${CURRENT_DIRECTORY}/install_github.sh
