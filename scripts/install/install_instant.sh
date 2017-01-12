#!/bin/sh
DEVELOPER_NAME="wbaek"
PROJECT_NAME="instant"
TAG_NAME="v0.1.1"
BUILD_SCRIPT="make install"

CURRENT_DIRECTORY=`cd "\`dirname \"$0\"\`";pwd`
source ${CURRENT_DIRECTORY}/install_github.sh
