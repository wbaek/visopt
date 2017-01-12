#!/bin/sh
# https://vision.in.tum.de/research/vslam/lsdslam
CURRENT_DIRECTORY=`cd "\`dirname \"$0\"\`";pwd`
DOWNLOAD_PATH="http://vmcremers8.informatik.tu-muenchen.de/lsd/LSD_room_images.zip"
TARGET_PATH="${CURRENT_DIRECTORY}/../../data"

mkdir -p ${TARGET_PATH}
INSTALL_SCRIPT="curl --silent -L ${DOWNLOAD_PATH} | tar -xz -C ${TARGET_PATH}"

echo ${INSTALL_SCRIPT}
eval ${INSTALL_SCRIPT}

