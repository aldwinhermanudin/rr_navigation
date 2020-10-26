#!/bin/bash
# Print usage
usage() {
  echo -n "$0 [OPTION]... [FILE]...

Helper to login to ROS Kinetic for Rover Robotic

 Options:
  -u, --ros-master-uri          Set ROS_MASTER_URI for Host ROS
  -m, --mount-dir               Directory to mount to the container
  -i, --docker-image            Docker image to run
  -s, --ros-package-source      Path ROS Package Source
  -e, --hosts-entry             Path additional hosts file
  -d, --debug                   Enable debug mode
  -h, --help                    Display this help and exit

 Example:
  $0 -u "http://172.17.0.1:11311" -m "/home/aldwin" -i "rr_kinetic:v1" -s "/home/aldwin/Documents/Projects/SLAMRoverRobotics/src" -e "data/cfg/hosts" -d
"
}

HELP=false
GUEST_ROS_MASTER_URI=""
MOUNT_DIR=""
DOCKER_IMAGE=""
DEBUG=false
ROS_PACKAGE_SRC=""
HOSTS_ENTRY=""

PARAMS=""
while (( "$#" )); do
  case "$1" in
    -h|--help)
      HELP=true
      shift
      ;;
    -u|--ros-master-uri)
      if [ -n "$2" ] && [ ${2:0:1} != "-" ]; then
        GUEST_ROS_MASTER_URI=$2
        shift 2
      else
        echo "Error: Argument for $1 is missing" >&2
        exit 1
      fi
      ;;
    -m|--mount-dir)
      if [ -n "$2" ] && [ ${2:0:1} != "-" ]; then
        MOUNT_DIR=$2
        shift 2
      else
        echo "Error: Argument for $1 is missing" >&2
        exit 1
      fi
      ;;
    -i|--docker-image)
      if [ -n "$2" ] && [ ${2:0:1} != "-" ]; then
        DOCKER_IMAGE=$2
        shift 2
      else
        echo "Error: Argument for $1 is missing" >&2
        exit 1
      fi
      ;;
    -s|--ros-package-source)
      if [ -n "$2" ] && [ ${2:0:1} != "-" ]; then
        ROS_PACKAGE_SRC=$2
        shift 2
      else
        echo "Error: Argument for $1 is missing" >&2
        exit 1
      fi
      ;;
    -e|--hosts-entry)
      if [ -n "$2" ] && [ ${2:0:1} != "-" ]; then
        HOSTS_ENTRY=$2
        shift 2
      else
        echo "Error: Argument for $1 is missing" >&2
        exit 1
      fi
      ;;
    -d|--debug)
      DEBUG=true
      shift
      ;;
    -*|--*=) # unsupported flags
      echo "Error: Unsupported flag $1" >&2
      exit 1
      ;;
    *) # preserve positional arguments
      PARAMS="$PARAMS $1"
      shift
      ;;
  esac
done
# set positional arguments in their proper place
eval set -- "$PARAMS"

if [[ ${HELP} == true ]]; then
    usage
    exit 0
fi

if [[ ${GUEST_ROS_MASTER_URI} == "" || ${MOUNT_DIR} == "" || ${DOCKER_IMAGE} == "" || ${HOSTS_ENTRY} == "" ]]; then
    echo "Please define GUEST_ROS_MASTER_URI, MOUNT_DIR, DOCKER_IMAGE, HOSTS_ENTRY"
    usage
    exit 1
fi

if [ ! -f "${HOSTS_ENTRY}" ]; then
    echo "${HOSTS_ENTRY}" "not exists."
    usage
    exit 1
fi


if [[ ${DEBUG} == true ]]; then 
    echo "HELP                  = $HELP"
    echo "GUEST_ROS_MASTER_URI  = $GUEST_ROS_MASTER_URI"
    echo "MOUNT_DIR             = $MOUNT_DIR"
    echo "DOCKER_IMAGE          = $DOCKER_IMAGE"
    echo "ROS_PACKAGE_SRC       = $ROS_PACKAGE_SRC"
    echo "HOSTS_ENTRY           = $HOSTS_ENTRY"
    echo "DEBUG                 = $DEBUG"
fi

if [[ ${ROS_PACKAGE_SRC} != "" ]]; then
    docker run -v ${ROS_PACKAGE_SRC}:/root/catkin_ws/src \
               -v ${MOUNT_DIR}:/mnt \
               -v ${HOSTS_ENTRY}:/data/hosts \
               -it ${DOCKER_IMAGE} "${GUEST_ROS_MASTER_URI}"
else
    docker run -v ${MOUNT_DIR}:/mnt \
               -v ${HOSTS_ENTRY}:/data/hosts \
               -it ${DOCKER_IMAGE} "${GUEST_ROS_MASTER_URI}"
fi