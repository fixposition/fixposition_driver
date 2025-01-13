#!/bin/bash
set -eEu -o pipefail

command=$1
shift

case ${command} in
    onCreateCommand)
        containerWorkspaceFolder=$1
        # ls -la ${containerWorkspaceFolder}
        # id
        case ${ROS_DISTRO} in
            noetic)
                if [ ! -d ${containerWorkspaceFolder}/ros1_ws ]; then
                    ${containerWorkspaceFolder}/create_ros_ws.sh -d ros1_ws
                fi
                ;;
            humble|jazzy)
                if [ ! -d ${containerWorkspaceFolder}/ros2_ws ]; then
                    ${containerWorkspaceFolder}/create_ros_ws.sh -d ros2_ws
                fi
                ;;
            *)
                echo "Don't know how to create a ROS workspace for ROS_DISTRO=${ROS_DISTRO}"
                exit 1
                ;;
        esac
        ;;
    *)
        echo "bad command: ${command}"
        exit 1
        ;;
esac

exit 0
