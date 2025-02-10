#!/bin/bash
set -eEu
set -o pipefail
set -o errtrace
trap 'panic' ERR

SCRIPTDIR=$(dirname $(readlink -f $0))
DEBUG=0

function main
{
    # Get command line options
    local help=0
    local dev=0
    local rosver=0
    local checksdk=1
    OPTERR=1
    while getopts "hdr:vs" opt; do
        case $opt in
            d)
                dev=1
                ;;
            r)
                rosver=${OPTARG}
                ;;
            v)
                DEBUG=1
                ;;
            s)
                checksdk=0
                ;;
            h)
                echo
                echo "Setup Fixposition driver for a ROS workspace"
                echo
                echo "This script sets up the Fixposition driver repo to be used in a existing ROS"
                echo "workspace. It assumes that you cloned the fixposition_driver repo to the src"
                echo "directory of your ROS workspace."
                echo
                echo "[ROS_DISTRO=...] $0 [-d] [-r 1|2]"
                echo
                echo "    -r 1|2  -- ROS versio (default: detect from ROS_DISTRO environment variable)"
                echo "    -s      -- Skip Fixposition SDK check"
                echo
                exit 0
                ;;
            *)
                error "Illegal option '$OPTARG'!" 1>&2
                exit 1
                ;;
        esac
    done
    if [ ${OPTIND} -gt 1 ]; then
        shift $(expr $OPTIND - 1)
    fi

    # Check that the Fixposition SDK is present
    if [ ${checksdk} -gt 0 -a ! -d ${SCRIPTDIR}/fixposition-sdk/fpsdk_common ]; then
        notice "Cloning Fixposition SDK"
        if ! git -C ${SCRIPTDIR} submodule update --init; then
            error "Failed cloning Fixposition SDK. Please fix that manually."
            exit 1
        fi
    fi

    # Get and check for ROS version
    if [ ${rosver} -eq 0 ]; then
        case "${ROS_DISTRO:-}" in
            noetic)
                rosver=1
                ;;
            humble|jazzy)
                rosver=2
                ;;
        esac
    fi
    debug "rosver=${rosver}"
    if ! [ ${rosver} -eq 1 -o ${rosver} -eq 2 ]; then
        exit_fail "Need a (valid) ROS version"
    fi

    local res=0

    # Setup workspace
    notice "Setup fixposition_driver for ROS${rosver}"

    if [ ${rosver} -eq 1 ]; then
        for path in fixposition_driver_ros2 \
            fixposition-sdk/fpsdk_ros2 fixposition-sdk/examples; do
            info "- ${path}/CATKIN_IGNORE";
            touch ${SCRIPTDIR}/${path}/CATKIN_IGNORE
        done
    else
        for path in fixposition_driver_ros1 \
            fixposition-sdk/fpsdk_ros1 fixposition-sdk/examples; do
            info "- ${path}/COLCON_IGNORE";
            touch ${SCRIPTDIR}/${path}/COLCON_IGNORE
        done
        # Workaround for colcon thinking it should try to build the top-level cmake.. :-(
        rm -f ${SCRIPTDIR}/fixposition-sdk/CMakeLists.txt
    fi

    # Happy?
    debug "res=${res}"
    if [ ${res} -eq 0 ]; then
        info "Splendid! You should now be able to:"
        info ""
        if [ ${rosver} -eq 1 ]; then
            info "    catkin build"
        else
            info "    colcon build"
        fi
        info ""
        exit 0
    else
        error "Ouch"
        exit 1
    fi

}

function exit_fail
{
    error "$@"
    echo "Try '$0 -h' for help." 1>&2
    exit 1
}

function notice
{
    echo -e "\033[1;37m$@\033[m" 1>&2
}

function info
{
    echo -e "\033[0m$@\033[m" 1>&2
}

function warning
{
    echo -e "\033[1;33mWarning: $@\033[m" 1>&2
}

function error
{
    echo -e "\033[1;31mError: $@\033[m" 1>&2
}

function debug
{
    if [ ${DEBUG} -gt 0 ]; then
        echo -e "\033[0;36mDebug: $@\033[m" 1>&2
    fi
}

function panic
{
    local res=$?
    echo -e "\033[1;35mPanic at ${BASH_SOURCE[1]}:${BASH_LINENO[0]}! ${BASH_COMMAND} (res=$res)\033[m" 1>&2
    exit $res
}

main "$@"
exit 99
