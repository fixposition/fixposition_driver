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
                echo "Create a ROS workspace for the Fixposition ROS driver"
                echo
                echo "Note that this is meant to be used in Fixposition driver development."
                echo "To use the driver in your existing ROS workspace, use the setup_ros_ws.sh script."
                echo
                echo "[ROS_DISTRO=...] $0 [-d] [-r 1|2] <path>"
                echo
                echo "    -r 1|2  -- ROS versio (default: detect from ROS_DISTRO environment variable)"
                echo "    -s      -- Skip Fixposition SDK check"
                echo "    -d      -- Setup for debug build (default: release)"
                echo "    <path>  -- Path to workspace root (absolute, or relative to ${SCRIPTDIR})"
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

    # Path can be relative to script directory (= fixposition_dirver repo root) or absolute
    if [ -z "${1:-}" ]; then
        exit_fail "Need a path"
    fi
    local abspath=$1
    local relpath=
    if [ "${abspath:0:1}" != "/" ]; then
        relpath=${abspath}
        abspath=${SCRIPTDIR}/${relpath}
    fi
    debug "abspath=${abspath} relpath=${relpath}"
    if [ -f ${abspath} -o -d ${abspath} ]; then
        exit_fail "Path ${abspath} already exists"
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

    # Create workspace
    if [ ${dev} -gt 0 ]; then
        notice "Creating debug ROS${rosver} workspace in ${abspath}"
    else
        notice "Creating release ROS${rosver} workspace in ${abspath}"
    fi
    mkdir -p ${abspath}/src

    # Path for symlink
    local srcpath=
    if [ -n "${relpath}" ]; then
        srcpath=$(realpath -s --relative-to=${abspath}/src ${SCRIPTDIR})
    else
        srcpath=${SCRIPTDIR}
    fi
    debug "srcpath=${srcpath}"

    # Symlink source packages
    ln -s ${srcpath}/fixposition-sdk/fpsdk_common         ${abspath}/src
    ln -s ${srcpath}/fixposition-sdk/fpsdk_apps           ${abspath}/src
    ln -s ${srcpath}/fixposition_driver_lib               ${abspath}/src
    ln -s ${srcpath}/fixposition_driver_msgs              ${abspath}/src
    ln -s ${srcpath}/rtcm_msgs                            ${abspath}/src
    if [ ${rosver} -eq 1 ]; then
        ln -s ${srcpath}/fixposition-sdk/fpsdk_ros1           ${abspath}/src
        ln -s ${srcpath}/fixposition_driver_ros1              ${abspath}/src
    else
        ln -s ${srcpath}/fixposition-sdk/fpsdk_ros2           ${abspath}/src
        ln -s ${srcpath}/fixposition_driver_ros2              ${abspath}/src
    fi

    # Initialise workspace
    cd ${abspath}
    if [ ${rosver} -eq 1 ]; then
        catkin init
        if [ ${dev} -gt 0 ]; then
            catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug
        else
            catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
        fi
    else
        if [ ${dev} -gt 0 ]; then
            echo 'build:' > ${abspath}/colcon_defaults.yaml
            echo '    event-handlers: [ "console_direct+" ]' >> ${abspath}/colcon_defaults.yaml
            echo '    cmake-args: [ "-DCMAKE_BUILD_TYPE=Debug", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" ]' >> ${abspath}/colcon_defaults.yaml
        else
            echo 'build:' > ${abspath}/colcon_defaults.yaml
            echo '#    event-handlers: [ "console_direct+" ]' >> ${abspath}/colcon_defaults.yaml
            echo '    cmake-args: [ "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" ]' >> ${abspath}/colcon_defaults.yaml
        fi
    fi

    # Happy?
    debug "res=${res}"
    if [ ${res} -eq 0 ]; then
        info "Splendid! You should now be able to:"
        info ""
        info "    cd ${relpath}"
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
