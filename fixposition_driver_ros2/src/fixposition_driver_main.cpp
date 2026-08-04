/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition driver node main for ROS2
 */

/* LIBC/STL */
#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

/* EXTERNAL */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_ros2/utils.hpp>

/* PACKAGE */
#include "fixposition_driver_ros2/fixposition_driver_node.hpp"
#include "fixposition_driver_ros2/params.hpp"

using namespace fixposition;

int main(int argc, char** argv) {
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
    WARNING("***** Running debug build *****");
#endif

    bool ok = true;

    // Initialise ROS, create node handle
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("fixposition_driver");
    auto logger = nh->get_logger();

    // Redirect Fixposition SDK logging to ROS console
    fpsdk::ros2::utils::RedirectLoggingToRosConsole(logger.get_name());

    // Say hello
    HelloWorld();

    // Load parameters
    RCLCPP_INFO(logger, "Loading parameters...");
    DriverParams driver_params;
    if (!LoadParamsFromRos2(nh, driver_params)) {
        RCLCPP_ERROR(logger, "Failed loading sensor params");
        ok = false;
    }
    DiagnosticsParams diag_params;
    if (!LoadDiagnosticsParamsFromRos2(nh, diag_params)) {
        RCLCPP_WARN(logger, "Failed loading diagnostics params, using defaults");
    }

    // Handle CTRL-C / SIGINT ourselves
    fpsdk::common::app::SigIntHelper sigint;

    // Start node
    std::unique_ptr<FixpositionDriverNode> node;
    if (ok) {
        try {
            node = std::make_unique<FixpositionDriverNode>(nh, driver_params, diag_params);
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(logger, "Failed creating node: %s", ex.what());
            ok = false;
        }
    }
    if (ok) {
        RCLCPP_INFO(logger, "Starting node...");
        if (node->StartNode()) {
            RCLCPP_INFO(logger, "main() spinning...");

            // Do the same as rclpp::spin(), but also handle CTRL-C / SIGINT nicely
            // Callbacks execute in main thread
            while (rclcpp::ok() && !sigint.ShouldAbort()) {
                rclcpp::spin_until_future_complete(nh, std::promise<bool>().get_future(),
                                                   std::chrono::milliseconds(337));
            }

            RCLCPP_INFO(logger, "main() stopping");
        } else {
            RCLCPP_ERROR(logger, "Failed starting node");
            ok = false;
        }
        node->StopNode();
        node.reset();
        nh.reset();
    }

    // Are we happy?
    if (ok) {
        RCLCPP_INFO(logger, "Done");
    } else {
        RCLCPP_FATAL(logger, "Ouch!");
    }

    // Shutdown ROS
    rclcpp::shutdown();

    exit(ok ? EXIT_SUCCESS : EXIT_FAILURE);
}
