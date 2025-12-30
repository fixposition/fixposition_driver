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
 * @brief Fixposition lifecycle driver node for ROS2
 */

/* LIBC/STL */
#include <chrono>
#include <memory>

/* EXTERNAL */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_ros2/utils.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/* PACKAGE */
#include "fixposition_driver_ros2/fixposition_driver_node.hpp"
#include "fixposition_driver_ros2/params.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

class FixpositionDriverLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
   public:
    FixpositionDriverLifecycleNode() : rclcpp_lifecycle::LifecycleNode("fixposition_driver") {}

   protected:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State&) override {
        RCLCPP_INFO(get_logger(), "Configuring...");
        auto node = shared_from_this();

        if (!LoadParamsFromRos2(node, driver_params_)) {
            RCLCPP_ERROR(get_logger(), "Failed loading sensor params");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        if (!LoadDiagnosticsParamsFromRos2(node, diagnostics_params_)) {
            RCLCPP_WARN(get_logger(), "Failed loading diagnostics params, using defaults");
        }

        try {
            node_ = std::make_unique<FixpositionDriverNode>(node, driver_params_, diagnostics_params_);
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(get_logger(), "Failed creating node: %s", ex.what());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State&) override {
        RCLCPP_INFO(get_logger(), "Activating...");
        if (!node_) {
            RCLCPP_ERROR(get_logger(), "No node configured");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        if (!node_->StartNode()) {
            RCLCPP_ERROR(get_logger(), "Failed starting node");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State&) override {
        RCLCPP_INFO(get_logger(), "Deactivating...");
        if (node_) {
            node_->StopNode();
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State&) override {
        RCLCPP_INFO(get_logger(), "Cleaning up...");
        if (node_) {
            node_->StopNode();
            node_.reset();
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State&) override {
        RCLCPP_INFO(get_logger(), "Shutting down...");
        if (node_) {
            node_->StopNode();
            node_.reset();
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

   private:
    DriverParams driver_params_;
    DiagnosticsParams diagnostics_params_;
    std::unique_ptr<FixpositionDriverNode> node_;
};

/* ****************************************************************************************************************** */
}  // namespace fixposition

using namespace fixposition;

int main(int argc, char** argv) {
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
    WARNING("***** Running debug build *****");
#endif

    rclcpp::init(argc, argv);

    auto node = std::make_shared<FixpositionDriverLifecycleNode>();
    fpsdk::ros2::utils::RedirectLoggingToRosConsole(node->get_logger().get_name());
    HelloWorld();

    fpsdk::common::app::SigIntHelper sigint;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    while (rclcpp::ok() && !sigint.ShouldAbort()) {
        executor.spin_once(std::chrono::milliseconds(200));
    }

    rclcpp::shutdown();
    return 0;
}
