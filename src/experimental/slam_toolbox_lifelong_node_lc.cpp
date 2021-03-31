/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Steve Macenski
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: PGotzmann */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include "slam_toolbox/experimental/slam_toolbox_lifelong.hpp"


class LifecycleSyncSlamToolbox : public rclcpp_lifecycle::LifecycleNode {
public:

    explicit LifecycleSyncSlamToolbox(const std::string &node_name, rclcpp::executors::SingleThreadedExecutor *executor,
                                      bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(node_name,
                                              rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
              executor(executor) {}

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &) override {
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");

        int stack_size = 40000000;
        {
            auto temp_node = std::make_shared<rclcpp::Node>("slam_toolbox");
            temp_node->declare_parameter("stack_size_to_use");
            if (temp_node->get_parameter("stack_size_to_use", stack_size)) {
                RCLCPP_INFO(temp_node->get_logger(), "Node using stack size %i", (int) stack_size);
                const rlim_t max_stack_size = stack_size;
                struct rlimit stack_limit;
                getrlimit(RLIMIT_STACK, &stack_limit);
                if (stack_limit.rlim_cur < stack_size) {
                    stack_limit.rlim_cur = stack_size;
                }
                setrlimit(RLIMIT_STACK, &stack_limit);
            }
        }

        rclcpp::NodeOptions options;
        lifelong_slam_toolbox = std::make_shared<slam_toolbox::LifelongSlamToolbox>(options);

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override {
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
        lifelong_slam_toolbox->configure();
        lifelong_slam_toolbox->loadPoseGraphByParams();
        executor->add_node(lifelong_slam_toolbox->get_node_base_interface());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &) override {
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
        executor->remove_node(lifelong_slam_toolbox->get_node_base_interface());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override {
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
        lifelong_slam_toolbox->shutdown();
        lifelong_slam_toolbox.reset();

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &state) override {
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    rclcpp::executors::SingleThreadedExecutor *executor;
    std::shared_ptr<slam_toolbox::LifelongSlamToolbox> lifelong_slam_toolbox;
};

int main(int argc, char **argv) {

    auto nodeName = "slam_toolbox_lc";
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<LifecycleSyncSlamToolbox> lc_node =
            std::make_shared<LifecycleSyncSlamToolbox>(nodeName, &exe);

    exe.add_node(lc_node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
