// ROS 2 libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

class SampleLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    /// SampleLifecycleNode constructor
    SampleLifecycleNode() : LifecycleNode{"lifecycle_node"}
    {
    }

    /**
     * Callback called when the lifecycle node enters the "configuring" state.
     * Depending on the return value of this function, the state machine
     * either invokes a transition to the "inactive" state or stays in "unconfigured".
     * TRANSITION_CALLBACK_SUCCESS transitions to "inactive".
     * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured".
     * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing".
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Attempting a 'configure' transition from the '%s' state.",
                    state.label().c_str());

        // Initialize your attributes and configure your communications here.

        return LifecycleNode::on_configure(state);
    }

    /**
     * Callback called when the lifecycle node enters the "activating" state.
     * Depending on the return value of this function, the state machine
     * either invokes a transition to the "active" state or stays in "inactive".
     * TRANSITION_CALLBACK_SUCCESS transitions to "active".
     * TRANSITION_CALLBACK_FAILURE transitions to "inactive".
     * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing".
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Attempting an 'activate' transition from the '%s' state.",
                    state.label().c_str());

        return LifecycleNode::on_activate(state);
    }

private:
    // Callbacks

    // Attributes
};

int main(int argc, char *argv[])
{
    // Force flush the stdout buffer.
    // This ensures a correct synchronization of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    std::shared_ptr<SampleLifecycleNode> node = std::make_shared<SampleLifecycleNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();

    return 0;
}