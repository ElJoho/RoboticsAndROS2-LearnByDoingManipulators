#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecyle/lifecycle_node.hpp>

class SImpleLifecycleNode : public rclcpp_lifecycle::LifeCycleNode{
    public:
        explicit SImpleLifecycleNode(
            const std::string & node_name,
            bool intra_process_comns = false
        ): LyfecycleNode(
            node_name,
            rclcpp::NodeOptions().use_intra_process_comms()
        )
};