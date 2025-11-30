#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("cpp_test");
//  if we type node.something we are interacting with the pointer so we cant use the node object properties, so we use -> to call the node object and we can use all the methods of a Node class in our object
    RCLCPP_INFO(node->get_logger(), "Hello world");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}