#include "rclcpp/rclcpp.hpp"


class MyNode : public rclcpp::Node
{
public:
    MyNode(): Node("cpp_oop"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello from OOP world"); // as we are already in a node class we dont need to metion node-> and in fact we dont even need to mention this->  we do it for the sake of our understanding that we are dealing with an object. it works even without this-> so we can directly access the methods of the node class directly in the MyNode child
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode::timerCallback, this));
    }


private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello %d ", counter_);
        counter_ ++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;



};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();


    return 0;
}