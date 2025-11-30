#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;

class NumberCounterNode : public rclcpp::Node
{
public :
    NumberCounterNode() : Node("number_counter"), count_(0)
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10, std::bind(&NumberCounterNode::callbackCounter,
        this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), " Number Counter has been started");
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
    }



private :

    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_ ;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;


    void callbackCounter(const example_interfaces::msg::Int64::SharedPtr msg)
    {
       
        count_ = count_ + msg->data;
       
       //RCLCPP_INFO(this->get_logger(), "Current count: %d", count_);
       auto message = example_interfaces::msg::Int64();
       message.data = count_;
       publisher_ ->publish(message);
        
    }
    int  count_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}