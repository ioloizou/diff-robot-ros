#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <algorithm>

// access time units such as 100ms
using namespace std::chrono_literals;

// some shortcuts for message classes
using geometry_msgs::msg::Twist;
using builtin_interfaces::msg::Time;

class DiffControlNode : public rclcpp::Node
{
  public:
    DiffControlNode() : Node("controller"), count_(0)
    {      
      // Init publisher
      cmd_pub_ = this->create_publisher<Time>("koumparos", 10);

      // init timer - the function publishCommand() should called with the given rate
      timer_ = this->create_wall_timer(100ms, std::bind(&DiffControlNode::timer_callback, this));
    }
    
  private:
    
    void timer_callback()
    {
      Time time = this->get_clock()->now();
      cmd_pub_->publish(time);
    }

    // declare any subscriber / publisher / member variables and functions
    rclcpp::Publisher<Time>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;  

};

int main(int argc, char** argv)
{   
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffControlNode>());
  rclcpp::shutdown();
  return 0;
}