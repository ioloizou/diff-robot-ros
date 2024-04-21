#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

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
      // Initialize the transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      timer_ = this->create_wall_timer(500ms, std::bind(&DiffControlNode::move_robot, this));
    }
    
  private:
    
    void move_robot()
    {
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "base_footprint";
      t.child_frame_id = "base_link";

      t.transform.translation.x = 2;
      t.transform.translation.y = 2;
      t.transform.translation.z = 0;

      tf2::Quaternion q;
      q.setRPY(0, 0, 3.14/2);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      // Send the transformation
      tf_broadcaster_->sendTransform(t);

      // Printing Sucessful transfom
      std::cout<<"The transform has taken place re koumpare"<<std::endl;
    }

    // declare any subscriber / publisher / member variables and functions
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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