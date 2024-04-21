/*
- Add slider publisher
- Add rotation
- Add wheel rotation based on speed and rotation
*/



#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <algorithm>

// access time units such as 100ms
using namespace std::chrono_literals;

// some shortcuts for message classes
using geometry_msgs::msg::Twist;

// time interval for publishing
std::chrono::milliseconds dt_milliseconds_= 10ms;

class DiffControlNode : public rclcpp::Node
{
  public:
    DiffControlNode() : Node("controller"), count_(0)
    {      
      // Initializing the transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      timer_ = this->create_wall_timer(dt_milliseconds_, std::bind(&DiffControlNode::move_robot, this));
    }
    
  private:
    
    void move_robot()
    {
      geometry_msgs::msg::TransformStamped t;

      // Transforming milliseconds to seconds
      double dt_seconds_ = std::chrono::duration<double>(dt_milliseconds_).count();
      // meters/second
      double v_=0.1; 
      rclcpp::Time now = this->get_clock()->now();

      // Computing travelled distance in x direction
      translation_x_ += v_*dt_seconds_;

      // Defining the transforms translation
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "base_footprint";
      t.child_frame_id = "base_link";

      t.transform.translation.x = translation_x_;
      t.transform.translation.y = 0;
      t.transform.translation.z = 0;

      // Defining the transforms rotation in euler angles and transforming to quartenions
      tf2::Quaternion q;
      q.setRPY(0, 0, 3.14);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      // Send the transformation
      tf_broadcaster_->sendTransform(t);

      // Printing sucessful transfom
      std::cout<<"The transform has taken place re koumpare: "<<t.transform.translation.x<<"m"<<std::endl;
    }

    // Declaring subscriber / publisher / member variables and functions
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;  
    double translation_x_ = 0;
      
};

int main(int argc, char** argv)
{   
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffControlNode>());
  rclcpp::shutdown();
  return 0;
}