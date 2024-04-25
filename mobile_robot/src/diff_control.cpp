/*
- Add wheel rotation based on speed and rotation
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <algorithm>
#include <cmath>

// access time units such as 100ms
using namespace std::chrono_literals;

// some shortcuts for message classes
using geometry_msgs::msg::Twist;
using sensor_msgs::msg::JointState;

// time interval for publishing
std::chrono::milliseconds dt_milliseconds_= 50ms;

class DiffControlNode : public rclcpp::Node
{
  public:
    DiffControlNode() : Node("controller"), count_(0)
    {      

      joint_state_msg_.name = {"wheel_right_joint", "wheel_left_joint"};
      joint_state_msg_.position.resize(2,0);

      // Initializing the transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      timer_ = this->create_wall_timer(dt_milliseconds_, std::bind(&DiffControlNode::moveRobot, this));

      // Initializing the subscriber to the slider publisher
      twist_subscription_ = this->create_subscription<Twist>("cmd_vel", 10, std::bind(&DiffControlNode::cmdVelCallback, this, std::placeholders::_1));

      // Initializing the pulisher to joint state
      joint_state_publisher_= this->create_publisher<JointState>("joint_states", 10);

    }
    
  private:
    
    void cmdVelCallback(const Twist& msg)
    {
      // meters/second
      v_ = msg.linear.x;

      // rad/seconds
      w_ = msg.angular.z;
    }

    // 
    void moveRobot()
    {
      geometry_msgs::msg::TransformStamped t;

      // Transforming milliseconds to seconds
      double dt_seconds_ = std::chrono::duration<double>(dt_milliseconds_).count();

      rclcpp::Time now = this->get_clock()->now();

      // Computing travelled distance
      delta_theta_ += w_*dt_seconds_;
      translation_x_ += v_*dt_seconds_*cos(delta_theta_);
      translation_y_ += v_*dt_seconds_*sin(delta_theta_);

      // Defining the transforms translation
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "base_footprint";
      t.child_frame_id = "base_link";

      t.transform.translation.x = translation_x_;
      t.transform.translation.y = translation_y_;
      t.transform.translation.z = 0;

      // Defining the transforms rotation in euler angles and transforming to quartenions
      tf2::Quaternion q;
      q.setRPY(0, 0, delta_theta_);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      // Send the transformation
      tf_broadcaster_->sendTransform(t);

      // Printing sucessful transfom
      std::cout<<"The transform has taken place re koumpare: "<<t.transform.translation.x<<"m"<<std::endl;

      // Calculating wheel rotation based on commanded velocity
      double wheel_radius_ = 0.15;
      double wheel_separation = 0.40;

      delta_d_ += v_*dt_seconds_;
      double delta_qr_ = (1/wheel_radius_)*delta_d_ - wheel_separation/wheel_radius_*delta_theta_;
      double delta_ql_ = (1/wheel_radius_)*delta_d_ + wheel_separation/wheel_radius_*delta_theta_;

      // Publishing constant joint state command
      joint_state_msg_.position[0] = delta_qr_;
      joint_state_msg_.position[1] = delta_ql_;

      joint_state_msg_.header.stamp = this->get_clock()->now();

      joint_state_publisher_->publish(joint_state_msg_);
    }

    // Declaring subscriber / publisher / member variables and functions
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<Twist>::SharedPtr twist_subscription_;
    rclcpp::Publisher<JointState>::SharedPtr joint_state_publisher_;

    JointState joint_state_msg_;

    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;  

    double v_ = 0;
    double w_ = 0;
    double translation_x_ = 0;
    double translation_y_ = 0;
    double delta_theta_ = 0;
    double delta_d_ = 0;
      
};

int main(int argc, char** argv)
{   
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffControlNode>());
  rclcpp::shutdown();
  return 0;
}