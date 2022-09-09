#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>

using namespace std::chrono_literals;

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MocapPublisher : public rclcpp::Node
{
  public:
    MocapPublisher()
    : Node("mocap_publisher"), count_(0)
    {
      // publisher
      px4_publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("fmu/vehicle_visual_odometry/in", 1);

      // timesync subscription
      timesync_subscription_ = this->create_subscription<px4_msgs::msg::Timesync>("/fmu/timesync/out", 10,
        [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
          timestamp_.store(msg->timestamp);
        });

      // vrpn subscription
      vrpn_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/RigidBody1/pose",10,std::bind(&MocapPublisher::topic_callback, this, _1));

      // Check if subscription is successfull and then initiate timer
      RCLCPP_INFO(this->get_logger(), "Waiting for first mocap message");
      rclcpp::WaitSet wait_set;
      wait_set.add_subscription(vrpn_subscription_);
      auto ret = wait_set.wait(std::chrono::seconds(100));
      if (ret.kind() == rclcpp::WaitResultKind::Ready) {
        geometry_msgs::msg::PoseStamped msg; // if successfull, store data
        rclcpp::MessageInfo info;
        auto ret_take = vrpn_subscription_->take(msg, info);
        if (ret_take) {
          RCLCPP_INFO(this->get_logger(), "heard vrpn, initializing px4_publisher now");
          timer_ = this->create_wall_timer(
            8ms, std::bind(&MocapPublisher::timer_callback, this));
        } 
        else {
          RCLCPP_ERROR(this->get_logger(), "no message recieved from vrpn");
        }
      } 
      else {
        RCLCPP_ERROR(this->get_logger(), "couldn't wait for vrpn message");
        return;
      }

    }

  private:
    void timer_callback()
    {
      auto message = px4_msgs::msg::VehicleVisualOdometry();

      // message.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
      message.timestamp = timestamp_.load();

      // message.timestamp_sample = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
      message.timestamp_sample = timestamp_.load();

      message.local_frame = 0;

      if (time_stamp == time_stamp_ref){
        count++;
      }
      else{
        count = 0;
        time_stamp_ref = time_stamp;
      }

      if (count > 60){
        RCLCPP_INFO(this->get_logger(), "MOCAP Failure - Exiting & Landing");
        exit(0);
      }

      // Node exits if same time_stamp is realyed for 0.5 seconds (60 times = 1/2 of 120 Hz)

      message.x = x_mocap;
      message.y = -y_mocap;
      message.z = -z_mocap;

      // If you have quaternion values, include here. Else set 1st element to NAN
      // message.q[0] = NAN; // uncomment this to set first element to NAN. 
      message.q[0] = q_mocap[3];
      message.q[1] = q_mocap[0];
      message.q[2] = -q_mocap[1];
      message.q[3] = -q_mocap[2];
      message.q_offset[0] = NAN;
      
      message.pose_covariance[0] = NAN;
      message.pose_covariance[15] = NAN;
      
      message.velocity_frame = 0;

      message.vx = NAN;
      message.vy = NAN;
      message.vz = NAN;

      message.rollspeed = NAN;
      message.pitchspeed = NAN;
      message.yawspeed = NAN;
      
      message.velocity_covariance[0] = NAN;
      message.velocity_covariance[15] = NAN; 


      //RCLCPP_INFO(this->get_logger(), "Publishing x y z: '%f' '%f' '%f'", message.x, message.y, message.z);
      px4_publisher_->publish(message);
    }

    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->pose.position.x);

      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->header.stamp.sec);

      time_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9;

      RCLCPP_INFO(this->get_logger(), "Time_Stamp: '%f",time_stamp);

      x_mocap = msg->pose.position.x;
      y_mocap = msg->pose.position.y;
      z_mocap = msg->pose.position.z;
      
      // storing quaternion values
      q_mocap[0] = msg->pose.orientation.x;
      q_mocap[1] = msg->pose.orientation.y;
      q_mocap[2] = msg->pose.orientation.z;
      q_mocap[3] = msg->pose.orientation.w;	

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr px4_publisher_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vrpn_subscription_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    size_t count_;

    // float x_mocap,y_mocap,z_mocap,vx,vy,vz,rollspeed,pitchspeed,yawspeed;
    float x_mocap,y_mocap,z_mocap;
    float x_position = 0.0, y_position = 0.0, z_position = 0.0;
    
    double time_stamp = 0.0 , time_stamp_ref = 0.0;

    int count = 0;

    Eigen::Vector4f q_mocap; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapPublisher>());
  rclcpp::shutdown();
  return 0;
}