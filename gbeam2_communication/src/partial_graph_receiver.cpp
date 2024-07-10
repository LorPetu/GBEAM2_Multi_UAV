//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
#include <chrono>
#include <vector>
#include <functional>
#include <map>
#include <algorithm>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "gbeam2_interfaces/msg/vertex.hpp"
#include "gbeam2_interfaces/msg/graph_edge.hpp"
#include "gbeam2_interfaces/msg/poly_area.hpp"
#include "gbeam2_interfaces/msg/graph.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "gbeam2_interfaces/msg/status.hpp"
#include "gbeam2_interfaces/msg/frontier_stamped.hpp"

class PartialGraphRXNode : public rclcpp::Node
{
public:
  PartialGraphRXNode() : Node("partial_graph_RX"){

    // Retrieve the number of robots from parameters
    this->declare_parameter<int>("num_robots", 2);
    int num_robots = this->get_parameter("num_robots").get_parameter_value().get<int>();
    RCLCPP_INFO(this->get_logger(),"################# PARAMETERS OF communication ############");
    RCLCPP_INFO(this->get_logger(),"1) num_robots: %d", num_robots);

    // Get the namespace of this node to avoid subscribing to its own topic
    std::string own_namespace = this->get_namespace();

    // Derive topic names and create message filters subscribers
    for (int i = 1; i <= num_robots; ++i) {
      std::string topic = "/robot" + std::to_string(i) + "/cmd_vel";
      if (own_namespace != "/robot" + std::to_string(i)) {
        auto sub = std::make_shared<message_filters::Subscriber<DataType>>(this, topic);
        subscribers_.push_back(sub);
      }
    }

    // Create a synchronizer with the appropriate policy
    setup_synchronizer();

    // Publisher
    pub_ = this->create_publisher<std_msgs::msg::String>("synced_topic", 10);

  };

private:

  using DataType = geometry_msgs::msg::Twist; // Here we eill use graph message
  void setup_synchronizer()
  {
    // Check the number of subscribers to determine the policy
    switch (subscribers_.size()) {
      case 1:
        // Direct subscription, no synchronization needed
        subscribers_[0]->registerCallback(std::bind(&PartialGraphRXNode::callback1, this, std::placeholders::_1));
        break;
      case 2:
        sync_2_ = std::make_shared<message_filters::Synchronizer<ApproxTime2>>(ApproxTime2(10), *subscribers_[0], *subscribers_[1]);
        sync_2_->registerCallback(std::bind(&PartialGraphRXNode::callback2, this, std::placeholders::_1, std::placeholders::_2));
        break;
      case 3:
        sync_3_ = std::make_shared<message_filters::Synchronizer<ApproxTime3>>(ApproxTime3(10), *subscribers_[0], *subscribers_[1], *subscribers_[2]);
        sync_3_->registerCallback(std::bind(&PartialGraphRXNode::callback3, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        break;
      // Add cases for more subscribers if needed
      default:
        RCLCPP_ERROR(this->get_logger(), "Number of robots not supported");
        break;
    }
  }

  void callback1(const DataType::ConstSharedPtr msg)
  {
    // Process a single message and publish a new message
    auto output_msg = std_msgs::msg::String();
    output_msg.data = "Single: " + std::to_string(msg->linear.x);
    pub_->publish(output_msg);
  }

  void callback2(const DataType::ConstSharedPtr msg1, const DataType::ConstSharedPtr msg2)
  {
    // Process synchronized messages and publish a new message
    auto output_msg = std_msgs::msg::String();
    output_msg.data = "Combined: " + std::to_string(msg1->linear.x) + ", " + std::to_string(msg2->linear.x);
    pub_->publish(output_msg);
  }

  void callback3(const DataType::ConstSharedPtr msg1, const DataType::ConstSharedPtr msg2, const DataType::ConstSharedPtr msg3)
  {
    // Process synchronized messages and publish a new message
    auto output_msg = std_msgs::msg::String();
    output_msg.data = "Combined: " + std::to_string(msg1->linear.x) + ", " + std::to_string(msg2->linear.x) + ", " + std::to_string(msg3->linear.x);
    pub_->publish(output_msg);
  }

  // Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  // Message filters subscribers
  std::vector<std::shared_ptr<message_filters::Subscriber<DataType>>> subscribers_;

  // Synchronizers
  typedef message_filters::sync_policies::ApproximateTime<DataType, DataType> ApproxTime2;
  typedef message_filters::sync_policies::ApproximateTime<DataType, DataType, DataType> ApproxTime3;
  
  std::shared_ptr<message_filters::Synchronizer<ApproxTime2>> sync_2_;
  std::shared_ptr<message_filters::Synchronizer<ApproxTime3>> sync_3_;



};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PartialGraphRXNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
