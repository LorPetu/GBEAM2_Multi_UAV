//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "gbeam2_interfaces/msg/vertex.hpp"
#include "gbeam2_interfaces/msg/graph_edge.hpp"
#include "gbeam2_interfaces/msg/poly_area.hpp"
#include "gbeam2_interfaces/msg/graph.hpp"

#include "gbeam2_interfaces/msg/status.hpp"
#include "gbeam2_interfaces/msg/frontier_stamped.hpp"


class CooperationNode : public rclcpp::Node
{
public:
  CooperationNode() : Node("coop_manager"){

    //SUBSCRIBED TOPICS
    merged_graph_sub_= this->create_subscription<gbeam2_interfaces::msg::Graph>(
      "gbeam/merged_graph",1,std::bind(&CooperationNode::mergedGraphCallback,this,std::placeholders::_1));

    //PUBLISHING TOPICS
    assigned_graph_pub_ = this->create_publisher<gbeam2_interfaces::msg::Graph>(
      "gbeam/assigned_graph",1);

   // Get namespace
    name_space = this->get_namespace();
    name_space_id = name_space.back()- '0';

    //Initialize parameters

    

  }

private:
  std::string name_space;
  int name_space_id;


  //adjacency matrix is related to that graph
  gbeam2_interfaces::msg::Graph merged_graph; 
  gbeam2_interfaces::msg::Graph assigned_graph;

  // Declare topics variables
  rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr merged_graph_sub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr assigned_graph_pub_;

  void mergedGraphCallback(const std::shared_ptr<gbeam2_interfaces::msg::Graph> merged_graph_ptr){
    

  }

};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CooperationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
