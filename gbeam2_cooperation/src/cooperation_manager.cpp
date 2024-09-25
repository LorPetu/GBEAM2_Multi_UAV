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
#include "library_fcn.hpp"


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

     // Initialize parameters
    this->declare_parameter<int>("N_robot",0);
    this->declare_parameter<double>("communication_range",0.0);

    // Get parameters
    N_robot = this->get_parameter("N_robot").get_parameter_value().get<int>();
    wifi_range = this->get_parameter("communication_range").get_parameter_value().get<double>();

    RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF COOPERATION: ############# ");
    RCLCPP_INFO(this->get_logger(),"############# (for %s) ############# ",name_space.c_str());
    RCLCPP_INFO(this->get_logger(),"1) Number of robots: %d",N_robot);
    RCLCPP_INFO(this->get_logger(),"1) Number of robots: %f",wifi_range);
    // Initialize vectors with the correct size
    stored_Graph.resize(N_robot);
    

    

  }

private:
  std::string name_space;
  int name_space_id;

  // Parameters variables
  int N_robot;
  double wifi_range;

  std::vector<std::shared_ptr<gbeam2_interfaces::msg::Graph>> stored_Graph;
  int last_updated_node;
  int last_updated_edge;


  //adjacency matrix is related to that graph
  gbeam2_interfaces::msg::Graph merged_graph; 
  gbeam2_interfaces::msg::Graph assigned_graph;

  // Declare topics variables
  rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr merged_graph_sub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr assigned_graph_pub_;

  void mergedGraphCallback(const std::shared_ptr<gbeam2_interfaces::msg::Graph> merged_graph_ptr){
    
    if(merged_graph.robot_id!=name_space_id){
      int req_robot_id = merged_graph.robot_id;
      last_updated_node = stored_Graph[req_robot_id]->nodes.back().id;
      last_updated_edge = stored_Graph[req_robot_id]->edges.back().id;
      for(gbeam2_interfaces::msg::Vertex node: merged_graph.nodes){

        if(node.id<last_updated_node){
          stored_Graph[req_robot_id]->nodes[node.id]=node;          
        }
        else{
          stored_Graph[req_robot_id]->nodes.push_back(node);
        }

      }

      for(gbeam2_interfaces::msg::GraphEdge edge : merged_graph.edges){
        
        if(edge.id<last_updated_edge){
          stored_Graph[req_robot_id]->edges[edge.id]=edge;          
        }
        else{
          stored_Graph[req_robot_id]->edges.push_back(edge);
        }
      }

    }


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
