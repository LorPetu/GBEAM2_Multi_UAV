//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
#include <utility>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Vector3.h"

#include "gbeam2_interfaces/msg/vertex.hpp"
#include "gbeam2_interfaces/msg/graph_edge.hpp"
#include "gbeam2_interfaces/msg/poly_area.hpp"
#include "gbeam2_interfaces/msg/graph.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "gbeam2_interfaces/msg/status.hpp"
#include "gbeam2_interfaces/msg/frontier_stamped.hpp"
#include "library_fcn.hpp"

#define INF 100000

class CooperationNode : public rclcpp::Node
{
public:
  CooperationNode() : Node("coop_manager"){

    //SUBSCRIBED TOPICS
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            name_space+ "odom", 1, std::bind(&CooperationNode::odomCallback, this, std::placeholders::_1));

    merged_graph_sub_= this->create_subscription<gbeam2_interfaces::msg::Graph>(
      "gbeam/merged_graph",1,std::bind(&CooperationNode::mergedGraphCallback,this,std::placeholders::_1));
    

    std::string filter_string = "robot_id!=%0";          
    rclcpp::SubscriptionOptions options;
    options.content_filter_options.filter_expression = filter_string;
    options.content_filter_options.expression_parameters = {std::to_string(name_space_id)};
    
    status_sub_ = this->create_subscription<gbeam2_interfaces::msg::Status>("/status", 1,
              std::bind(&CooperationNode::statusCallback, this, std::placeholders::_1),//);
              options);

    //PUBLISHING TOPICS
    assigned_graph_pub_ = this->create_publisher<gbeam2_interfaces::msg::Graph>(
      "gbeam/assigned_graph",1);

    frontier_pub_ = this->create_publisher<gbeam2_interfaces::msg::FrontierStamped>(
      "frontier",1);

     start_frontiers_service_ = this->create_service<std_srvs::srv::SetBool>(
        "start_frontier",std::bind(&CooperationNode::startFrontier,this, std::placeholders::_1, std::placeholders::_2));


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
    last_status.resize(N_robot);
    last_status[name_space_id].robot_id = name_space_id;
    for (int i = 0; i < N_robot; i++)
    {
      stored_Graph[i] = std::make_shared<gbeam2_interfaces::msg::Graph>();
      last_status[i].connection_status.resize(N_robot);
      last_status[i].joint_vector.resize(N_robot);
      last_status[i].normal_joint_vector.resize(N_robot);
    }

    

    

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
  bool start_frontier = false;

  std::vector<gbeam2_interfaces::msg::Status> last_status;
  nav_msgs::msg::Odometry robot_odom_;


  //adjacency matrix is related to that graph
  gbeam2_interfaces::msg::Graph merged_graph; 
  gbeam2_interfaces::msg::Graph assigned_graph;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Declare topics variables
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr merged_graph_sub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::FrontierStamped>::SharedPtr frontier_pub_;
  rclcpp::Subscription<gbeam2_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr assigned_graph_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  start_frontiers_service_;

   void startFrontier(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
        start_frontier=request->data;     
    }
  double sideOfLine(geometry_msgs::msg::Point lineStart, geometry_msgs::msg::Point lineEnd, gbeam2_interfaces::msg::Vertex point) {
    return (lineEnd.x - lineStart.x) * (point.y - lineStart.y) - 
           (lineEnd.y - lineStart.y) * (point.x - lineStart.x);
}

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr)
  {
      robot_odom_ = *odom_ptr;
      //RCLCPP_INFO(this->get_logger(),"odom received: x:%f y:%f",robot_odom_.pose.pose.position.x,robot_odom_.pose.pose.position.y);
  }

  void mergedGraphCallback(const std::shared_ptr<gbeam2_interfaces::msg::Graph> graph_received){
  

    if(graph_received->robot_id!=name_space_id){
     
      int req_robot_id = graph_received->robot_id;
      last_updated_node = (stored_Graph[req_robot_id]->nodes.empty()) ? -1 : stored_Graph[req_robot_id]->nodes.back().id;
      last_updated_edge = (stored_Graph[req_robot_id]->edges.empty()) ? -1 : stored_Graph[req_robot_id]->edges.back().id;

      for(gbeam2_interfaces::msg::Vertex node: graph_received->nodes){

        if(node.id<last_updated_node){
          stored_Graph[req_robot_id]->nodes[node.id]=node;          
        }
        else{
          stored_Graph[req_robot_id]->nodes.push_back(node);
        }

      }

      for(gbeam2_interfaces::msg::GraphEdge edge : graph_received->edges){
        
        if(edge.id<last_updated_edge){
          stored_Graph[req_robot_id]->edges[edge.id]=edge;          
        }
        else{
          stored_Graph[req_robot_id]->edges.push_back(edge);
        }
      }

    } else{
      stored_Graph[name_space_id] = graph_received;
    }


  }

  void statusCallback(const gbeam2_interfaces::msg::Status::SharedPtr received_status){
    last_status[received_status->robot_id]=*received_status;
    // if the received robot has a connection with this robot AND last frontier is explored?
    //if(!start_frontier) return;

    if(received_status->connection_status[name_space_id]){
      
      //RCLCPP_INFO(this->get_logger(), "I have connection with robot %d",received_status->robot_id);
      gbeam2_interfaces::msg::FreePolygonStamped received_poly = get_obstacles_and_reachable_nodes(stored_Graph[received_status->robot_id]);
      gbeam2_interfaces::msg::FreePolygonStamped my_poly = get_obstacles_and_reachable_nodes(stored_Graph[name_space_id]);

      //RCLCPP_INFO(this->get_logger(), "Get obstacles and reachables");
      std::vector<gbeam2_interfaces::msg::Vertex> merged_obstacles = my_poly.polygon.vertices_obstacles;
      std::vector<gbeam2_interfaces::msg::Vertex> obstacles_left;
      std::vector<gbeam2_interfaces::msg::Vertex> obstacles_right;
      std::vector<gbeam2_interfaces::msg::Vertex> merged_reachables = my_poly.polygon.vertices_reachable;
      std::vector<gbeam2_interfaces::msg::Vertex> reachables_left;
      std::vector<gbeam2_interfaces::msg::Vertex> reachables_right;

      merged_obstacles.insert(merged_obstacles.end(), received_poly.polygon.vertices_obstacles.begin(), received_poly.polygon.vertices_obstacles.end());
      merged_reachables.insert(merged_reachables.end(), received_poly.polygon.vertices_reachable.begin(), received_poly.polygon.vertices_reachable.end());
      //RCLCPP_INFO(this->get_logger(), "Merged obstacles and reachables");
      
      for(auto node: merged_obstacles){
        // compute value of the inequality mx-y>0, evaluated for joint vector direction
        float value = sideOfLine(received_status->current_position.pose.pose.position,robot_odom_.pose.pose.position,node);
        if(value>0)
          obstacles_right.push_back(node);
        else
          obstacles_left.push_back(node);
      }
      //RCLCPP_INFO(this->get_logger(), "Evaluate obstacles");

      for(auto node: merged_reachables){
        // compute value of the inequality mx-y>0, evaluated for joint vector direction
        float value = sideOfLine(received_status->current_position.pose.pose.position,robot_odom_.pose.pose.position,node);
        if(value>0)
          reachables_right.push_back(node);
        else
          reachables_left.push_back(node);
      }
      std::pair<gbeam2_interfaces::msg::Vertex, gbeam2_interfaces::msg::Vertex> obs_min_pair;
      double min_dist = INF;
      for (int i = 0; i < obstacles_right.size(); i++){
        for (int j = i+1; j < obstacles_left.size(); j++){
          double dist_ij = dist(obstacles_right[i],obstacles_left[j]);
              if(dist_ij>0.05 && dist_ij<min_dist){
                min_dist = dist_ij;  
                obs_min_pair = std::make_pair(obstacles_right[i],obstacles_left[j]) ;
                }   
        }
      }
      RCLCPP_INFO(this->get_logger(),"Best frontier obstacles node: RIGHT: id: %ld of rob: %ld, LEFT: id: %ld of rob: %ld",obs_min_pair.first.id,obs_min_pair.first.belong_to,obs_min_pair.second.id,obs_min_pair.second.belong_to);
      

      gbeam2_interfaces::msg::FrontierStamped resulted_frontier;

      resulted_frontier.frontier.vertices_obstacles = obstacles_right;

      resulted_frontier.frontier.vertices_reachable = obstacles_left;

      frontier_pub_->publish(resulted_frontier);

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
