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
#include "gbeam2_interfaces/msg/frontier_stamped_array.hpp"
#include "library_fcn.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp> // For easier point cloud population

#define INF 100000

class Ellipse {
  private:
      gbeam2_interfaces::msg::Vertex focus1;
      gbeam2_interfaces::msg::Vertex focus2;
      double a;  // Semi-major axis
      double b; // Semi-minor axis

      gbeam2_interfaces::msg::Vertex pointToVertex(const geometry_msgs::msg::Point& point) const {
        gbeam2_interfaces::msg::Vertex vert;
        vert.x = point.x;
        vert.y = point.y;
        vert.z = point.z;
        // Initialize other Vertex fields with default values if needed
        return vert;
      }
      void calculateSemiMajorAxis() {
          double c = dist(focus1, focus2) / 2.0;  // Half the distance between foci 
          a = std::sqrt(b*b + c*c);
      }

  public:
      // Constructor
      Ellipse(const gbeam2_interfaces::msg::Vertex& f1, const gbeam2_interfaces::msg::Vertex& f2, double semi_minor_axis)
          : focus1(f1), focus2(f2), b(semi_minor_axis) {
            calculateSemiMajorAxis();
          }

           // Constructor with Point objects
      Ellipse(const geometry_msgs::msg::Point& f1, const geometry_msgs::msg::Point& f2, double semi_minor_axis)
        : focus1(pointToVertex(f1)), focus2(pointToVertex(f2)), b(semi_minor_axis) {
          calculateSemiMajorAxis();
        }

      // Check if a Vertex is inside the ellipse
      bool isInside(const gbeam2_interfaces::msg::Vertex& vert) const {
          double d1 = dist(vert, focus1);
          double d2 = dist(vert, focus2);
          return (d1 + d2) <= 2 * a;
      }

      // Check if a point is inside the ellipse
      bool isInside(const geometry_msgs::msg::Point point) const {
          gbeam2_interfaces::msg::Vertex vert;
          vert.x = point.x;
          vert.y = point.y;
          //vert.z = z;
          return isInside(vert);
      }

      // Getters
      const gbeam2_interfaces::msg::Vertex& getFocus1() const { return focus1; }
      const gbeam2_interfaces::msg::Vertex& getFocus2() const { return focus2; }
      double getSemiMajorAxis() const { return a; }
      double getSemiMinorAxis() const { return b; }

      // Setters
      void setFocus1(const gbeam2_interfaces::msg::Vertex& f1) { focus1 = f1; }
      void setFocus2(const gbeam2_interfaces::msg::Vertex& f2) { focus2 = f2; }
      void setSemiMinorAxis(double semi_minor_axis) { b = semi_minor_axis; }
  };



class CooperationNode : public rclcpp::Node
{
public:
  CooperationNode() : Node("coop_manager"){

    //SUBSCRIBED TOPICS
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            name_space+ "odom", 1, std::bind(&CooperationNode::odomCallback, this, std::placeholders::_1));

    merged_graph_sub_= this->create_subscription<gbeam2_interfaces::msg::Graph>(
      "gbeam/merged_graph",1,std::bind(&CooperationNode::mergedGraphCallback,this,std::placeholders::_1));

    // Get namespace
    name_space = this->get_namespace();
    name_space_id = name_space.back()- '0';
    

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

    frontier_pub_ = this->create_publisher<gbeam2_interfaces::msg::FrontierStampedArray>(
      "frontier",1);

     start_frontiers_service_ = this->create_service<std_srvs::srv::SetBool>(
        "start_frontier",std::bind(&CooperationNode::startFrontier,this, std::placeholders::_1, std::placeholders::_2));

      point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "merged_obstacles",1);

     // Initialize parameters
    this->declare_parameter<int>("N_robot",0);
    this->declare_parameter<double>("communication_range",0.0);
    this->declare_parameter<double>("elipse_scaling_obs",0.0);
    this->declare_parameter<double>("elipse_scaling_robot",0.0);

    // Get parameters
    N_robot = this->get_parameter("N_robot").get_parameter_value().get<int>();
    wifi_range = this->get_parameter("communication_range").get_parameter_value().get<double>();
    elipse_scaling_obs = this->get_parameter("elipse_scaling_obs").get_parameter_value().get<double>();
    elipse_scaling_robot = this->get_parameter("elipse_scaling_robot").get_parameter_value().get<double>();

    RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF COOPERATION: ############# ");
    RCLCPP_INFO(this->get_logger(),"############# (for %s) ############# ",name_space.c_str());
    RCLCPP_INFO(this->get_logger(),"1) Number of robots: %d",N_robot);
    RCLCPP_INFO(this->get_logger(),"2) Communication range: %f",wifi_range);
    RCLCPP_INFO(this->get_logger(),"3) Elipse scaling for frontier obstacle node: %f",elipse_scaling_obs);
    RCLCPP_INFO(this->get_logger(),"4) Elipse scaling between robots: %f",elipse_scaling_robot);

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
  double elipse_scaling_obs;
  double elipse_scaling_robot;

  std::vector<std::shared_ptr<gbeam2_interfaces::msg::Graph>> stored_Graph;
  int last_updated_node;
  int last_updated_edge;
  bool start_frontier = false;

  //Frontiers variables
  int N_my_frontiers = 0;
  std::pair<gbeam2_interfaces::msg::Vertex, gbeam2_interfaces::msg::Vertex> obs_min_pair;
  gbeam2_interfaces::msg::FrontierStampedArray res_frontier_array;
  bool has_bridge = false;
  double dist_ij;
  std::vector<gbeam2_interfaces::msg::Vertex> candidates_reach_nodes;
  std::vector<gbeam2_interfaces::msg::Vertex> merged_obstacles;
  std::vector<gbeam2_interfaces::msg::Vertex> obstacles_left;
  std::vector<gbeam2_interfaces::msg::Vertex> obstacles_right;
  std::vector<gbeam2_interfaces::msg::Vertex> merged_reachables;
  std::vector<gbeam2_interfaces::msg::Vertex> inside_reachables;
  std::vector<gbeam2_interfaces::msg::Vertex> reachables_left;
  std::vector<gbeam2_interfaces::msg::Vertex> reachables_right;
  std::vector<gbeam2_interfaces::msg::Vertex> obstacles_to_evaluate;


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
  rclcpp::Publisher<gbeam2_interfaces::msg::FrontierStampedArray>::SharedPtr frontier_pub_;
  rclcpp::Subscription<gbeam2_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr assigned_graph_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  start_frontiers_service_;

  double deg_90 = M_PI / 2.0;
  double deg_30 = M_PI / 6.0;


  void startFrontier(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
      start_frontier=request->data;     
  }

  bool checkIntersection(gbeam2_interfaces::msg::Vertex p1,gbeam2_interfaces::msg::Vertex p2,gbeam2_interfaces::msg::Vertex p3,gbeam2_interfaces::msg::Vertex p4){
      // Check it there's an intersection between two segment: 
      // p1 - rec_pos: the segment between the 2 robot
      // p3   - p4:    the segment between the two obstacles boundary vertex.  

      double t = ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x)==0) ? -1 : 
                  ((p1.x - p3.x)*(p3.y - p4.y) - (p1.y - p3.y)*(p3.x - p4.x))
                / ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x));

      double u = ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x)==0) ? -1 : 
                -((p1.x - p2.x)*(p1.y - p3.y) - (p1.y-p2.y)*(p1.x - p3.x))
                / ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x));

      return (t>=0.0 && t<=1.0 && u>=0.0 && u<=1.0) ? true : false;

  }

  bool checkIntersection(gbeam2_interfaces::msg::Vertex p1,geometry_msgs::msg::Point p2,gbeam2_interfaces::msg::Vertex p3,gbeam2_interfaces::msg::Vertex p4){
      // Check it there's an intersection between two segment: 
      // p1 - rec_pos: the segment between the 2 robot
      // p3   - p4:    the segment between the two obstacles boundary vertex.  

      double t = ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x)==0) ? -1 : 
                  ((p1.x - p3.x)*(p3.y - p4.y) - (p1.y - p3.y)*(p3.x - p4.x))
                / ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x));

      double u = ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x)==0) ? -1 : 
                -((p1.x - p2.x)*(p1.y - p3.y) - (p1.y-p2.y)*(p1.x - p3.x))
                / ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x));

      return (t>=0.0 && t<=1.0 && u>=0.0 && u<=1.0) ? true : false;

  }

  bool checkIntersection(geometry_msgs::msg::Point p1,geometry_msgs::msg::Point p2,gbeam2_interfaces::msg::Vertex p3,gbeam2_interfaces::msg::Vertex p4){
    // Check it there's an intersection between two segment: 
    // p1 - rec_pos: the segment between the 2 robot
    // p3   - p4:    the segment between the two obstacles boundary vertex.  

    double t = ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x)==0) ? -1 : 
                ((p1.x - p3.x)*(p3.y - p4.y) - (p1.y - p3.y)*(p3.x - p4.x))
              / ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x));

    double u = ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x)==0) ? -1 : 
              -((p1.x - p2.x)*(p1.y - p3.y) - (p1.y-p2.y)*(p1.x - p3.x))
              / ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x));

    return (t>=0.0 && t<=1.0 && u>=0.0 && u<=1.0) ? true : false;

  }

  std::pair<double, bool> sideOfLine(geometry_msgs::msg::Point lineStart, geometry_msgs::msg::Point lineEnd, gbeam2_interfaces::msg::Vertex point) {
      // 1. Compute the cross product to determine which side of the main line the point is on.
      double cross_product = (lineEnd.x - lineStart.x) * (point.y - lineStart.y) - 
                            (lineEnd.y - lineStart.y) * (point.x - lineStart.x);

      // The value will be positive if the point is on one side, negative if on the other.
      double value = cross_product;

      // 2. Check if the point is within the perpendicular bounds defined by the start and end points.
      // This can be done by projecting the point onto the main line, and seeing if the projected point lies between the start and end.

      // Vector from lineStart to lineEnd
      double dx = lineEnd.x - lineStart.x;
      double dy = lineEnd.y - lineStart.y;

      // Vector from lineStart to the point
      double px = point.x - lineStart.x;
      double py = point.y - lineStart.y;

      // Project the point onto the line (calculate the projection scalar)
      double dot_product = px * dx + py * dy;
      double line_length_squared = dx * dx + dy * dy;
      double projection = dot_product / line_length_squared;

      // Check if the projection is between 0 and 1 (i.e., the point is between lineStart and lineEnd in the direction of the line)
      bool is_inside = (projection >= 0 && projection <= 1);

      return std::make_pair(value, is_inside);
  }

  std::pair<double, bool> sideOfLine(gbeam2_interfaces::msg::Vertex lineStart, gbeam2_interfaces::msg::Vertex lineEnd, gbeam2_interfaces::msg::Vertex point) {
    // 1. Compute the cross product to determine which side of the main line the point is on.
    double cross_product = (lineEnd.x - lineStart.x) * (point.y - lineStart.y) - 
                           (lineEnd.y - lineStart.y) * (point.x - lineStart.x);

    // The value will be positive if the point is on one side, negative if on the other.
    double value = cross_product;

    // 2. Check if the point is within the perpendicular bounds defined by the start and end points.
    // This can be done by projecting the point onto the main line, and seeing if the projected point lies between the start and end.

    // Vector from lineStart to lineEnd
    double dx = lineEnd.x - lineStart.x;
    double dy = lineEnd.y - lineStart.y;

    // Vector from lineStart to the point
    double px = point.x - lineStart.x;
    double py = point.y - lineStart.y;

    // Project the point onto the line (calculate the projection scalar)
    double dot_product = px * dx + py * dy;
    double line_length_squared = dx * dx + dy * dy;
    double projection = dot_product / line_length_squared;

    // Check if the projection is between 0 and 1 (i.e., the point is between lineStart and lineEnd in the direction of the line)
    bool is_inside = (projection >= 0 && projection <= 1);

    return std::make_pair(value, is_inside);
}


  std::pair<double, bool> sideOfLine(gbeam2_interfaces::msg::Vertex lineStart,  geometry_msgs::msg::Point lineEnd, gbeam2_interfaces::msg::Vertex point) {
    // 1. Compute the cross product to determine which side of the main line the point is on.
    double cross_product = (lineEnd.x - lineStart.x) * (point.y - lineStart.y) - 
                           (lineEnd.y - lineStart.y) * (point.x - lineStart.x);

    // The value will be positive if the point is on one side, negative if on the other.
    double value = cross_product;

    // 2. Check if the point is within the perpendicular bounds defined by the start and end points.
    // This can be done by projecting the point onto the main line, and seeing if the projected point lies between the start and end.

    // Vector from lineStart to lineEnd
    double dx = lineEnd.x - lineStart.x;
    double dy = lineEnd.y - lineStart.y;

    // Vector from lineStart to the point
    double px = point.x - lineStart.x;
    double py = point.y - lineStart.y;

    // Project the point onto the line (calculate the projection scalar)
    double dot_product = px * dx + py * dy;
    double line_length_squared = dx * dx + dy * dy;
    double projection = dot_product / line_length_squared;

    // Check if the projection is between 0 and 1 (i.e., the point is between lineStart and lineEnd in the direction of the line)
    bool is_inside = (projection >= 0 && projection <= 1);

    return std::make_pair(value, is_inside);
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
  
  void updateMergedNodes(const int robot_id){
    // this function updates the global variable merged_obstacles and merged_reachables with last updated nodes merged with the robot of robot_id
      
      gbeam2_interfaces::msg::FreePolygonStamped received_poly = get_obstacles_and_reachable_nodes(stored_Graph[robot_id]);
      gbeam2_interfaces::msg::FreePolygonStamped my_poly = get_obstacles_and_reachable_nodes(stored_Graph[name_space_id]);

      merged_obstacles = my_poly.polygon.vertices_obstacles;
      merged_reachables = my_poly.polygon.vertices_reachable;


      merged_obstacles.insert(merged_obstacles.end(), received_poly.polygon.vertices_obstacles.begin(), received_poly.polygon.vertices_obstacles.end());
      merged_reachables.insert(merged_reachables.end(), received_poly.polygon.vertices_reachable.begin(), received_poly.polygon.vertices_reachable.end());

  }

  // ###############################################################################
  // ################### FREE FRONTIERS COMPUTATION ################################
  // ###############################################################################


  void computeFreeFrontiers(const geometry_msgs::msg::Point my_pos,const geometry_msgs::msg::Point rec_pos,const gbeam2_interfaces::msg::FrontierStamped SHARED_frontier){
      // compute free frontiers around the shared frontier given as input and position of the robot

      RCLCPP_INFO(this->get_logger(),"Computing FREE frontiers for a  %s ..." , (SHARED_frontier.belong_to != name_space_id)? "received frontier" : "new frontier of mine");
      bool CCW = true;

      gbeam2_interfaces::msg::Vertex start_node = SHARED_frontier.frontier.vertices_obstacles[0];
      gbeam2_interfaces::msg::Vertex end_node = SHARED_frontier.frontier.vertices_obstacles[1];

      gbeam2_interfaces::msg::Vertex dummy_vert;
      tf2::Quaternion rotation;
      tf2::Vector3 v_start_temp;
      tf2::Vector3 v_start(start_node.x - my_pos.x, start_node.y - my_pos.y, 0);
      tf2::Vector3 v_end(end_node.x - my_pos.x, end_node.y - my_pos.y, 0);
      tf2::Vector3 z_axiz(0.0,0.0,1.0);
      double span = wifi_range;//dist(obs_min_pair.first,my_pos); 

      gbeam2_interfaces::msg::FrontierStamped resulted_FREE_frontier;
      std::pair<gbeam2_interfaces::msg::Vertex, gbeam2_interfaces::msg::Vertex> FREE_obs_min_pair;

      gbeam2_interfaces::msg::Vertex last_node = start_node;
      
      double FREE_min_dist = INF;
      double angle_tot_rotation = 0;
      int count=0;
      int count2=0;

      RCLCPP_INFO(this->get_logger(), "START:: odom: (%f, %f)",my_pos.x,my_pos.y);
      RCLCPP_INFO(this->get_logger(), "START:: v_start: (%f, %f), v_end: (%f, %f)", v_start.x(), v_start.y(), v_end.x(), v_end.y());

      // Rotate the vector by a small angle (e.g., 5 degrees)
      double angle = 10.0 * M_PI / 180.0; // 5 degrees in radians
      
      rotation.setRPY(0, 0, angle); // Rotate around Z-axis
      v_start = tf2::quatRotate(rotation, v_start);
      angle_tot_rotation+= angle;
      // Normalize the vector
      //v_start.normalize();


      // Calculate the position of the dummy vertex
      dummy_vert.x = my_pos.x + span * v_start.x();
      dummy_vert.y = my_pos.y + span * v_start.y();
      if(checkIntersection(dummy_vert, my_pos,start_node,end_node)){

        // Rotate the vector by a small angle (e.g., 5 degrees)
    
        rotation.setRPY(0, 0, -2*angle); // Rotate around Z-axis
        v_start = tf2::quatRotate(rotation, v_start);
        angle_tot_rotation+= angle;

        // Normalize the vector
        //v_start.normalize();


        // Calculate the position of the dummy vertex
        dummy_vert.x = my_pos.x + span * v_start.x();
        dummy_vert.y = my_pos.y + span * v_start.y();
        CCW=false;
        RCLCPP_INFO(this->get_logger(),"Clockwise Cycle");
      };

        

        while(cos(v_start.angle(v_end))<0.8 && count<5){
          RCLCPP_INFO(this->get_logger(),"## COSINE:: %f:: ANGLE: (%f)",cos(v_start.angle(v_end)),v_start.angle(v_end)*180/M_PI);
          RCLCPP_INFO(this->get_logger(),"## RESTART:: %d:: dummy_vert: (%f, %f)",count,dummy_vert.x,dummy_vert.y);
          obstacles_left.clear();
          obstacles_right.clear();
          reachables_left.clear();
          reachables_right.clear(); 
          FREE_min_dist = INF;
          resulted_FREE_frontier.frontier.vertices_obstacles.clear();


          while (FREE_min_dist==INF && count<10)
          {
            for(auto& node: merged_obstacles){
            // compute value of the inequality mx-y>0, evaluated for joint vector direction
            auto [value, is_inside] = sideOfLine(dummy_vert,my_pos,node);
              if(is_inside){
                if(value>0){
                  obstacles_right.push_back(node);
                  if(node.id == last_node.id && node.belong_to==node.belong_to) obstacles_to_evaluate = obstacles_left;
                }            
                else{
                  obstacles_left.push_back(node);
                  if(node.id == last_node.id && node.belong_to==node.belong_to) obstacles_to_evaluate = obstacles_right;
                } 
                if (node.id == FREE_obs_min_pair.second.id && node.belong_to == FREE_obs_min_pair.second.belong_to)
                {
                  if(count>0) RCLCPP_INFO(this->get_logger(),"## CONDITION:: end_node is inside!");
                }
                
              } 
            }
            //RCLCPP_INFO(this->get_logger(),"## CONDITION:: %d:: (last_node_value>0): %s",count,(last_node_value>0) ? "LEFT" : "RIGHT");
            RCLCPP_INFO(this->get_logger(),"## CONDITION:: %d:: obstacles_left: %d  obstacles_right: %d",count,obstacles_left.size(), obstacles_right.size());
            for (int i = 0; i < obstacles_to_evaluate.size(); i++){
              dist_ij = dist(last_node,obstacles_to_evaluate[i]);
                if(dist_ij>0.3 && dist_ij<FREE_min_dist){
                  candidates_reach_nodes.clear();
                  for(auto& reach_node : merged_reachables){

                    //auto [value, is_inside_line] = sideOfLine(last_node,obstacles_to_evaluate[i],reach_node);
                    bool is_insideEllipse = Ellipse(last_node,obstacles_to_evaluate[i],elipse_scaling_obs*dist_ij).isInside(reach_node);
                    if(is_insideEllipse){ //
                      candidates_reach_nodes.push_back(reach_node);
                    }  


                    if(candidates_reach_nodes.size()>0){
                      FREE_min_dist = dist_ij;
                      resulted_FREE_frontier.frontier.vertices_reachable = candidates_reach_nodes;  
                      FREE_obs_min_pair = std::make_pair(last_node,obstacles_to_evaluate[i]) ;
                    }
                  
                  } 
                }            
              }
              // Rotate the vector by a small angle (e.g., 5 degrees)
              rotation.setRPY(0, 0, (CCW) ? angle: -angle); // Rotate around Z-axis
              v_start = tf2::quatRotate(rotation, v_start);
              angle_tot_rotation+= angle;
              
              // Normalize the updated vector
              //v_start.normalize();

              // Calculate the position of the dummy vertex
              dummy_vert.x = my_pos.x + span * v_start.x();
              dummy_vert.y = my_pos.y + span * v_start.y();

              count2++;
          }
          


          RCLCPP_INFO(this->get_logger(),"## FREE FRONTIER:: %d:: FREE_min_dist %f",count , FREE_min_dist);
          RCLCPP_INFO(this->get_logger(),"## FREE FRONTIER:: %d:: pair::first (%f, %f), pair::second (%f, %f)",count,
                                                                  FREE_obs_min_pair.first.x,FREE_obs_min_pair.first.y,
                                                                  FREE_obs_min_pair.second.x, FREE_obs_min_pair.second.y );
          last_node = FREE_obs_min_pair.second;

          // Update v_start based on the new FREE_obs_min_pair.second
          v_start_temp = tf2::Vector3(last_node.x - my_pos.x,last_node.y - my_pos.y,0);
          angle_tot_rotation+=v_start_temp.angle(v_start);
          v_start = v_start_temp;

          if(cos(v_start.angle(v_end))>=0.8){
            RCLCPP_INFO(this->get_logger(),"Vector equal!");
            FREE_obs_min_pair.second = end_node;
          }else{
            // Rotate the vector by a small angle (e.g., 5 degrees)
            rotation.setRPY(0, 0, (CCW) ? angle: -angle); // Rotate around Z-axis
            v_start = tf2::quatRotate(rotation, v_start);
            angle_tot_rotation+= angle;
            // Normalize the updated vector
            //v_start.normalize();

            // Calculate the position of the dummy vertex
            dummy_vert.x = my_pos.x + span * v_start.x();
            dummy_vert.y = my_pos.y + span * v_start.y();
          }
          RCLCPP_INFO(this->get_logger(), "END:: v_start: (%f, %f), v_end: (%f, %f)", v_start.x(), v_start.y(), v_end.x(), v_end.y());

          resulted_FREE_frontier.type = 1; // 0 is SHARED, 1 FREE
          resulted_FREE_frontier.id = N_my_frontiers; N_my_frontiers++; 
          resulted_FREE_frontier.shared_with = SHARED_frontier.shared_with; 
          resulted_FREE_frontier.belong_to = name_space_id;
          resulted_FREE_frontier.is_assigned = false;
          resulted_FREE_frontier.is_explored = false;
          resulted_FREE_frontier.frontier.vertices_obstacles.push_back(FREE_obs_min_pair.first);
          resulted_FREE_frontier.frontier.vertices_obstacles.push_back(FREE_obs_min_pair.second);
          last_status[name_space_id].frontiers.push_back(resulted_FREE_frontier);
          res_frontier_array.frontiers.push_back(resulted_FREE_frontier);
    

          count++;
          RCLCPP_INFO(this->get_logger(),"Tot angle of rotation: %f",angle_tot_rotation*180/M_PI);
        }
  }

  // ###############################################################################
  // ################### SHARED FRONTIERS COMPUTATION ##############################
  // ###############################################################################

  gbeam2_interfaces::msg::FrontierStamped computeSharedFrontier(const geometry_msgs::msg::Point my_pos,const geometry_msgs::msg::Point rec_pos){
        std::pair<gbeam2_interfaces::msg::Vertex, gbeam2_interfaces::msg::Vertex> obs_min_pair;
        RCLCPP_INFO(this->get_logger(),"Computing a new SHARED frontier...");
        gbeam2_interfaces::msg::FrontierStamped resulted_frontier;
        double min_dist = INF;

        for(auto& node: merged_obstacles){
          // compute value from the cross product evaluated for joint vector direction
          auto [value, is_inside] = sideOfLine(rec_pos,my_pos,node);
          if(is_inside){
            if(value>0) obstacles_right.push_back(node);
            else obstacles_left.push_back(node);
          } 
        }

        for(auto& node: merged_reachables){
          // compute value from the cross product evaluated for joint vector direction
          auto [value, is_inside] = sideOfLine(rec_pos,my_pos,node);
          if(is_inside){
            inside_reachables.push_back(node);
            if(value>0) reachables_right.push_back(node);
            else reachables_left.push_back(node);
          } 
        }


        for (int i = 0; i < obstacles_right.size(); i++){
          for (int j = 0; j < obstacles_left.size(); j++){
            dist_ij = dist(obstacles_right[i],obstacles_left[j]);
                if(dist_ij>0.3 && dist_ij<min_dist){
                  candidates_reach_nodes.clear();
                  has_bridge = false;
                    for(auto& reach_node : inside_reachables){
                        int N = stored_Graph[reach_node.belong_to]->adj_matrix.size;
                        auto start_alloc = stored_Graph[reach_node.belong_to]->adj_matrix.data.begin();
                        auto edges_ids = std::vector<int>(start_alloc + reach_node.id * N, start_alloc + (reach_node.id + 1) * N);
                          for(int sel_id:edges_ids){
                            // TODO: make better condition, this doesn't work
                            if(sel_id!=-1){ // && reach_node.belong_to!=name_space_id
                              gbeam2_interfaces::msg::GraphEdge sel_edge = stored_Graph[reach_node.belong_to]->edges[sel_id]; 
                              gbeam2_interfaces::msg::Vertex v1 = stored_Graph[reach_node.belong_to]->nodes[sel_edge.v1];
                              gbeam2_interfaces::msg::Vertex v2 = stored_Graph[reach_node.belong_to]->nodes[sel_edge.v2];

                              if(checkIntersection(obstacles_right[i],obstacles_left[j],v1,v2)){
                                has_bridge = true;
                                candidates_reach_nodes.push_back(reach_node);
                                break;
                              }
                            }
                        
                          }

                                              
                      
                    }
                    if(has_bridge){
                      min_dist = dist_ij;
                      resulted_frontier.frontier.vertices_reachable = candidates_reach_nodes;  
                      obs_min_pair = std::make_pair(obstacles_right[i],obstacles_left[j]) ;
                    }
                  
                  }   
          }
        }


          RCLCPP_INFO(this->get_logger(),"## COMPUTED SHARED FRONTIER:: pair::first (%f, %f), pair::second (%f, %f)",
                                                                  obs_min_pair.first.x,obs_min_pair.first.y,
                                                                  obs_min_pair.second.x, obs_min_pair.second.y );

      
     
      if(min_dist!=INF){
        resulted_frontier.id = N_my_frontiers;           N_my_frontiers++;
        resulted_frontier.belong_to = name_space_id;
        resulted_frontier.header.stamp = this->get_clock()->now();
        resulted_frontier.is_assigned = false;
        resulted_frontier.is_explored = false;
        // ADD the SHARED frontier
        resulted_frontier.type = 0; // 0 is SHARED, 1 FREE
      
        resulted_frontier.frontier.vertices_obstacles.push_back(obs_min_pair.first);
        resulted_frontier.frontier.vertices_obstacles.push_back(obs_min_pair.second);
        last_status[name_space_id].frontiers.push_back(resulted_frontier);

        return resulted_frontier;  
      }
      else{
        RCLCPP_INFO(this->get_logger(),"## SHARED FRONTIER:: No feasible frontier found!");
      }
  }

  void statusCallback(const gbeam2_interfaces::msg::Status::SharedPtr received_status){
    last_status[received_status->robot_id]=*received_status;
    if(!start_frontier) return;

    res_frontier_array.frontiers.clear(); 

    auto my_pos = robot_odom_.pose.pose.position; // Odom position of the robot itself
    auto rec_pos = received_status->current_position.pose.pose.position; // Received position of the other robot

    if(stored_Graph[name_space_id]->nodes.size()==0) return;
    

    if(received_status->connection_status[name_space_id]){

      bool is_present = false;              // A frontier has been found among mine or among other's drone 
      for(auto& frontier:received_status->frontiers){
        if(frontier.belong_to!=name_space_id){ // Need to check only received frontier that are not created by me
          if(checkIntersection(my_pos,rec_pos,frontier.frontier.vertices_obstacles[0],frontier.frontier.vertices_obstacles[1])){ 
            RCLCPP_INFO(this->get_logger(),"INTERSECTION: with frontier id: %d of %d -- TYPE: %s",frontier.id,frontier.belong_to,(frontier.type==1) ? "FREE" : "SHARED");
            // Add this frontier to mine if is not already present
            is_present = false;    
            
            for(auto& my_frontier:last_status[name_space_id].frontiers){
              if(my_frontier.belong_to == frontier.belong_to && my_frontier.id == frontier.id ) {
                is_present=true;
                break; // no need to continue searching
                }
            }

            if(!is_present && frontier.type!=1){ 
              //I need to add only SHARED frontiers
              RCLCPP_INFO(this->get_logger(),"AND I add to mine");
              //add it and compute free frontiers
              updateMergedNodes(received_status->robot_id);
              last_status[name_space_id].frontiers.push_back(frontier);
              computeFreeFrontiers(my_pos,rec_pos,frontier);
            
              
            }
            else{
              return;
            }
          }
        }
      }
      
      // Check among my frontiers
      is_present=false;
      for(auto& frontier:last_status[name_space_id].frontiers){
          // It should be checking only with mine belongings
          if(checkIntersection(my_pos,rec_pos,frontier.frontier.vertices_obstacles[0],frontier.frontier.vertices_obstacles[1]))
          {
            RCLCPP_INFO(this->get_logger(),"INTERSECTION: with MY frontier id: %d of %d -- TYPE: %s",frontier.id,frontier.belong_to,(frontier.type==1) ? "FREE" : "SHARED");
            is_present = true;
            break;
          }
        
      }
      if(!is_present){
        updateMergedNodes(received_status->robot_id);
        auto SHARED_frontier = computeSharedFrontier(my_pos,rec_pos);
        SHARED_frontier.shared_with = received_status->robot_id; 
        computeFreeFrontiers(my_pos,rec_pos,SHARED_frontier);
      } 


      RCLCPP_INFO(this->get_logger(),"N of my frontiers: %d",N_my_frontiers);
      res_frontier_array.frontiers = last_status[name_space_id].frontiers;
      frontier_pub_->publish(res_frontier_array);

      // If a new frontier is computed i need to partion the graph taking into account the new frontier

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
