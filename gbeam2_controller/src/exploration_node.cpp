//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>



#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#include "gbeam2_interfaces/msg/vertex.hpp"
#include "gbeam2_interfaces/msg/graph_edge.hpp"
#include "gbeam2_interfaces/msg/poly_area.hpp"
#include "gbeam2_interfaces/msg/reachability_graph.hpp"

#include "gbeam2_interfaces/srv/set_mapping_status.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "library_fcn.hpp"

class ExplorationNode : public rclcpp::Node
{
public:
    ExplorationNode() : Node("graph_expl")
    {   
        name_space = this->get_namespace();
        graph_subscriber_ = this->create_subscription<gbeam2_interfaces::msg::ReachabilityGraph>(
            "gbeam/reachability_graph", 1, std::bind(&ExplorationNode::graphCallback, this, std::placeholders::_1));

        pos_ref_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "gbeam/gbeam_pos_ref", 1);

    
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



      // DECLARATION OF PARAMETERS FROM YAML FILE
        
        this-> declare_parameter<float>("reached_tol",0.0);
        this-> declare_parameter<float>("distance_exp",0.0);
        
        //Exploration limits
        this->declare_parameter<double>("limit_xi",0.0);
        this->declare_parameter<double>("limit_xs",0.0);
        this->declare_parameter<double>("limit_yi",0.0);
        this->declare_parameter<double>("limit_ys",0.0);

        // Get parameter from yaml file
        reached_tol = this->get_parameter("reached_tol").get_parameter_value().get<float>();
        distance_exp = this->get_parameter("distance_exp").get_parameter_value().get<float>();
        limit_xi = this->get_parameter("limit_xi").get_parameter_value().get<double>();
        limit_xs = this->get_parameter("limit_xs").get_parameter_value().get<double>();
        limit_yi = this->get_parameter("limit_yi").get_parameter_value().get<double>();
        limit_ys = this->get_parameter("limit_ys").get_parameter_value().get<double>();

        RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF EXPLORATION NODE: ############# ");
        RCLCPP_INFO(this->get_logger(),"1) REACHED_TOL: %f", reached_tol);
        RCLCPP_INFO(this->get_logger(),"2) DISTANCE_EXP: %f", distance_exp);
        RCLCPP_INFO(this->get_logger(),"3) LIMIT_XI: %f", limit_xi);
        RCLCPP_INFO(this->get_logger(),"4) LIMIT_XS: %f", limit_xs);
        RCLCPP_INFO(this->get_logger(),"5) LIMIT_YI: %f", limit_yi);
        RCLCPP_INFO(this->get_logger(),"6) LIMIT_YS: %f", limit_ys);
        
      //---------------------------------------------
    
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ExplorationNode::explorationCallback, this));
    }

    
    
private:
    double distance_exp;
    double reached_tol;
    double limit_xi, limit_xs, limit_yi, limit_ys;

    int N=0, E=0; 
    int last_target =-1;
    int mapping_z = 5;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string name_space;
    
    gbeam2_interfaces::msg::ReachabilityGraph graph;
    gbeam2_interfaces::msg::Vertex last_target_vertex;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pos_ref_publisher_;
    rclcpp::Subscription<gbeam2_interfaces::msg::ReachabilityGraph>::SharedPtr graph_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;


    void graphCallback(const gbeam2_interfaces::msg::ReachabilityGraph::SharedPtr graph_ptr)
    {
        graph = *graph_ptr;
        N = graph.nodes.size();
        E = graph.nodes.size();
    }

    void computeNewTarget()
    {
        geometry_msgs::msg::PoseStamped pos_ref;

        float max_reward = 0;
        int best_node = 0;
        float dist[N];

        shortestDistances(graph, dist, last_target);

        for(int n=0; n<N; n++)
        {
        if(graph.nodes[n].is_reachable)  // Choose only reachable targets
        {
            float reward = graph.nodes[n].gain / pow(dist[n], distance_exp);
            if(reward > max_reward)
            {
            max_reward = reward;
            best_node = n;
            }
        }
        }

        RCLCPP_INFO(this->get_logger(), "Target node (best): %d", best_node);

        RCLCPP_INFO(this->get_logger(), "Computing path from %d to %d", last_target, best_node);
        std::vector<int> path = dijkstra(graph, last_target, best_node);

        std::string path_str;
        for (int i=0; i<path.size(); i++)
        path_str = path_str + std::to_string(path[i]) + "-";
        RCLCPP_INFO(this->get_logger(), "New path is: %s", path_str.c_str());

        if (path.size() > 1) last_target = path[1];

        gbeam2_interfaces::msg::Vertex vert = applyBoundary(graph.nodes[last_target], limit_xi, limit_xs, limit_yi, limit_ys);
        //RCLCPP_INFO(this->get_logger(),"applyBoundary executed ");
        last_target_vertex = vert;

        pos_ref.pose.position = vertex2point(vert);
        //RCLCPP_INFO(this->get_logger(),"vertex to point executed ");
        pos_ref.pose.position.z = mapping_z;
        RCLCPP_INFO(this->get_logger()," publish pos ref: x: %f y: %f",pos_ref.pose.position.x,pos_ref.pose.position.y);
        pos_ref.header.frame_id = name_space.substr(1, name_space.length()-1) + "/odom";
        pos_ref_publisher_->publish(pos_ref);
        RCLCPP_INFO(this->get_logger(),"PUBLISHED!!!!!!! ");
    }
        
    void explorationCallback(){
        geometry_msgs::msg::TransformStamped l2g_tf;

        std::string target_frame = name_space.substr(1, name_space.length()-1) + "/odom"; //becasue lookupTransform doesn't allow "/" as first character
        std::string source_frame = name_space.substr(1, name_space.length()-1) + "/base_scan";
        try {
            
            //RCLCPP_INFO(this->get_logger(), "lookupTransform -------> %s to %s", target_frame.c_str(), source_frame.c_str());
            l2g_tf = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(
                this->get_logger(), "GBEAM:graph_update:lookupTransform: Could not transform %s to %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return;
        } 

        //create dummy vertex corresponding to robot position
        gbeam2_interfaces::msg::Vertex position;
        position = vert_transform(position, l2g_tf);  // create temporary vertex at robot position
        // check if last_target has been reached
        if(N>0)
        {
            if(last_target < 0){
                //RCLCPP_INFO(this->get_logger(),"last_target initial value: %d",last_target);
                last_target = 0, computeNewTarget();
            }else{
                if(dist(last_target_vertex, position) <= reached_tol){ 
                    //RCLCPP_INFO(this->get_logger(),"last_target reached value: %d",last_target);
                    computeNewTarget();
                }      
            }
        }
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorationNode>());
  rclcpp::shutdown();
  return 0;
}