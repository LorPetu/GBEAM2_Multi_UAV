//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread> // Include for threading
#include <future> // Include for promises and futures
#include <mutex>
#include <condition_variable>

#include "gbeam2_interfaces/msg/vertex.hpp"
#include "gbeam2_interfaces/msg/graph_edge.hpp"
#include "gbeam2_interfaces/msg/poly_area.hpp"
#include "gbeam2_interfaces/msg/graph.hpp"
#include "gbeam2_interfaces/msg/free_polygon.hpp"
#include "gbeam2_interfaces/msg/free_polygon_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "gbeam2_interfaces/msg/status.hpp"
#include "gbeam2_interfaces/msg/frontier_stamped.hpp"

#include "gbeam2_interfaces/srv/graph_update.hpp"

using namespace std::chrono_literals;

class GraphMergerNode : public rclcpp::Node
{
public:
    GraphMergerNode() : Node("partial_graph_merger")
    {

    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);        
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = service_cb_group_;

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = service_cb_group_;

    // SUBSCRIBED TOPICS
    graph_subscriber_ = this->create_subscription<gbeam2_interfaces::msg::Graph>(
                "gbeam/reachability_graph", 1, std::bind(&GraphMergerNode::switchCallback, this, std::placeholders::_1),sub_options);

    // PUBLISHING TOPICS
    merged_graph_pub_ = this->create_publisher<gbeam2_interfaces::msg::Graph>(
                "gbeam/merged_graph", 1);
    fake_poly_pub_ = this->create_publisher<gbeam2_interfaces::msg::FreePolygonStamped>(
                "/external_nodes", 1);

    // SERVICE 
    graph_updates_service_ = this->create_service<gbeam2_interfaces::srv::GraphUpdate>(
        "getGraphUpdates",std::bind(&GraphMergerNode::serverCallback,this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,service_cb_group_);
    
    timer_ptr_ = this->create_wall_timer(2s, std::bind(&GraphMergerNode::timerCallback, this),
                                            timer_cb_group_);
    timer_ptr_->cancel();

    // Get namespace
    name_space = this->get_namespace();
    name_space_id = name_space.back()- '0';

    // Initialize parameters
    this->declare_parameter<int>("N_robot",0);
    this->declare_parameter<double>("communication_range",0.0);

    // Get parameters
    N_robot = this->get_parameter("N_robot").get_parameter_value().get<int>();
    wifi_range = this->get_parameter("communication_range").get_parameter_value().get<double>();

    RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF PARTIAL_GRAPH_MERGER: ############# ");
    RCLCPP_INFO(this->get_logger(),"############# (for %s) ############# ",name_space.c_str());
    RCLCPP_INFO(this->get_logger(),"1) Number of robots: %d",N_robot);
    RCLCPP_INFO(this->get_logger(),"1) Number of robots: %f",wifi_range);
    // Initialize vectors with the correct size
    updateBuffer.resize(N_robot);
    graph_updates_CLIENTS.resize(N_robot);
    last_node_indexes.resize(N_robot);
    last_edge_indexes.resize(N_robot);

    RCLCPP_INFO(this->get_logger(),"1) Size of clients: %d",graph_updates_CLIENTS.size());

    // Create a client for each robot
    std::string service_name;
    for (size_t i = 0; i < N_robot; i++)
    {   
        service_name = "/robot"+std::to_string(i)+"/getGraphUpdates";
        if (i!=name_space_id) graph_updates_CLIENTS[i]=this->create_client<gbeam2_interfaces::srv::GraphUpdate>(service_name);    
        RCLCPP_INFO(this->get_logger(),"Initialize client: %s",service_name.c_str());     
    }
     

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    };

private:
    std::string name_space;
    int name_space_id;

    // Parameters variables
    int N_robot;
    double wifi_range;

    rclcpp::CallbackGroup::SharedPtr service_cb_group_;        
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr graph_subscriber_;
    rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr merged_graph_pub_;
    rclcpp::Publisher<gbeam2_interfaces::msg::FreePolygonStamped>::SharedPtr fake_poly_pub_;
    rclcpp::Service<gbeam2_interfaces::srv::GraphUpdate>::SharedPtr  graph_updates_service_;
    std::vector<rclcpp::Client<gbeam2_interfaces::srv::GraphUpdate>::SharedPtr> graph_updates_CLIENTS;

    rclcpp::TimerBase::SharedPtr timer_ptr_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


    std::mutex mutex_;
    std::condition_variable cv_;
    bool data_received_ = false;
    gbeam2_interfaces::srv::GraphUpdate::Response updateResponse;

    std::vector<int> last_node_indexes;
    std::vector<int> last_edge_indexes;
    std::vector<gbeam2_interfaces::msg::Graph> updateBuffer;

    geometry_msgs::msg::TransformStamped getTransform(std::string target_frame,std::string source_frame){

        try {
            
            //RCLCPP_INFO(this->get_logger(), "lookupTransform -------> %s to %s", target_frame.c_str(), source_frame.c_str());
            return tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero); 
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(
                this->get_logger(), "GBEAM:graph_update:lookupTransform: Could not transform %s to %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
          //return;
        }
  }

    void serverCallback(const std::shared_ptr<gbeam2_interfaces::srv::GraphUpdate::Request> request,
                        std::shared_ptr<gbeam2_interfaces::srv::GraphUpdate::Response> response)
    {
        int req_robot_id = request->update_request.robot_id;
        std::unique_lock<std::mutex> lock(mutex_);
        RCLCPP_INFO(this->get_logger(), "Service received");
        //RCLCPP_INFO(this->get_logger(), "I'm receiving %ld nodes from: %d", request->update_request.nodes.size(), request->update_request.robot_id);

        // Prepare and publish the fake polygon
        gbeam2_interfaces::msg::FreePolygonStamped fake_poly;
        fake_poly.robot_id = req_robot_id;
        std::string target_frame = name_space.substr(1, name_space.length()-1) + "/odom";
        fake_poly.header.frame_id = target_frame;
        fake_poly.polygon.vertices_reachable = request->update_request.nodes;

        //RCLCPP_INFO(this->get_logger(), "-->I'm sending %ld nodes from: %d", fake_poly.polygon.vertices_reachable.size(), fake_poly.robot_id);
        
        // Reset flags and prepare for new update
        data_received_ = false;
        updateResponse.success = false;
        
        // Publish the fake polygon
        fake_poly_pub_->publish(fake_poly);

        // Wait for the update to be processed with a timeout
        auto timeout = std::chrono::seconds(5); // Adjust the timeout as needed
        auto status = cv_.wait_for(lock, timeout, [this] { return data_received_; });

        if (status) {
            // Data received within the timeout period
            //RCLCPP_INFO(this->get_logger(),"Update response success: %d (false is:%d)",updateResponse.success, false);
            response->update_response = updateBuffer[req_robot_id];
            last_node_indexes[req_robot_id]=updateBuffer[req_robot_id].nodes.back().id;
            last_edge_indexes[req_robot_id]=updateBuffer[req_robot_id].edges.back().id;
            response->success = updateResponse.success;
            RCLCPP_WARN(this->get_logger(), "Data received");
           
        } else {
            // Timeout occurred
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "Timeout waiting for graph update");
        }
        this->timer_ptr_->reset();
    }

    void switchCallback(const gbeam2_interfaces::msg::Graph::SharedPtr graph)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        for (int i = 0; i < N_robot; i++)
        {
            RCLCPP_INFO(this->get_logger(), "MERGER:: last nodes updated: %d last_edge_updated: %d", last_node_indexes[i], last_edge_indexes[i]);
            if (i!=name_space_id) updateBuffer[graph->last_updater_id]=compareUpdates(graph,last_node_indexes[i],last_edge_indexes[i]);
        }
        
        
        if (graph->last_updater_id != name_space_id) { 
            updateResponse.success = true;
            data_received_ = true;
            cv_.notify_one();
            RCLCPP_INFO(this->get_logger(), "updateResponse processing...");
        } else {
            // This is our own update, publish it
            //RCLCPP_INFO(this->get_logger(), "Publishing on my own topic...");
            merged_graph_pub_->publish(*graph);
        }
    }

    void timerCallback(){
        RCLCPP_INFO(this->get_logger(),"Send a request...");
        timer_ptr_->cancel();
    }

    gbeam2_interfaces::msg::Graph compareUpdates(gbeam2_interfaces::msg::Graph::SharedPtr graph2compare, int last_node_index, int last_edge_index){
        // Retrieve all the new nodes after last_node_index
        gbeam2_interfaces::msg::Graph result;
        result.nodes = std::vector<gbeam2_interfaces::msg::Vertex>(graph2compare->nodes.begin() + last_node_index, graph2compare->nodes.end());
        result.edges = std::vector<gbeam2_interfaces::msg::GraphEdge>(graph2compare->edges.begin() + last_edge_index, graph2compare->edges.end());
        result.adj_matrix=graph2compare->adj_matrix;
        return result;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphMergerNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

