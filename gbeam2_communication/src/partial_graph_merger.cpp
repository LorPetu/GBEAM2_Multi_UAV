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
  GraphMergerNode() : Node("partial_graph_merger"), response_received(false)
  {

  //SUBSCRIBED TOPICS
  graph_subscriber_ = this->create_subscription<gbeam2_interfaces::msg::Graph>(
            "gbeam/reachability_graph", 1, std::bind(&GraphMergerNode::switchCallback, this, std::placeholders::_1));

  //PUBLISHING TOPICS
  merged_graph_pub_ = this->create_publisher<gbeam2_interfaces::msg::Graph>(
            "gbeam/merged_graph", 1);
  fake_poly_pub_ = this->create_publisher<gbeam2_interfaces::msg::FreePolygonStamped>(
            "gbeam/free_polytope", 1);

  //SERVICE 
  graph_updates_service_ = this->create_service<gbeam2_interfaces::srv::GraphUpdate>(
    "gbeam/getGraphUpdates",std::bind(&GraphMergerNode::serverCallback,this, std::placeholders::_1, std::placeholders::_2));


  //Get namespace
  name_space = this->get_namespace();
  name_space_id = name_space.back()- '0';

  //Initialize parameters
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  };

private:
  std::string name_space;
  int name_space_id;

  rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr graph_subscriber_;
  rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr merged_graph_pub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::FreePolygonStamped>::SharedPtr fake_poly_pub_;
  rclcpp::Service<gbeam2_interfaces::srv::GraphUpdate>::SharedPtr  graph_updates_service_;

  gbeam2_interfaces::srv::GraphUpdate::Response updateResponse;

   std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::mutex response_mutex_;
  bool response_received = false;

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

  void switchCallback(const gbeam2_interfaces::msg::Graph graph){
    //RCLCPP_INFO(this->get_logger(), "MERGER:: namespace: %s namespace_id: %d",name_space.c_str(),name_space_id);
    std::lock_guard<std::mutex> lock(response_mutex_);
    if(graph.last_updater_id!=name_space_id){
        // This means I received the graph with other drones nodes
        // updateResponse processing...
        RCLCPP_INFO(this->get_logger(),"updateResponse processing...");
        updateResponse.success=true;

        response_received=true;
    }
    else{
      // fill buffers Continue to publish and 
      RCLCPP_INFO(this->get_logger(),"Publishing on my own topic...");
      merged_graph_pub_->publish(graph);
    }

  }
    void serverCallback(const std::shared_ptr<gbeam2_interfaces::srv::GraphUpdate::Request> request,
                        std::shared_ptr<gbeam2_interfaces::srv::GraphUpdate::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service received");
        /*//compute polygon in global coordinates
        gbeam2_interfaces::msg::FreePolygon polyGlobal = poly_transform(poly_ptr->polygon, l2g_tf);*/

        gbeam2_interfaces::msg::FreePolygonStamped fake_poly ;
        fake_poly.robot_id = request->update_request.robot_id;
        std::string target_frame = name_space.substr(1, name_space.length()-1) + "/odom";
        fake_poly.header.frame_id = target_frame;
        fake_poly.polygon.vertices_reachable = request->update_request.nodes;

        fake_poly_pub_->publish(fake_poly);

        // Reset the flag before waiting for the new message
        response_received = false;

        // Launch a separate thread to handle the waiting
        std::thread([this, response]() {
            // Wait for a response or timeout
            auto timeout = std::chrono::system_clock::now() + std::chrono::seconds(2);
            while (rclcpp::ok() && std::chrono::system_clock::now() < timeout) {
                {
                    std::lock_guard<std::mutex> lock(response_mutex_);
                    if (response_received) {
                        // Graph message received and processed
                        RCLCPP_INFO(this->get_logger(), "Message has been processed, I can send update with new nodes");
                        // Fill in the response here based on the received graphy
                        
                        
                        return;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Avoid busy-waiting
            }

            RCLCPP_WARN(this->get_logger(), "Timeout waiting for graph message");
            // Respond with a timeout flag or other appropriate response
            response->success = false;

        }).detach(); // Detach the thread so it runs independently of the service call
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphMergerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

