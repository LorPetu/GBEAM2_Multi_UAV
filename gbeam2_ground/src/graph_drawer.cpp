#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "gbeam2_interfaces/msg/vertex.hpp"
#include "gbeam2_interfaces/msg/graph_edge.hpp"
#include "gbeam2_interfaces/msg/poly_area.hpp"
#include "gbeam2_interfaces/msg/reachability_graph.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "library_fcn.hpp"
// #include "polytope_fcn.hpp"

class GraphDrawer : public rclcpp::Node
{
public:
    GraphDrawer() : Node("graph_draw")
    {
        graph_nodes_pub = this->create_publisher<sensor_msgs::msg::PointCloud>("gbeam_visualization/graph_nodes", 1);
        graph_normals_pub = this->create_publisher<visualization_msgs::msg::Marker>("gbeam_visualization/graph_nodes_normals", 1);
        graph_edges_pub = this->create_publisher<visualization_msgs::msg::Marker>("gbeam_visualization/graph_edges", 1);

        graph_sub = this->create_subscription<gbeam2_interfaces::msg::ReachabilityGraph>(
            "gbeam/reachability_graph", 1,
            std::bind(&GraphDrawer::graphCallback, this, std::placeholders::_1));

        this->declare_parameter<float>("scaling", 0.0);

        scaling = this->get_parameter("scaling").get_parameter_value().get<float>();

        RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF GRAPH_DRAW: ############# ");
        RCLCPP_INFO(this->get_logger(),"1) SCALING: %f", scaling);

    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr graph_nodes_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_normals_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_edges_pub;
    rclcpp::Subscription<gbeam2_interfaces::msg::ReachabilityGraph>::SharedPtr graph_sub;

    float scaling;

    void graphCallback(const gbeam2_interfaces::msg::ReachabilityGraph::SharedPtr graph_ptr)
    {
        

        // define colors
        std_msgs::msg::ColorRGBA boundary_color;
        boundary_color.r = 1, boundary_color.g = 0.8, boundary_color.b = 0.5, boundary_color.a = 1;
        std_msgs::msg::ColorRGBA inside_color;
        inside_color.r = 0, inside_color.g = 0.6, inside_color.b = 0.5, inside_color.a = 0.15;
        boundary_color.r = 1, boundary_color.g = 0.8, boundary_color.b = 0.5, boundary_color.a = 1;
        std_msgs::msg::ColorRGBA walkable_color;
        walkable_color.r = 1, walkable_color.g = 0.1, walkable_color.b = 0.8, walkable_color.a = 0.15;
        std_msgs::msg::ColorRGBA normals_color;
        normals_color.r = 0.6, normals_color.g = 0.3, normals_color.b = 0.6, normals_color.a = 1;

        float normal_length = 0.2 * scaling;

        //initialize node_points for /graph_nodes
        sensor_msgs::msg::PointCloud node_points;
        sensor_msgs::msg::ChannelFloat32 expGainChan, obstacleChan, connectedChan;
        expGainChan.name = "exploration_gain";
        obstacleChan.name = "is_obstacle";
        connectedChan.name = "is_compl_connected";

        //initialize normals, for /graph_nodes_normals
        visualization_msgs::msg::Marker nodes_normals;
        nodes_normals.ns = "graph_drawer", nodes_normals.id = 1, nodes_normals.type = 5, nodes_normals.scale.x = 0.005 * scaling;

        //initialize edge_markers for /graph_edges
        visualization_msgs::msg::Marker edges_markers;
        edges_markers.ns = "graph_drawer", edges_markers.id = 1, edges_markers.type = 5, edges_markers.scale.x = 0.005 * scaling;
        // edges_markers.pose   could be initialized, actually not needed, ony gives a warning

        //add nodes and nodes normals
        for (int n = 0; n < graph_ptr->nodes.size(); n++)
        {
            geometry_msgs::msg::Point32 point;
            point.x = graph_ptr->nodes[n].x;
            point.y = graph_ptr->nodes[n].y;
            point.z = graph_ptr->nodes[n].z;
            node_points.points.push_back(point);
            expGainChan.values.push_back(graph_ptr->nodes[n].gain);
            obstacleChan.values.push_back(graph_ptr->nodes[n].is_obstacle);
            connectedChan.values.push_back(graph_ptr->nodes[n].is_completely_connected);

            if (graph_ptr->nodes[n].is_obstacle)
            {
                geometry_msgs::msg::Point w, z;
                w.x = graph_ptr->nodes[n].x, w.y = graph_ptr->nodes[n].y, w.z = graph_ptr->nodes[n].z;
                z.x = graph_ptr->nodes[n].x, z.y = graph_ptr->nodes[n].y, z.z = graph_ptr->nodes[n].z;
                w.x += 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.x;
                w.y += 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.y;
                z.x -= 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.x;
                z.y -= 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.y;
                nodes_normals.points.push_back(w);
                nodes_normals.points.push_back(z);
                nodes_normals.colors.push_back(normals_color);
                nodes_normals.colors.push_back(normals_color);
            }
        }

        //add edges
        for (int e = 0; e < graph_ptr->edges.size(); e++)
        {
            edges_markers.points.push_back(vertex2point(graph_ptr->nodes[graph_ptr->edges[e].v1]));
            edges_markers.points.push_back(vertex2point(graph_ptr->nodes[graph_ptr->edges[e].v2]));
            if (graph_ptr->edges[e].is_boundary)
            {
                edges_markers.colors.push_back(boundary_color);
                edges_markers.colors.push_back(boundary_color);
            }
            else
            {
                if (graph_ptr->edges[e].is_walkable)
                {
                    edges_markers.colors.push_back(walkable_color);
                    edges_markers.colors.push_back(walkable_color);
                }
                else
                {
                    edges_markers.colors.push_back(inside_color);
                    edges_markers.colors.push_back(inside_color);
                }
            }
        }
        node_points.channels.push_back(expGainChan);
        node_points.channels.push_back(obstacleChan);
        node_points.channels.push_back(connectedChan);

        node_points.header.frame_id = "odom";
        edges_markers.header.frame_id = "odom";
        nodes_normals.header.frame_id = "odom";

        graph_nodes_pub->publish(node_points);
        graph_normals_pub->publish(nodes_normals);
        graph_edges_pub->publish(edges_markers);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphDrawer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
