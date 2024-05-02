#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

#include "gbeam2_interfaces/msg/free_polygon.hpp"
#include "gbeam2_interfaces/msg/free_polygon_stamped.hpp"
#include "gbeam2_interfaces/msg/vertex.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "library_fcn.hpp"

class PolyDrawNode : public rclcpp::Node
{
public:
  PolyDrawNode() : Node("poly_draw")
  {
    poly_sub_ = this->create_subscription<gbeam2_interfaces::msg::FreePolygonStamped>(
        "gbeam/free_polytope", 1, std::bind(&PolyDrawNode::polyCallback, this, std::placeholders::_1));

    draw_poly_obs_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
        "gbeam_visualization/poly_obstacles", 1);
    draw_poly_reach_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
        "gbeam_visualization/poly_reachable", 1);
    vertices_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "gbeam_visualization/poly_vert", 1);
    poly_normals_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "gbeam_visualization/poly_normals", 1);
  }

private:

  rclcpp::Subscription<gbeam2_interfaces::msg::FreePolygonStamped>::SharedPtr poly_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr draw_poly_obs_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr draw_poly_reach_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr vertices_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr poly_normals_pub_;


  void polyCallback(const gbeam2_interfaces::msg::FreePolygonStamped::SharedPtr poly_ptr)
  {
      //initialize polygon, for /poly
  geometry_msgs::msg::PolygonStamped polygon_obs;
    polygon_obs.header = poly_ptr->header;
  geometry_msgs::msg::PolygonStamped polygon_reach;
    polygon_reach.header = poly_ptr->header;

  //initialize vertices, for /poly_vert
  sensor_msgs::msg::PointCloud vertices;
    vertices.header = poly_ptr->header;
  sensor_msgs::msg::ChannelFloat32 expGainChan, obstacleChan;
    expGainChan.name = "exp_gain";
    obstacleChan.name = "is_obstacle";

  //initialize normals, for /poly_normals
  visualization_msgs::msg::Marker normals;
    normals.ns = "graph_drawer", normals.id = 1, normals.type = 5, normals.scale.x = 0.01;
    normals.header = poly_ptr->header;
  std_msgs::msg::ColorRGBA normals_color;
    normals_color.r = 1, normals_color.g = 0.8, normals_color.b = 0.5, normals_color.a = 1;

  //attribute data
  int num_vertices = poly_ptr->polygon.vertices_obstacles.size();
  for(int v=0; v<num_vertices; v++)
  {
    //create point corresponding to vertex obstacle
    geometry_msgs::msg::Point32 point_obs;
    point_obs.x = poly_ptr->polygon.vertices_obstacles[v].x;
    point_obs.y = poly_ptr->polygon.vertices_obstacles[v].y;
    point_obs.z = poly_ptr->polygon.vertices_obstacles[v].z;

    //create point corresponding to vertex reachable
    geometry_msgs::msg::Point32 point_reach;
    point_reach.x = poly_ptr->polygon.vertices_reachable[v].x;
    point_reach.y = poly_ptr->polygon.vertices_reachable[v].y;
    point_reach.z = poly_ptr->polygon.vertices_reachable[v].z;

    //add vertex to polygon
    polygon_obs.polygon.points.push_back(point_obs);
    polygon_reach.polygon.points.push_back(point_reach);

    //add vertex to vertices
    vertices.points.push_back(point_obs);
    expGainChan.values.push_back(poly_ptr->polygon.vertices_obstacles[v].gain);
    obstacleChan.values.push_back(poly_ptr->polygon.vertices_obstacles[v].is_obstacle);

    //add normal to normals if the vertex is obstacle
    if(poly_ptr->polygon.vertices_obstacles[v].is_obstacle)
    {
      geometry_msgs::msg::Point w, z;
      w.x = point_obs.x, w.y = point_obs.y, w.z = point_obs.z;
      z.x = point_obs.x, z.y = point_obs.y, z.z = point_obs.z;
      z.x += 0.1 * poly_ptr->polygon.vertices_obstacles[v].obstacle_normal.x;
      z.y += 0.1 * poly_ptr->polygon.vertices_obstacles[v].obstacle_normal.y;
      w.x -= 0.1 * poly_ptr->polygon.vertices_obstacles[v].obstacle_normal.x;
      w.y -= 0.1 * poly_ptr->polygon.vertices_obstacles[v].obstacle_normal.y;
      normals.points.push_back(w);
      normals.points.push_back(z);
      normals.colors.push_back(normals_color);
      normals.colors.push_back(normals_color);
    }
  }

  //push back channels
  vertices.channels.push_back(expGainChan);
  vertices.channels.push_back(obstacleChan);

  //publish on topics
  draw_poly_obs_pub_->publish(polygon_obs);
  draw_poly_reach_pub_->publish(polygon_reach);
  vertices_pub_->publish(vertices);
  poly_normals_pub_->publish(normals);

  }

  
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolyDrawNode>());
  rclcpp::shutdown();
  return 0;
}
