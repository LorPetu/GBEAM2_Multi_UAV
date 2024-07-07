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
#include "gbeam2_interfaces/msg/reachability_graph.hpp"

#include "gbeam2_interfaces/msg/status.hpp"
#include "gbeam2_interfaces/msg/frontier_stamped.hpp"

class StatusTXNode : public rclcpp::Node
{
public:
  StatusTXNode() : Node("status_TX"){

  //SUBSCRIBED TOPICS

  //PUBLISHING TOPICS

  //Get namespace

  //Initialize parameters

  };

private:
  std::string name_space;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StatusTXNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
