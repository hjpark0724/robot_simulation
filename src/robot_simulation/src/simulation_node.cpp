#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SimulationNode : public rclcpp::Node
{
public:
  SimulationNode()
  : Node("simulation_node")
  {
    timer_ = create_wall_timer(1s, [this]() {
      RCLCPP_INFO(get_logger(), "Robot simulation is running");
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationNode>());
  rclcpp::shutdown();
  return 0;
}
