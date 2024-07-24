#include "pure_pursuit/pure_pursuit.h"
#include <unistd.h>

void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Received SIGINT. Killing pure_pursuit process.\n";
        rclcpp::shutdown();
    }
}

int main(int argc, char **argv){
  signal(SIGINT, handleSignal);
  rclcpp::init(argc, argv );
  auto node = std::make_shared<PurePursuit>();
  RCLCPP_INFO(node->get_logger(), "Starting pure pursuit");

  // rclcpp::TimerBase::SharedPtr controllerTimer = nh->create_wall_timer(
  //    50ms, //std::chrono::seconds(1/purePursuit.getControlHz()), <- this give erroe find a better fix TODO
  //    std::bind(&PurePursuit::calculateCarCommands, &purePursuit));
  
  // rclcpp::QoS ctrlcmp_qos(rclcpp::KeepLast(1));
	// ctrlcmp_qos.reliable();
	// ctrlcmp_qos.transient_local();

  RCLCPP_INFO(node->get_logger(), "starting node");

  try
  {
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);
    
    while (true)
    {
      executor.spin_all(50ms);
      sched_yield();
    }

    rclcpp::shutdown();
  }
  catch(const rclcpp::exceptions::InvalidNodeError& e)
  {
    std::cerr << e.what() << '\n';
  }

  return 0;
}
