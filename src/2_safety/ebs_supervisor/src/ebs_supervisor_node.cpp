#include <ebs_supervisor/ebs_supervisor.hpp>

void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Received SIGINT. Killing node process.\n";
        rclcpp::shutdown();
    }
}

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  signal(SIGINT, handleSignal);
  /* node initialization */
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto node = std::make_shared<EBSSupervisor>();
    executor.add_node(node);
    
    while (true)
    {
      executor.spin_all(10s);
      /* do not use chrono::timer of ros create public method and use here */
      
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