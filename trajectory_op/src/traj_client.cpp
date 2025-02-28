#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/saver.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using namespace std;
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: duration ");
      return 1;
  }
  cout<<"Enter the duration:"<<endl;

  shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("saver_client");
  rclcpp::Client<tutorial_interfaces::srv::Saver>::SharedPtr client =
    node->create_client<tutorial_interfaces::srv::Saver>("Saver");

  auto request = std::make_shared<tutorial_interfaces::srv::Saver::Request>();
  request->duration = atoll(argv[1]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "File_saved: %ld", result.get()->result);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service saver");
  }

  rclcpp::shutdown();
  return 0;
}