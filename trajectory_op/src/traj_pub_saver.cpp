
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tutorial_interfaces/srv/saver.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;
namespace fs = std::filesystem;


class traj_pub_Saver : public rclcpp::Node
{
  public:
    traj_pub_Saver()
    : Node("traj_publisher_Saver"), count_(0)
    {

      odom_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions options;
      options.callback_group = odom_callback_group_;
      traj_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&traj_pub_Saver::traj_callback,this, _1),options);

      publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&traj_pub_Saver::timer_callback, this));

      service = this->create_service<tutorial_interfaces::srv::Saver>(
    "Saver",
    [this](const std::shared_ptr<tutorial_interfaces::srv::Saver::Request> request,
           std::shared_ptr<tutorial_interfaces::srv::Saver::Response> response) {
        this->server(request, response);
    });
    }

void server(const std::shared_ptr<tutorial_interfaces::srv::Saver::Request> request, std::shared_ptr<tutorial_interfaces::srv::Saver::Response> response)
{
    int64_t duration = request->duration;
    string file_name = request-> file_name;
    visualization_msgs::msg::MarkerArray filtered_marker = filterMarkersWithinDuration(marker_array, duration);
    response->result = true;
    saveMarkerArrayToYAML(filtered_marker,file_name);
}

visualization_msgs::msg::MarkerArray filterMarkersWithinDuration(const visualization_msgs::msg::MarkerArray& marker_array, int64_t duration)
{

  
  int start_count = count_;
  cout<<start_count<<endl;
  this_thread::sleep_for(chrono::seconds(duration));
  visualization_msgs::msg::MarkerArray filtered_array;
  int end_count = count_;
  cout<<end_count<<endl;
  for(int i=start_count;i<end_count;i++){
    filtered_array.markers.push_back(marker_array.markers[i]);
  }
  return filtered_array;
}

void saveMarkerArrayToYAML(const visualization_msgs::msg::MarkerArray& filter_marker, string file_name)
{
    YAML::Emitter emitter;
    emitter << YAML::BeginSeq;
    int marker_count = 0;
    for (const auto &marker : filter_marker.markers)
    {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "id" << YAML::Value << marker_count++;
        emitter << YAML::Key << "type" << YAML::Value << marker.type;
        emitter << YAML::Key << "pose";
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "position";
        emitter << YAML::Value << YAML::Flow << YAML::BeginMap;
        emitter << YAML::Key << "x" << YAML::Value << marker.pose.position.x;
        emitter << YAML::Key << "y" << YAML::Value << marker.pose.position.y;
        emitter << YAML::Key << "z" << YAML::Value << marker.pose.position.z;
        emitter << YAML::EndMap;
        emitter << YAML::Key << "orientation";
        emitter << YAML::Value << YAML::Flow << YAML::BeginMap;
        emitter << YAML::Key << "x" << YAML::Value << marker.pose.orientation.x;
        emitter << YAML::Key << "y" << YAML::Value << marker.pose.orientation.y;
        emitter << YAML::Key << "z" << YAML::Value << marker.pose.orientation.z;
        emitter << YAML::Key << "w" << YAML::Value << marker.pose.orientation.w;
        emitter << YAML::EndMap;
        emitter << YAML::EndMap;
        emitter << YAML::EndMap;
    }
    emitter << YAML::EndSeq;
    string package_share_dir = ament_index_cpp::get_package_share_directory("trajectory_op");
    string trajectories_dir = package_share_dir + "/trajectories/";
    if (!fs::exists(trajectories_dir)) {
        fs::create_directories(trajectories_dir);
    }

    stringstream filename_stream;
    filename_stream << trajectories_dir<<"/"<<file_name << ".yaml";
    std::string filename = filename_stream.str();

    std::ofstream fout(filename);
    fout << emitter.c_str();
    RCLCPP_INFO(this->get_logger(), "Markers saved to file: %s", filename.c_str());
}

private:
    void timer_callback()
    {
        if (!marker_array.markers.empty()) {
            publisher_->publish(marker);
        }
    }

    void traj_callback(const nav_msgs::msg::Odometry &msg)
    { 
        geometry_msgs::msg::Pose pose = msg.pose.pose;
        if (marker_array.markers.empty() || calculateDistance(marker_array.markers.back().pose.position, pose.position) > 0.2)
        {
            marker.header = msg.header;
            marker.id = count_++;
            marker.ns = "tracker";
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = pose;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker_array.markers.push_back(marker);
        }
    }

    double calculateDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
    {
        return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr traj_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Service<tutorial_interfaces::srv::Saver>::SharedPtr service;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<traj_pub_Saver>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
