#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "anscer_interface/srv/trajectory_path.hpp"
#include <chrono>
#include <fstream>

using namespace std::chrono_literals;

class TrajectoryPublisherSaver : public rclcpp::Node
{
private:
    static bool recording_started;
    static std::chrono::time_point<std::chrono::steady_clock> start_time;
    static std::chrono::seconds duration;
    static std::ofstream csv_file;

public:
    TrajectoryPublisherSaver() : Node("trajectory_publisher_saver")
    {
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&TrajectoryPublisherSaver::odometry_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to Odometry Data!");

        trajectory_publisher_ = this->create_publisher
        <visualization_msgs::msg::MarkerArray>("trajectory_marker_publisher_test", 10);
        RCLCPP_INFO(this->get_logger(), "Publishing Marker Data!");

        trajectory_server_ = this->create_service
        <anscer_interface::srv::TrajectoryPath>("trajectory_service_test", std::bind(&TrajectoryPublisherSaver::callbacksaver, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Service has Started!");
    }

private:
    rclcpp::Publisher
    <visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_publisher_;
    rclcpp::Service
    <anscer_interface::srv::TrajectoryPath>::SharedPtr trajectory_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

    std::vector
    <geometry_msgs::msg::Pose> trajectory_;

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        trajectory_.push_back(msg->pose.pose);
        publish_trajectory();
        
        if (recording_started && std::chrono::steady_clock::now() >= start_time + duration)
        {
            recording_started = false;
            csv_file.close();
            RCLCPP_INFO(this->get_logger(), "Recording Stopped!");
            return;
        }

        if (!recording_started)
        {
            return;
        }


        if (!csv_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "CSV file is not open");
            return;
        }

        csv_file << msg->pose.pose.position.x << ","
                 << msg->pose.pose.position.y << ","
                 << msg->pose.pose.position.z << ","
                 << msg->pose.pose.orientation.x << ","
                 << msg->pose.pose.orientation.y << ","
                 << msg->pose.pose.orientation.z << ","
                 << msg->pose.pose.orientation.w << "\n";
    }

    void publish_trajectory()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (const auto &pose : trajectory_)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_footprint";
            marker.header.stamp = this->now();
            marker.ns = "trajectory";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = pose;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker_array.markers.push_back(marker);
            // RCLCPP_INFO(this->get_logger(), "Marker Pose: [x=%f, y=%f, z=%f]",
            // marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
        }

        trajectory_publisher_->publish(marker_array);
    }

    void callbacksaver(const anscer_interface::srv::TrajectoryPath::Request::SharedPtr request,
                       const anscer_interface::srv::TrajectoryPath::Response::SharedPtr response)
    {
        csv_file.open(request->file_name + ".csv", std::ios::app);

        if (csv_file.is_open())
        {
            start_time = std::chrono::steady_clock::now();
            duration = std::chrono::seconds(request->duration);
            recording_started = true;

            RCLCPP_INFO(this->get_logger(), "File Name: %s.csv", request->file_name.c_str());
            RCLCPP_INFO(this->get_logger(), "Recording started for %ld seconds",request->duration);
            response->success = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing: %s", request->file_name.c_str());
            response->success = false;
        }
    }
};

bool TrajectoryPublisherSaver::recording_started = false;
std::chrono::time_point<std::chrono::steady_clock> TrajectoryPublisherSaver::start_time;
std::chrono::seconds TrajectoryPublisherSaver::duration;
std::ofstream TrajectoryPublisherSaver::csv_file;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPublisherSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
