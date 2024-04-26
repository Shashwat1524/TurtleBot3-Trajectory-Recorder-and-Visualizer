#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "anscer_interface/srv/trajectory_path.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <fstream>
#include <chrono> 
#include <thread>


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;



class TrajectoryPublisherSaver : public rclcpp::Node
{
public:
    TrajectoryPublisherSaver() : Node("trajectory_publisher_saver")
    {
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&TrajectoryPublisherSaver::odometry_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),"Subscribing to Odometry Data!");

        trajectory_publisher_ = this->create_publisher
        <visualization_msgs::msg::MarkerArray>("trajectory_marker_publisher_test", 10);
        RCLCPP_INFO(this->get_logger(),"Publishing Marker Data!");
    
        trajectory_server_ =this->create_service<anscer_interface::srv::TrajectoryPath>("trajectory_service_test",std::bind(&TrajectoryPublisherSaver::callbacksaver, this ,_1,_2));
        RCLCPP_INFO(this->get_logger(),"Service has Started!");
    }


private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_publisher_;
    rclcpp::Service<anscer_interface::srv::TrajectoryPath>::SharedPtr trajectory_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

    std::vector<geometry_msgs::msg::Pose> trajectory_;
    visualization_msgs::msg::MarkerArray marker_array;

    geometry_msgs::msg::Pose createPose(double x, double y, double z)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        return pose;
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

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        trajectory_.push_back(msg->pose.pose);
        publish_trajectory();
    }

    void callbacksaver(const anscer_interface::srv::TrajectoryPath::Request::SharedPtr request,
                   const anscer_interface::srv::TrajectoryPath::Response::SharedPtr response)
    {
        std::ofstream csv_file(request->file_name + ".csv");

        if (csv_file.is_open())
        {
            csv_file << "PositionX,PositionY,PositionZ,OrientationX,OrientationY,OrientationZ,OrientationW\n";

            auto start_time = std::chrono::steady_clock::now();
            auto end_time = start_time + std::chrono::seconds(request->duration);
            auto sample_duration = std::chrono::milliseconds(10);

            RCLCPP_INFO(this->get_logger(), "Recording Trajectory Data for %ld seconds", request->duration);

            
            while (std::chrono::steady_clock::now() < end_time )
            {
                
                for (const auto &pose : trajectory_)
                {
                    auto current_time = std::chrono::steady_clock::now();
                    if (current_time >= end_time)
                        break;
                    
                    csv_file << pose.position.x << ","
                            << pose.position.y << ","
                            << pose.position.z << ",";

                    
                    csv_file << pose.orientation.x << ","
                            << pose.orientation.y << ","
                            << pose.orientation.z << ","
                            << pose.orientation.w << "\n";

                    std::this_thread::sleep_for(sample_duration);
                }
            }

            csv_file.close();
            RCLCPP_INFO(this->get_logger(), "CSV file Created Successfully with File Name %s.csv", request->file_name.c_str());
            response->success = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create CSV file: %s.csv due to Error", request->file_name.c_str());
            response->success = false;
        }
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPublisherSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

