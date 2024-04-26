#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class TrajectoryNode : public rclcpp::Node
{
public:
    TrajectoryNode() : Node("trajectory_reader_node")
    {
        
        marker_pub_ = this->create_publisher
        <visualization_msgs::msg::MarkerArray>("trajectory_reader_marker", 10);
        std::string file_name_;
        declare_parameter<std::string>("file_name", "s.csv");
        get_parameter("file_name", file_name_);
       
        std::ifstream file(file_name_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory file: %s", file_name_.c_str());
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Opened trajectory file: %s", file_name_.c_str());
        }

        
        std::string line;
        std::getline(file, line); // this skips the heading
        visualization_msgs::msg::MarkerArray marker_array;

        // Parse and publish trajectory data
        int id = 0;
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string token;
            std::vector<double> trajectory_data;

            while (std::getline(iss, token, ','))
            {
                trajectory_data.push_back(std::stod(token));
            }

            // Check if the data has correct number of values
            if (trajectory_data.size() != 7)
            {
                RCLCPP_WARN(this->get_logger(), "Invalid data in trajectory file");
                continue;
            }

     
            geometry_msgs::msg::Pose pose;
            pose.position.x = trajectory_data[0];
            pose.position.y = trajectory_data[1];
            pose.position.z = trajectory_data[2];
            pose.orientation.x = trajectory_data[3];
            pose.orientation.y = trajectory_data[4];
            pose.orientation.z = trajectory_data[5];
            pose.orientation.w = trajectory_data[6];

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
        }

        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Publishing Marker Values!");
    }

private:
    rclcpp::Publisher
    <visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
