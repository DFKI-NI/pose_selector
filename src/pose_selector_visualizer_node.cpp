#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include <rmw/qos_profiles.h>
#include "visualization_msgs/msg/marker_array.hpp"
#include "pose_selector/srv/get_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace std::chrono_literals;

class PoseSelectorVisualizer : public rclcpp::Node
{
public:
    PoseSelectorVisualizer(bool wait_for_pose_selector_srv = true)
    : Node("pose_selector_visualizer_node", rclcpp::NodeOptions().allow_undeclared_parameters(true))
    {
        this->declare_parameter("object_color_rgba", std::vector<double>{0.0, 0.0, 0.0, 0.0});
        this->get_parameter("object_color_rgba", color_);

        this->declare_parameter<std::string>("mesh_config_file_path", "");
        std::string mesh_config_file_path = this->get_parameter("mesh_config_file_path").as_string();

        if (!mesh_config_file_path.empty()) {
            try {
                YAML::Node config = YAML::LoadFile(mesh_config_file_path);
                const YAML::Node& meshes = config["meshes"];
                if (meshes) {
                    for (YAML::const_iterator it = meshes.begin(); it != meshes.end(); ++it) {
                        mesh_urls_[it->first.as<std::string>()] = it->second.as<std::string>();
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "'meshes' section not found in config file: %s", mesh_config_file_path.c_str());
                }
            } catch (const YAML::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load mesh config file %s. Error: %s", mesh_config_file_path.c_str(), e.what());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No config file path provided for mesh URLs.");
        }

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        objects_mesh_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pose_selector_objects", qos);
        pose_selector_get_all_poses_srv_ = this->create_client<pose_selector::srv::GetPoses>("/pose_selector_get_all_service");

        if (wait_for_pose_selector_srv) {
            RCLCPP_INFO(this->get_logger(), "Waiting for pose selector services");
            if(pose_selector_get_all_poses_srv_->wait_for_service(10s)) {
                RCLCPP_INFO(this->get_logger(), "Found pose selector services");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Pose selector service not available after waiting!");
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Pose selector visualizer node started");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PoseSelectorVisualizer::update_object_poses, this));
    }

private:
    visualization_msgs::msg::Marker make_mesh_marker_msg(const std::string& mesh_path, const geometry_msgs::msg::PoseStamped& mesh_pose, const std::vector<double>& mesh_scale, int id, const std::vector<double>& color)
    {
        visualization_msgs::msg::Marker mesh_marker_msg;
        mesh_marker_msg.id = id;
        mesh_marker_msg.ns = "object";
        mesh_marker_msg.header.frame_id = mesh_pose.header.frame_id;
        mesh_marker_msg.pose = mesh_pose.pose;
        mesh_marker_msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        mesh_marker_msg.mesh_use_embedded_materials = true;
        mesh_marker_msg.scale.x = mesh_scale[0];
        mesh_marker_msg.scale.y = mesh_scale[1];
        mesh_marker_msg.scale.z = mesh_scale[2];
        mesh_marker_msg.color.r = color[0];
        mesh_marker_msg.color.g = color[1];
        mesh_marker_msg.color.b = color[2];
        mesh_marker_msg.color.a = color[3];
        mesh_marker_msg.mesh_resource = mesh_path;
        return mesh_marker_msg;
    }

    visualization_msgs::msg::Marker make_obj_marker_msg(const std::string& object_name, const geometry_msgs::msg::PoseStamped& mesh_pose, int id)
    {
        auto it = mesh_urls_.find(object_name);
        if (it != mesh_urls_.end())
        {
            return make_mesh_marker_msg(it->second, mesh_pose, {1.0, 1.0, 1.0}, id, color_);
        }
        return visualization_msgs::msg::Marker();
    }

    void update_object_poses()
    {
        auto request = std::make_shared<pose_selector::srv::GetPoses::Request>();
        auto result_future = pose_selector_get_all_poses_srv_->async_send_request(request, std::bind(&PoseSelectorVisualizer::process_poses, this, std::placeholders::_1));
    }

    void process_poses(rclcpp::Client<pose_selector::srv::GetPoses>::SharedFuture future)
    {
        try
        {
            auto resp = future.get();
            visualization_msgs::msg::MarkerArray marker_array_msg;
            int id = 0;
            if (!resp->poses.objects.empty()) {
                for (const auto& obj_pose : resp->poses.objects) {
                    if (mesh_urls_.count(obj_pose.class_id)) {
                        RCLCPP_DEBUG(this->get_logger(), "obj found: %s", obj_pose.class_id.c_str());
                        geometry_msgs::msg::PoseStamped pose_stamped_msg;
                        pose_stamped_msg.header.frame_id = "map";
                        pose_stamped_msg.pose.position = obj_pose.pose.position;
                        pose_stamped_msg.pose.orientation = obj_pose.pose.orientation;
                        marker_array_msg.markers.push_back(make_obj_marker_msg(obj_pose.class_id, pose_stamped_msg, id++));
                    } else {
                        RCLCPP_WARN(this->get_logger(), "URL for %s mesh not provided in configuration file", obj_pose.class_id.c_str());
                    }
                }
            }

            if (!marker_array_msg.markers.empty()) {
                objects_mesh_publisher_->publish(marker_array_msg);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    std::vector<double> color_;
    std::map<std::string, std::string> mesh_urls_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objects_mesh_publisher_;
    rclcpp::Client<pose_selector::srv::GetPoses>::SharedPtr pose_selector_get_all_poses_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSelectorVisualizer>(true));
    rclcpp::shutdown();
    return 0;
}