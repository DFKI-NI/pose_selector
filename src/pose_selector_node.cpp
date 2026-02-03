/*
 * Copyright (c) 2022, Amos Smith (DFKI GmbH) and contributors
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "pose_selector/srv/pose_query.hpp"
#include "pose_selector/srv/class_query.hpp"
#include "pose_selector/srv/pose_update.hpp"
#include "pose_selector/srv/pose_delete.hpp"
#include "pose_selector/srv/config_save.hpp"
#include "pose_selector/srv/get_poses.hpp"
#include "object_pose_msgs/msg/object_list.hpp"
#include <regex>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>

struct PoseEntry
{
    std::string class_id;
    int instance;
    object_pose_msgs::msg::ObjectPose pose_stamped;

    PoseEntry(){};

    PoseEntry(object_pose_msgs::msg::ObjectPose pose_msg_){
        class_id = pose_msg_.class_id;
        instance = pose_msg_.instance_id;
        pose_stamped = pose_msg_;
    }

};

class PoseSelector : public rclcpp::Node
{
    private:
    bool debug_;
    bool recording_enabled_;
    rclcpp::Service<pose_selector::srv::PoseQuery>::SharedPtr query_service_;
    rclcpp::Service<pose_selector::srv::ClassQuery>::SharedPtr class_query_service_;
    rclcpp::Service<pose_selector::srv::PoseUpdate>::SharedPtr update_service_;
    rclcpp::Service<pose_selector::srv::PoseDelete>::SharedPtr delete_service_;
    rclcpp::Service<pose_selector::srv::ConfigSave>::SharedPtr save_service_;
    rclcpp::Service<pose_selector::srv::GetPoses>::SharedPtr get_all_poses_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_activate_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pose_selector_clear_;
    rclcpp::Subscription<object_pose_msgs::msg::ObjectList>::SharedPtr pose_sub_;
    std::map<std::string,PoseEntry> pose_map_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<std::string> objects_of_interest_;
    std::string global_reference_frame_;

    public:
    PoseSelector() : Node("pose_selector_node")
    {
        this->declare_parameter("debug", false);
        this->get_parameter("debug", debug_);
        this->declare_parameter("global_reference_frame", "world");
        this->get_parameter("global_reference_frame", global_reference_frame_);
        recording_enabled_ = false;

        query_service_ = this->create_service<pose_selector::srv::PoseQuery>("pose_selector_query", std::bind(&PoseSelector::callbackPoseQuery, this, std::placeholders::_1, std::placeholders::_2));
        class_query_service_ = this->create_service<pose_selector::srv::ClassQuery>("pose_selector_class_query", std::bind(&PoseSelector::callbackClassQuery, this, std::placeholders::_1, std::placeholders::_2));
        update_service_ = this->create_service<pose_selector::srv::PoseUpdate>("pose_selector_update", std::bind(&PoseSelector::callbackPoseUpdate, this, std::placeholders::_1, std::placeholders::_2));
        delete_service_ = this->create_service<pose_selector::srv::PoseDelete>("pose_selector_delete", std::bind(&PoseSelector::callbackPoseDelete, this, std::placeholders::_1, std::placeholders::_2));
        save_service_ = this->create_service<pose_selector::srv::ConfigSave>("pose_selector_save", std::bind(&PoseSelector::callbackSave, this, std::placeholders::_1, std::placeholders::_2));
        record_activate_service_ = this->create_service<std_srvs::srv::SetBool>("pose_selector_activate", std::bind(&PoseSelector::activateRecording, this, std::placeholders::_1, std::placeholders::_2));
        get_all_poses_service_ = this->create_service<pose_selector::srv::GetPoses>("pose_selector_get_all", std::bind(&PoseSelector::getAllPoses, this, std::placeholders::_1, std::placeholders::_2));
        pose_selector_clear_ = this->create_service<std_srvs::srv::Trigger>("pose_selector_clear", std::bind(&PoseSelector::clearPoseSelector, this, std::placeholders::_1, std::placeholders::_2));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter("objects_of_interest", std::vector<std::string>{});
        this->get_parameter("objects_of_interest", objects_of_interest_);
    }

    //Service to query for an item ID and to return the pose of the item
    void callbackPoseQuery(const std::shared_ptr<pose_selector::srv::PoseQuery::Request> req, std::shared_ptr<pose_selector::srv::PoseQuery::Response> res)
    {
        std::string item_id = req->class_id + "_" + std::to_string(req->instance_id);

        if(debug_) RCLCPP_INFO_STREAM(this->get_logger(), "Pose query service call: " << item_id);

        std::map<std::string, PoseEntry>::iterator itr = pose_map_.find(item_id);

        if(itr != pose_map_.end())
        {
            res->pose_query_result = itr->second.pose_stamped;
        }else{
            RCLCPP_ERROR_STREAM(this->get_logger(), item_id << " does not exist!");
        }
    }

    //Service to query all items of a certain class and to return the poses and ids of the items
    void callbackClassQuery(const std::shared_ptr<pose_selector::srv::ClassQuery::Request> req, std::shared_ptr<pose_selector::srv::ClassQuery::Response> res)
    {
        std::string class_id = req->class_id;

        if(debug_) RCLCPP_INFO_STREAM(this->get_logger(), "Class query service call: " << class_id);

        std::vector<object_pose_msgs::msg::ObjectPose> pose_result;

        for (const auto& [key, value] : pose_map_)
        {
            if (value.class_id == class_id)
            {
                pose_result.push_back(value.pose_stamped);
            }
        }

        res->poses = pose_result;
    }

    //Service to update one or more poses
    void callbackPoseUpdate(const std::shared_ptr<pose_selector::srv::PoseUpdate::Request> req, std::shared_ptr<pose_selector::srv::PoseUpdate::Response> res)
    {
        (void)res;
        updatePoses(req->poses);

        if(debug_) printPoses();
    }

    void updatePoses(object_pose_msgs::msg::ObjectList object_list)
    {
        std::string reference_tf = object_list.header.frame_id;

        geometry_msgs::msg::TransformStamped camera_to_world_tf;

        try{
            camera_to_world_tf = tf_buffer_->lookupTransform(this->global_reference_frame_, reference_tf, tf2::TimePointZero);
            }
        catch (tf2::TransformException &ex){
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        tf2::Transform camera_transform;
        tf2::fromMsg(camera_to_world_tf.transform, camera_transform);

        for (auto i: object_list.objects)
        {
            if(!objects_of_interest_.empty() && std::find(objects_of_interest_.begin(), objects_of_interest_.end(), i.class_id) == objects_of_interest_.end()){
                if(debug_) RCLCPP_INFO_STREAM(this->get_logger(), "Class: " << i.class_id << " not of interest, ignoring associated pose");
                continue;
            }

            tf2::Transform obj_transform;
            tf2::fromMsg(i.pose, obj_transform);

            obj_transform = camera_transform*obj_transform;
            tf2::Vector3 final_position = obj_transform.getOrigin();
            tf2::Quaternion final_orientation = obj_transform.getRotation();
            
            //Extract and update PoseEntry with correct pose information
            PoseEntry update_entry = PoseEntry(i);
            i.pose.position.x = final_position[0];
            i.pose.position.y = final_position[1];
            i.pose.position.z = final_position[2];

            i.pose.orientation.x = final_orientation.getX();
            i.pose.orientation.y = final_orientation.getY();
            i.pose.orientation.z = final_orientation.getZ();
            i.pose.orientation.w = final_orientation.getW();

            pose_map_.insert_or_assign(i.class_id + "_" + std::to_string(i.instance_id), PoseEntry(i));
        }
    }

    void poseCallback(const object_pose_msgs::msg::ObjectList::SharedPtr msg)
    {
        updatePoses(*msg);
        if(debug_) printPoses();
    }

    //Service to delete an item
    void callbackPoseDelete(const std::shared_ptr<pose_selector::srv::PoseDelete::Request> req, std::shared_ptr<pose_selector::srv::PoseDelete::Response> res)
    {
        (void)res;
        std::string item_id = req->class_id + "_" + std::to_string(req->instance_id);

        if(debug_) RCLCPP_INFO_STREAM(this->get_logger(), "Delete pose service call: " << item_id);

        pose_map_.erase(item_id);

        if(debug_) printPoses();
    }

    //Save parameters to yaml file
    void callbackSave(const std::shared_ptr<pose_selector::srv::ConfigSave::Request> req, std::shared_ptr<pose_selector::srv::ConfigSave::Response> res)
    {
        (void)res;
        YAML::Node node;
        for (const auto& [key, value] : pose_map_)
        {
            node["poses"][key]["rw"] = value.pose_stamped.pose.orientation.w;
            node["poses"][key]["rx"] = value.pose_stamped.pose.orientation.x;
            node["poses"][key]["ry"] = value.pose_stamped.pose.orientation.y;
            node["poses"][key]["rz"] = value.pose_stamped.pose.orientation.z;
            node["poses"][key]["x"] = value.pose_stamped.pose.position.x;
            node["poses"][key]["y"] = value.pose_stamped.pose.position.y;
            node["poses"][key]["z"] = value.pose_stamped.pose.position.z;
            node["poses"][key]["size_x"] = value.pose_stamped.size.x;
            node["poses"][key]["size_y"] = value.pose_stamped.size.y;
            node["poses"][key]["size_z"] = value.pose_stamped.size.z;
            node["poses"][key]["min_x"] = value.pose_stamped.min.x;
            node["poses"][key]["min_y"] = value.pose_stamped.min.y;
            node["poses"][key]["min_z"] = value.pose_stamped.min.z;
            node["poses"][key]["max_x"] = value.pose_stamped.max.x;
            node["poses"][key]["max_y"] = value.pose_stamped.max.y;
            node["poses"][key]["max_z"] = value.pose_stamped.max.z;
        }

        std::string save_dir = ament_index_cpp::get_package_share_directory("pose_selector") + "/config/" + req->file_name + ".yaml";
        std::ofstream fout(save_dir);
        fout << node;
    }

    //Load the poses from a yaml file
    void loadPoses()
    {
        this->declare_parameter<std::string>("config_file", "");
        std::string config_file_path = this->get_parameter("config_file").as_string();

        if (config_file_path.empty())
        {
            if (debug_)
                RCLCPP_WARN_STREAM(this->get_logger(), "config_file parameter is not set. You may ignore this message if no prior poses are needed.");
            return;
        }

        try
        {
            YAML::Node poses_yaml = YAML::LoadFile(config_file_path);
            if (!poses_yaml["poses"])
            {
                if(debug_) RCLCPP_WARN_STREAM(this->get_logger(), "Pose_selector failed to get initial poses from configuration file. 'poses' node not found in " << config_file_path << ". You may ignore this message if no prior poses are needed.");
                return;
            }

            const YAML::Node& poses_list = poses_yaml["poses"];
            for (const auto& pose_it : poses_list)
            {
                std::string name = pose_it.first.as<std::string>();
                const YAML::Node& pose_data = pose_it.second;

                object_pose_msgs::msg::ObjectPose pose_item;
                pose_item.pose.position.x = pose_data["x"].as<double>();
                pose_item.pose.position.y = pose_data["y"].as<double>();
                pose_item.pose.position.z = pose_data["z"].as<double>();
                pose_item.pose.orientation.x = pose_data["rx"].as<double>();
                pose_item.pose.orientation.y = pose_data["ry"].as<double>();
                pose_item.pose.orientation.z = pose_data["rz"].as<double>();
                pose_item.pose.orientation.w = pose_data["rw"].as<double>();
                pose_item.size.x = pose_data["size_x"].as<double>();
                pose_item.size.y = pose_data["size_y"].as<double>();
                pose_item.size.z = pose_data["size_z"].as<double>();
                pose_item.min.x = pose_data["min_x"].as<double>();
                pose_item.min.y = pose_data["min_y"].as<double>();
                pose_item.min.z = pose_data["min_z"].as<double>();
                pose_item.max.x = pose_data["max_x"].as<double>();
                pose_item.max.y = pose_data["max_y"].as<double>();
                pose_item.max.z = pose_data["max_z"].as<double>();

                std::cmatch m;
                if(std::regex_search(name.c_str(), m, std::regex("(.+)_([0-9]+)")))
                {
                    pose_item.class_id = m[1];
                    pose_item.instance_id = std::stoi(m[2]);
                }else{
                    RCLCPP_INFO_STREAM(this->get_logger(), "Pose " << name << " not named correctly in configuration file");
                continue;
                }

                struct PoseEntry pose_entry(pose_item);

                pose_map_[name] = pose_entry;
            }

            if(debug_) printPoses();
        }
        catch (const YAML::Exception& e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load poses file '" << config_file_path << "'. Error: " << e.what());
        }
    }


//Iterate through the pose_map and print all items and their poses
    void printPoses()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "-------------------------------------------------------------------------------------------");
        for(const auto& elem : pose_map_)
        {
            std::stringstream ss;
            ss << "\nId: " << elem.first << "\n";
            ss << "  Class ID: " << elem.second.class_id << "\n";
            ss << "  Instance ID: " << elem.second.instance << "\n";
            ss << "  Position:\n";
            ss << "    x: " << elem.second.pose_stamped.pose.position.x << "\n";
            ss << "    y: " << elem.second.pose_stamped.pose.position.y << "\n";
            ss << "    z: " << elem.second.pose_stamped.pose.position.z << "\n";
            ss << "  Orientation:\n";
            ss << "    x: " << elem.second.pose_stamped.pose.orientation.x << "\n";
            ss << "    y: " << elem.second.pose_stamped.pose.orientation.y << "\n";
            ss << "    z: " << elem.second.pose_stamped.pose.orientation.z << "\n";
            ss << "    w: " << elem.second.pose_stamped.pose.orientation.w << "\n";
            ss << "  Size:\n";
            ss << "    x: " << elem.second.pose_stamped.size.x << "\n";
            ss << "    y: " << elem.second.pose_stamped.size.y << "\n";
            ss << "    z: " << elem.second.pose_stamped.size.z << "\n";
            ss << "  Min:\n";
            ss << "    x: " << elem.second.pose_stamped.min.x << "\n";
            ss << "    y: " << elem.second.pose_stamped.min.y << "\n";
            ss << "    z: " << elem.second.pose_stamped.min.z << "\n";
            ss << "  Max:\n";
            ss << "    x: " << elem.second.pose_stamped.max.x << "\n";
            ss << "    y: " << elem.second.pose_stamped.max.y << "\n";
            ss << "    z: " << elem.second.pose_stamped.max.z;

            RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
        }
    }

    /// Turn on/off recording
    void activateRecording(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res)
    {
        bool recording_activated = req->data;

        //Activate or deactivate subscriber
        if(recording_activated)
        {
            pose_sub_ = this->create_subscription<object_pose_msgs::msg::ObjectList>("/logical_image", 1, std::bind(&PoseSelector::poseCallback, this, std::placeholders::_1));
            if(debug_) RCLCPP_INFO_STREAM(this->get_logger(), "Pose_selector activated");

        }else{
            pose_sub_.reset();
            if(debug_) RCLCPP_INFO_STREAM(this->get_logger(), "Pose_selector deactivated");
        }

        res->success = true;
    }

    /// Get all current poses stored in pose_selector
    void getAllPoses(const std::shared_ptr<pose_selector::srv::GetPoses::Request> req, std::shared_ptr<pose_selector::srv::GetPoses::Response> res)
    {
        (void)req;
        object_pose_msgs::msg::ObjectList pose_list;

        for(const auto& elem : pose_map_)
        {
            pose_list.objects.push_back(elem.second.pose_stamped);
        }

        pose_list.header.frame_id = global_reference_frame_;

        res->poses = pose_list;
    }

    void clearPoseSelector(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        (void)req;
        pose_map_.clear();
        res->success = true;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSelector>();
    node->loadPoses();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
