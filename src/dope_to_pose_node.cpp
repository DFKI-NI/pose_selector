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
#include "object_pose_msgs/msg/object_list.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include <yaml-cpp/yaml.h>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <functional>

class DopeConverter : public rclcpp::Node
{
    private:
    bool debug_;
    std::map<std::string, std::string> dope_ids_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr dope_sub_;
    rclcpp::Publisher<object_pose_msgs::msg::ObjectList>::SharedPtr converter_pub_;
    std::mutex connect_mutex_;

    public:
    DopeConverter()
    : Node("dope_converter_node", rclcpp::NodeOptions().allow_undeclared_parameters(true))
    {    
        this->declare_parameter("debug", false);
        this->get_parameter("debug", debug_);
        this->declare_parameter("config_path", "");
        std::string config_file_path = this->get_parameter("config_path").as_string();

        if (config_file_path.empty())
        {
            if (debug_)
                RCLCPP_WARN_STREAM(this->get_logger(), "config_path parameter is not set. No DOPE class IDs will be loaded from a file.");
        }
        else
        {
            try
            {
                YAML::Node config_yaml = YAML::LoadFile(config_file_path);
                if (config_yaml["class_ids"])
                {
                    for (YAML::const_iterator it = config_yaml["class_ids"].begin(); it != config_yaml["class_ids"].end(); ++it)
                    {
                        std::string class_name = it->first.as<std::string>();
                        int dope_id = it->second.as<int>();
                        dope_ids_[std::to_string(dope_id)] = class_name;
                    }
                }
                else
                {
                    if (debug_)
                        RCLCPP_WARN_STREAM(this->get_logger(), "The YAML file '" << config_file_path << "' does not contain a 'class_ids' map.");
                }
            }
            catch (const YAML::Exception& e)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load configuration file '" << config_file_path << "'. Error: " << e.what());
            }
        }

        if(debug_)
        {
            for(auto const& [key, val] : dope_ids_)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Dope ID: " << key << " Label: " << val);
            }
        }

        rclcpp::PublisherOptions pub_options;
        pub_options.event_callbacks.matched_callback = 
            [this](const rclcpp::MatchedInfo& info)
        {
            if(debug_) RCLCPP_INFO_STREAM(this->get_logger(), "Dope Converter Connect Callback Called");
            std::lock_guard<std::mutex> lock(connect_mutex_);
            if(info.current_count == 0)
            {
                if(debug_) RCLCPP_INFO_STREAM(this->get_logger(), "No DOPE converter subscribers, shutting down DOPE subscriber");
                dope_sub_.reset();
            } else if (!dope_sub_)
            {
                if(debug_) RCLCPP_INFO_STREAM(this->get_logger(), "Starting up DOPE subscriber");
                dope_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
                    "/dope_output",
                    1000,
                    std::bind(&DopeConverter::dopeCallback, this, std::placeholders::_1));
            }
        };
        
        converter_pub_ = this->create_publisher<object_pose_msgs::msg::ObjectList>("/dope_converter_poses", 1000, pub_options);
    }

    void dopeCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        object_pose_msgs::msg::ObjectList converted_msg;

        converted_msg.header = msg->header;

        std::vector<object_pose_msgs::msg::ObjectPose> converted_poses;

        for(auto i: msg->detections)
        {
            if (i.results.empty())
            {
                continue;
            }

            object_pose_msgs::msg::ObjectPose new_pose;

            //search for object label based on dope ID
            std::map<std::string,std::string>::iterator itr = dope_ids_.find(i.results[0].hypothesis.class_id);

            if(itr != dope_ids_.end())
            {
                new_pose.class_id = itr->second;
            }else{
                RCLCPP_ERROR_STREAM(this->get_logger(), i.results[0].hypothesis.class_id << " does not exist!");
            }

            //set instance id to zero for situations where only one instance per class is present
            ///TODO: Update this if necessary
            new_pose.instance_id = 1;

            //save object pose and bbox size
            new_pose.pose = i.results[0].pose.pose;
            new_pose.size = i.bbox.size;

            converted_poses.push_back(new_pose);
        }

        converted_msg.objects = converted_poses;

        converter_pub_->publish(converted_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DopeConverter>());
    rclcpp::shutdown();
    return 0;
}
