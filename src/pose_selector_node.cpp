#include "ros/ros.h"
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <pose_selector/PoseQuery.h>
#include <pose_selector/ClassQuery.h>
#include <pose_selector/PoseUpdate.h>
#include <pose_selector/PoseDelete.h>
#include <pose_selector/ConfigSave.h>
#include <object_pose_msgs/ObjectList.h>
#include <regex>

struct PoseEntry
{
    std::string class_id;
    int instance;
    object_pose_msgs::ObjectPose pose_stamped;

    PoseEntry(){};

    PoseEntry(object_pose_msgs::ObjectPose pose_msg_){
        class_id = pose_msg_.class_id;
        instance = pose_msg_.instance_id;
        pose_stamped = pose_msg_;
    }

};

class PoseSelector
{
    private:
    bool debug_;
    bool recording_enabled_;
    ros::ServiceServer query_service_;
    ros::ServiceServer class_query_service_;
    ros::ServiceServer update_service_;
    ros::ServiceServer delete_service_;
    ros::ServiceServer save_service_;
    ros::ServiceServer record_activate_service_;
    ros::Subscriber pose_sub_;
    std::map<std::string,PoseEntry> pose_map_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<std::string> objects_of_interest_;

    public:
    PoseSelector(ros::NodeHandle *nh) : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle pn("~");
        pn.param("debug", debug_, false);
        recording_enabled_ = false;
        query_service_ = nh->advertiseService("/pose_selector_query", &PoseSelector::callbackPoseQuery, this);
        class_query_service_ = nh->advertiseService("/pose_selector_class_query", &PoseSelector::callbackClassQuery, this);
        update_service_ = nh->advertiseService("/pose_selector_update", &PoseSelector::callbackPoseUpdate, this);
        delete_service_ = nh->advertiseService("/pose_selector_delete", &PoseSelector::callbackPoseDelete, this);
        save_service_ = nh->advertiseService("/pose_selector_save", &PoseSelector::callbackSave, this);
        record_activate_service_ = nh->advertiseService("/pose_selector_activate", &PoseSelector::activateRecording, this );
        ///TODO: set subscription topic as launch or config parameter 
        pose_sub_ = nh->subscribe("/mobipick/gripper_astra/rgb/logical_image",1,&PoseSelector::poseCallback, this);

        //Parse ROS parameter related to objects of interest (object classes not listed will be ignored)
        XmlRpc::XmlRpcValue v;
        pn.param("objects_of_interest", v, v);

        if (v.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          for(int i =0;i<v.size();i++)
          {
              objects_of_interest_.push_back(v[i]);
          }
        } else {
          ROS_ERROR("Parameter not set, or not an array: %s", pn.resolveName("objects_of_interest").c_str());
        }
    }

    //Service to query for an item ID and to return the pose of the item
    bool callbackPoseQuery(pose_selector::PoseQuery::Request &req, pose_selector::PoseQuery::Response &res)
    {
        std::string item_id = req.class_id + "_" + std::to_string(req.instance_id);
        
        if(debug_) ROS_INFO_STREAM("Pose query service call: " << item_id);

        std::map<std::string, PoseEntry>::iterator itr = pose_map_.find(item_id);

        if(itr != pose_map_.end())
        {
            res.pose_query_result = itr->second.pose_stamped;
        }else{
            ROS_ERROR_STREAM(item_id << " does not exist!");
        }

        return true;
    }

    //Service to query all items of a certain class and to return the poses and ids of the items
    bool callbackClassQuery(pose_selector::ClassQuery::Request &req, pose_selector::ClassQuery::Response &res)
    {
        std::string class_id = req.class_id;

        if(debug_) ROS_INFO_STREAM("Class query service call: " << class_id);

        std::vector<object_pose_msgs::ObjectPose> pose_result;

        for (const auto& [key, value] : pose_map_)
        {
            if (value.class_id == class_id)
            {
                pose_result.push_back(value.pose_stamped);
            }
        }

        res.poses = pose_result;

        return true;
    }

    //Service to update one or more poses
    bool callbackPoseUpdate(pose_selector::PoseUpdate::Request &req, pose_selector::PoseUpdate::Response &res)
    {
        
        updatePoses(req.poses);

        if(debug_) printPoses();

        return true;
    }

    void updatePoses(object_pose_msgs::ObjectList object_list)
    {
        ///TODO: Alternative ways to do conversion?
        ///TODO: Should world frame always be used to perform lookup?

        //Perform TF lookup based on object_list.header.frame_id
        std::string reference_tf = object_list.header.frame_id;

        geometry_msgs::TransformStamped camera_to_world_tf;

        try{
            camera_to_world_tf = tf_buffer_.lookupTransform("world",reference_tf,ros::Time(0));
            }
        catch (tf2::TransformException &ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        
        //Convert camera_to_world_tf to tf::Transform
        tf::Transform camera_transform;
        camera_transform.setOrigin(tf::Vector3(camera_to_world_tf.transform.translation.x,
            camera_to_world_tf.transform.translation.y,
            camera_to_world_tf.transform.translation.z));
        camera_transform.setRotation(tf::Quaternion(camera_to_world_tf.transform.rotation.x,
            camera_to_world_tf.transform.rotation.y,
            camera_to_world_tf.transform.rotation.z,
            camera_to_world_tf.transform.rotation.w));

        //Iterate through each object detected
        for (auto i: object_list.objects)
        {
            //Check if current object class is not an object of interest
            if(objects_of_interest_.size()>0 && std::find(objects_of_interest_.begin(), objects_of_interest_.end(), i.class_id) == objects_of_interest_.end()){
                if(debug_) ROS_INFO_STREAM("Class: " << i.class_id << " not of interest, ignoring associated pose");
                continue;
            }

            //Convert object pose to tf::Transform
            geometry_msgs::Point obj_pos = i.pose.position;
            geometry_msgs::Quaternion obj_orient = i.pose.orientation;
            tf::Transform obj_transform;
            obj_transform.setOrigin(tf::Vector3(obj_pos.x,obj_pos.y,obj_pos.z));
            obj_transform.setRotation(tf::Quaternion(obj_orient.x,obj_orient.y,obj_orient.z,obj_orient.w));

            //Calculate resultant transform from reference to object
            obj_transform = camera_transform*obj_transform;
            tf::Vector3 final_position = obj_transform.getOrigin();
            tf::Quaternion final_orientation = obj_transform.getRotation();

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

    void poseCallback(object_pose_msgs::ObjectList object_list)
    {
        if(recording_enabled_)
        {
            updatePoses(object_list);
            if(debug_) printPoses();
        }
    }

    //Service to delete an item
    bool callbackPoseDelete(pose_selector::PoseDelete::Request &req, pose_selector::PoseDelete::Response &res)
    {
        
        std::string item_id = req.class_id + "_" + std::to_string(req.instance_id);
        
        if(debug_) ROS_INFO_STREAM("Delete pose service call: " << item_id);

        pose_map_.erase(item_id);

        if(debug_) printPoses();

        return true;
    }

    //Save parameters to yaml file
    bool callbackSave(pose_selector::ConfigSave::Request &req, pose_selector::ConfigSave::Response &res)
    {
        ros::NodeHandle pn("~");

        for (const auto& [key, value] : pose_map_)
        {
            pn.setParam("poses/"+key+"/rw",value.pose_stamped.pose.orientation.w);
            pn.setParam("poses/"+key+"/rx",value.pose_stamped.pose.orientation.x);
            pn.setParam("poses/"+key+"/ry",value.pose_stamped.pose.orientation.y);
            pn.setParam("poses/"+key+"/rz",value.pose_stamped.pose.orientation.z);
            pn.setParam("poses/"+key+"/x",value.pose_stamped.pose.position.x);
            pn.setParam("poses/"+key+"/y",value.pose_stamped.pose.position.y);
            pn.setParam("poses/"+key+"/z",value.pose_stamped.pose.position.z);
            pn.setParam("poses/"+key+"/size_x",value.pose_stamped.size.x);
            pn.setParam("poses/"+key+"/size_y",value.pose_stamped.size.y);
            pn.setParam("poses/"+key+"/size_z",value.pose_stamped.size.z);
            pn.setParam("poses/"+key+"/min_x",value.pose_stamped.min.x);
            pn.setParam("poses/"+key+"/min_y",value.pose_stamped.min.y);
            pn.setParam("poses/"+key+"/min_z",value.pose_stamped.min.z);
            pn.setParam("poses/"+key+"/max_x",value.pose_stamped.max.x);
            pn.setParam("poses/"+key+"/max_y",value.pose_stamped.max.y);
            pn.setParam("poses/"+key+"/max_z",value.pose_stamped.max.z);
        }

        std::string save_dir = ros::package::getPath("pose_selector") + "/config/" + req.file_name + ".yaml";
        std::string command = "rosparam dump " + save_dir + " " + pn.getNamespace();
        system(command.c_str());
        return true;
    }

    //Load the poses from a yaml file
    void loadPoses()
    {
        ros::NodeHandle pn("~");

        XmlRpc::XmlRpcValue poses_list;
        
        if (!pn.getParam("poses",poses_list))
        {
            ROS_ERROR_STREAM("Failed to get initial poses");
        }else
        {
            //Check that main poses parameter is a structure
            ROS_ASSERT(poses_list.getType()==XmlRpc::XmlRpcValue::TypeStruct);

            for (XmlRpc::XmlRpcValue::iterator i = poses_list.begin(); i != poses_list.end(); i++)
            {
                //Check that sub-parameters in poses are also structures
                ROS_ASSERT(i->second.getType()==XmlRpc::XmlRpcValue::TypeStruct);

                //Check that pose, class, and instance sub-parameters are correct format
                ROS_ASSERT(i->second["x"].getType()==XmlRpc::XmlRpcValue::TypeDouble);
                ROS_ASSERT(i->second["y"].getType()==XmlRpc::XmlRpcValue::TypeDouble);
                ROS_ASSERT(i->second["z"].getType()==XmlRpc::XmlRpcValue::TypeDouble);
                ROS_ASSERT(i->second["rx"].getType()==XmlRpc::XmlRpcValue::TypeDouble);
                ROS_ASSERT(i->second["ry"].getType()==XmlRpc::XmlRpcValue::TypeDouble);
                ROS_ASSERT(i->second["rz"].getType()==XmlRpc::XmlRpcValue::TypeDouble);
                ROS_ASSERT(i->second["rw"].getType()==XmlRpc::XmlRpcValue::TypeDouble);
                
                object_pose_msgs::ObjectPose pose_item;
                
                pose_item.pose.position.x = i->second["x"];
                pose_item.pose.position.y = i->second["y"];
                pose_item.pose.position.z = i->second["z"];
                pose_item.pose.orientation.x = i->second["rx"];
                pose_item.pose.orientation.y = i->second["ry"];
                pose_item.pose.orientation.z = i->second["rz"];
                pose_item.pose.orientation.w = i->second["rw"];
                pose_item.size.x = i->second["size_x"];
                pose_item.size.y = i->second["size_y"];
                pose_item.size.z = i->second["size_z"];
                pose_item.min.x = i->second["min_x"];
                pose_item.min.y = i->second["min_y"];
                pose_item.min.z = i->second["min_z"];
                pose_item.max.x = i->second["max_x"];
                pose_item.max.y = i->second["max_y"];
                pose_item.max.z = i->second["max_z"];

                std::cmatch m;
                if(std::regex_search(i->first.c_str(), m, std::regex("(.+)_([0-9]+)")))
                {
                    pose_item.class_id = m[1];
                    pose_item.instance_id = std::stoi(m[2]);
                }else{
                    ROS_INFO_STREAM("Pose " << i->first << " not named correctly in configuration file");
                continue;
                }

                struct PoseEntry pose_entry(pose_item);

                pose_map_[i->first] = pose_entry;
            }

            if(debug_) printPoses();

        pn.deleteParam("poses");
        }

    }

    //Iterate through the pose_map and print all items and their poses
    void printPoses()
    {
        ROS_INFO_STREAM("-------------------------------------------------------------------------------------------");
        for(const auto& elem : pose_map_)
        {
            ROS_INFO_STREAM("\nId: " << elem.first << "\n" << elem.second.pose_stamped);
        }
    }

    /// Turn on/off recording 
    bool activateRecording(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        recording_enabled_ = req.data;
        res.success = true;
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_selector_server");
    ros::NodeHandle nh;
    PoseSelector pc = PoseSelector(&nh);
    pc.loadPoses();

    ros::spin();

    return 0;
}