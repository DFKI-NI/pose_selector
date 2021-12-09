#include "ros/ros.h"
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <pose_selector/PoseQuery.h>
#include <pose_selector/ClassQuery.h>
#include <pose_selector/PoseUpdate.h>
#include <pose_selector/PoseDelete.h>
#include <pose_selector/ConfigSave.h>

struct PoseEntry
{
    std::string class_id;
    int instance;
    geometry_msgs::PoseStamped pose_stamped;

    PoseEntry(){};

    PoseEntry(std::string id_ , int instance_id_ , geometry_msgs::PoseStamped pose_msg_) : class_id(id_), instance(instance_id_), pose_stamped(pose_msg_) {}

};

class PoseSelector
{
    private:
    bool debug_;
    std::string frame_id_;
    ros::ServiceServer query_service;
    ros::ServiceServer class_query_service;
    ros::ServiceServer update_service;
    ros::ServiceServer delete_service;
    ros::ServiceServer save_service;
    std::map<std::string,PoseEntry> pose_map;

    public:
    PoseSelector(ros::NodeHandle *nh)
    {
        ros::NodeHandle pn("~");
        pn.param("debug", debug_, false);
        pn.param<std::string>("frame_id", frame_id_, "map");

        query_service = nh->advertiseService("/pose_selector_query", &PoseSelector::callback_pose_query, this);
        class_query_service = nh->advertiseService("/pose_selector_class_query", &PoseSelector::callback_class_query, this);
        update_service = nh->advertiseService("/pose_selector_update", &PoseSelector::callback_pose_update, this);
        delete_service = nh->advertiseService("/pose_selector_delete", &PoseSelector::callback_pose_delete, this);
        save_service = nh->advertiseService("/pose_selector_save", &PoseSelector::callback_save, this);
    }

    //Service to query for an item ID and to return the pose of the item
    bool callback_pose_query(pose_selector::PoseQuery::Request &req, pose_selector::PoseQuery::Response &res)
    {
        std::string item_id = req.class_id + "_" + std::to_string(req.instance_id);
        
        if(debug_) ROS_INFO_STREAM("Pose query service call: " << item_id);

        std::map<std::string, PoseEntry>::iterator itr = pose_map.find(item_id);

        if(itr != pose_map.end())
        {
            res.pose_query_result = itr->second.pose_stamped;
        }else{
            ROS_ERROR_STREAM(item_id << " does not exist!");
        }

        return true;
    }

    //Service to query all items of a certain class and to return the poses of the items
    bool callback_class_query(pose_selector::ClassQuery::Request &req, pose_selector::ClassQuery::Response &res)
    {
        std::string class_id = req.class_id;

        if(debug_) ROS_INFO_STREAM("Class query service call: " << class_id);

        std::vector<geometry_msgs::PoseStamped> srv_result;

        for (const auto& [key, value] : pose_map)
        {
            if (value.class_id == class_id)
            {
                srv_result.push_back(value.pose_stamped);
            }
        }

        res.pose_query_result = srv_result;

        return true;
    }

    //Service to update/add the pose of an item
    bool callback_pose_update(pose_selector::PoseUpdate::Request &req, pose_selector::PoseUpdate::Response &res)
    {
        
        std::string item_id = req.class_id + "_" + std::to_string(req.instance_id);
        
        if(debug_) ROS_INFO_STREAM("Update pose service call: " << item_id);

        pose_map.insert_or_assign(item_id, PoseEntry(req.class_id, req.instance_id, req.pose_update));

        if(debug_) print_poses();

        return true;
    }

    //Service to delete an item
    bool callback_pose_delete(pose_selector::PoseDelete::Request &req, pose_selector::PoseDelete::Response &res)
    {
        
        std::string item_id = req.class_id + "_" + std::to_string(req.instance_id);
        
        if(debug_) ROS_INFO_STREAM("Delete pose service call: " << item_id);

        pose_map.erase(item_id);

        if(debug_) print_poses();

        return true;
    }

    //Save parameters to yaml file
    bool callback_save(pose_selector::ConfigSave::Request &req, pose_selector::ConfigSave::Response &res)
    {
        ros::NodeHandle pn("~");

        for (const auto& [key, value] : pose_map)
        {
            pn.setParam("poses/"+key+"/class",value.class_id);
            pn.setParam("poses/"+key+"/instance",value.instance);
            pn.setParam("poses/"+key+"/rw",value.pose_stamped.pose.orientation.w);
            pn.setParam("poses/"+key+"/rx",value.pose_stamped.pose.orientation.x);
            pn.setParam("poses/"+key+"/ry",value.pose_stamped.pose.orientation.y);
            pn.setParam("poses/"+key+"/rz",value.pose_stamped.pose.orientation.z);
            pn.setParam("poses/"+key+"/x",value.pose_stamped.pose.position.x);
            pn.setParam("poses/"+key+"/y",value.pose_stamped.pose.position.y);
            pn.setParam("poses/"+key+"/z",value.pose_stamped.pose.position.z);
        }

        std::string save_dir = ros::package::getPath("pose_selector") + "/config/" + req.file_name + ".yaml";
        std::string command = "rosparam dump " + save_dir + " " + pn.getNamespace();
        system(command.c_str());
        return true;
    }

    //Load the poses from a yaml file
    void load_poses()
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
                ROS_ASSERT(i->second["class"].getType()==XmlRpc::XmlRpcValue::TypeString);
                ROS_ASSERT(i->second["instance"].getType()==XmlRpc::XmlRpcValue::TypeInt);
                
                geometry_msgs::PoseStamped pose_item;
                pose_item.pose.position.x = i->second["x"];
                pose_item.pose.position.y = i->second["y"];
                pose_item.pose.position.z = i->second["z"];
                pose_item.pose.orientation.x = i->second["rx"];
                pose_item.pose.orientation.y = i->second["ry"];
                pose_item.pose.orientation.z = i->second["rz"];
                pose_item.pose.orientation.w = i->second["rw"];
                struct PoseEntry pose_entry(i->second["class"],i->second["instance"],pose_item);

                pose_map[i->first] = pose_entry;
            }

            if(debug_) print_poses();
        }

        pn.deleteParam("poses");
    }

    //Iterate through the pose_map and print all items and their poses
    void print_poses()
    {
        ROS_INFO_STREAM("-------------------------------------------------------------------------------------------");
        for(const auto& elem : pose_map)
        {
            ROS_INFO_STREAM("\nId: " << elem.first << " \nClass: " << elem.second.class_id << "\nInstance: " << elem.second.instance << "\n" << elem.second.pose_stamped);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_selector_server");
    ros::NodeHandle nh;
    PoseSelector pc = PoseSelector(&nh);
    pc.load_poses();

    ros::spin();

    return 0;
}