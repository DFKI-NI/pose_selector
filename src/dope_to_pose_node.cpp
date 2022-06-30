#include "ros/ros.h"
#include <ros/package.h>
#include <object_pose_msgs/ObjectList.h>
#include <vision_msgs/Detection3DArray.h>

class DopeConverter
{
    private:
    bool debug_;
    std::map<int,std::string> dope_ids_;
    ros::Subscriber dope_sub_;
    ros::Publisher converter_pub_;

    public:
    DopeConverter(ros::NodeHandle *nh)
    {
        ros::NodeHandle pn("~");
        pn.param("debug", debug_, false);

        //load in dope class ids
        XmlRpc::XmlRpcValue v;
        pn.param("class_ids",v,v);
        ROS_ASSERT(v.getType()==XmlRpc::XmlRpcValue::TypeStruct);

        for(XmlRpc::XmlRpcValue::iterator i = v.begin(); i!=v.end(); ++i)
        {
            ROS_ASSERT(i->second.getType()==XmlRpc::XmlRpcValue::TypeInt);
            dope_ids_[i->second] = i->first;
        }

        if(debug_)
        {
            std::map<int,std::string>::iterator it;
            for(it=dope_ids_.begin(); it!=dope_ids_.end(); ++ it)
            {
                ROS_INFO_STREAM("Dope ID: " << it->first << " Label: " << it->second);
            }
        }

        //subscriber to dope output
        dope_sub_ = nh->subscribe("/dope_output",1,&DopeConverter::dopeCallback,this);

        //converted messages to be published 
        converter_pub_ = nh->advertise<object_pose_msgs::ObjectList>("/dope_converter_poses",1000);
    }

    void dopeCallback(const vision_msgs::Detection3DArray::ConstPtr& msg)
    {
        object_pose_msgs::ObjectList converted_msg;

        converted_msg.header = msg->header;

        std::vector<object_pose_msgs::ObjectPose> converted_poses;

        for(auto i: msg->detections)
        {
            object_pose_msgs::ObjectPose new_pose;

            //search for object label based on dope ID
            std::map<int,std::string>::iterator itr = dope_ids_.find(i.results[0].id);

            if(itr != dope_ids_.end())
            {
                new_pose.class_id = itr->second;
            }else{
                ROS_ERROR_STREAM(i.results[0].id << " does not exist!");
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

        converter_pub_.publish(converted_msg);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dope_to_object_pose_msgs_converter");
    ros::NodeHandle nh;

    DopeConverter node = DopeConverter(&nh);
    
    ros::spin();

    return 0;
}
